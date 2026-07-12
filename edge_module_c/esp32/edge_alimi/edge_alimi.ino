/*
 * edge_alimi.ino — 엣지알리미 ESP32 펌웨어 (실무 투입판)
 *
 * 3계층 구조에서 "계산기" 역할만 수행:
 *   센서 읽기 → FFT → 특징 추출 → 3σ 판정 → 결과(JSON)만 송출
 *
 * 하드웨어: ESP32 DevKit + MPU-6050 (I2C: SDA=21, SCL=22)
 *           ALARM_PIN → 릴레이/부저 (기존 경보반 무전압 접점 연동 가능)
 *
 * [실무 기능]
 *  - baseline NVS 영속화: 정전·재부팅 후 재학습 없이 감시 즉시 재개
 *  - 가동/정지 자동 판별: 설비를 꺼도 오경보하지 않음 (코어에서 처리)
 *  - 센서 장애 격리: I2C 실패/클리핑 감지 → 판정 대신 SENSOR_FAULT 보고
 *    (센서 고장을 설비 이상으로 오인하지 않음)
 *  - Wi-Fi 자동 재연결, 수집 루프와 HTTP 분리(샘플링 지터 방지)
 *
 * HTTP API:
 *   GET  /                 상태 요약
 *   GET  /data             최신 판정 JSON (대시보드 폴링)
 *   GET  /health           가동시간·힙·I2C 오류·RSSI (현장 점검용)
 *   POST /learn?n=120      정상 baseline 학습 시작 (완료 시 NVS 자동 저장)
 *   POST /config?rpm=3000  명판 정격 RPM 변경 (baseline 무효화 + 재학습 필요)
 *   POST /baseline/clear   저장된 baseline 삭제
 *
 * 시리얼 명령: 'l' 학습 / 'r<rpm>' 정격 변경 / 'c' baseline 삭제
 *
 * [정직성 노트]
 *  - MPU-6050 + I2C(400kHz) 실효 샘플링 상한 ~1kHz → 관측 대역 ~500Hz.
 *    타겟은 저주파 결함(불평형/정렬불량/이완). 베어링 정밀 진단 아님.
 *  - HTTP 응답은 윈도 수집(~1초) 사이사이에 처리되어 최대 ~1초 지연될
 *    수 있음. 샘플링 균일성이 응답성보다 우선한다는 의도된 선택.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

extern "C" {
#include "em_pipeline.h"
}
#include "dashboard_html.h"   /* tools/embed_dashboard.py 가 생성 */

/* ================= 사용자 설정 ================= */
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_PASSWORD";

#define RATED_RPM_DEFAULT  3000.0f   /* 설비 명판 정격 RPM — 유일한 필수 설정 */
#define SAMPLE_RATE_HZ     1000.0f   /* 목표 샘플링 (실측치로 보정됨) */
#define LEARN_WINDOWS      120       /* 정상 baseline 수집 윈도 수 (~2분) */
#define ACCEL_AXIS         0         /* 0=X(방사방향 권장), 1=Y, 2=Z */
#define ALARM_PIN          2         /* is_anomaly 시 HIGH (릴레이/LED) */
/* =============================================== */

/* ---- MPU-6050 최소 드라이버 (라이브러리 의존 제거) ---- */
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B
#define ACCEL_CLIP_RAW  32200        /* ±4g 풀스케일(32767) 근접 = 클리핑 */

static uint32_t g_i2c_err_total = 0; /* 수명 누적 (health 용) */

static bool mpu_write(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool mpu_init()
{
    Wire.begin(21, 22);
    Wire.setClock(400000);           /* I2C fast mode — 1kHz 샘플링 전제 */
    Wire.setTimeOut(5);              /* 배선 불량 시 5ms 내 실패 처리 */
    bool ok = true;
    ok &= mpu_write(REG_PWR_MGMT_1, 0x00); /* 슬립 해제 */
    delay(50);
    ok &= mpu_write(REG_CONFIG, 0x00);     /* DLPF 260Hz — 대역 최대 확보 */
    ok &= mpu_write(REG_ACCEL_CFG, 0x08);  /* ±4g */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);                /* WHO_AM_I */
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    return ok && Wire.available() && (Wire.read() == 0x68);
}

/* 단일 축 가속도 읽기. 반환: 성공 여부. raw_out 에 원시값 기록 */
static bool mpu_read_accel(float *g_out, int16_t *raw_out)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT + ACCEL_AXIS * 2);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    *raw_out = raw;
    *g_out = (float)raw / 8192.0f;   /* ±4g → 8192 LSB/g */
    return true;
}

/* ---- 전역 상태 ---- */
static em_pipeline_t g_pipe;
static em_output_t   g_last;
static char          g_json[640] = "{\"mode\":0,\"sfault\":0}";
static float         g_sig[EM_FFT_SIZE];
static WebServer     server(80);
static Preferences   prefs;
static bool          g_sensor_fault = false;
static bool          g_clip = false;
static uint32_t      g_boot_ms = 0;

/* ---- baseline NVS 영속화 ---- */
static void baseline_persist()
{
    em_snapshot_t snap;
    em_pipeline_save_baseline(&g_pipe, &snap);
    prefs.begin("edgealimi", false);
    prefs.putBytes("baseline", &snap, sizeof(snap));
    prefs.putFloat("rpm", g_pipe.cfg.rated_rpm);
    prefs.end();
    Serial.println("{\"event\":\"baseline_saved\"}");
}

static bool baseline_restore()
{
    em_snapshot_t snap;
    prefs.begin("edgealimi", true);
    size_t n = prefs.getBytesLength("baseline");
    bool ok = false;
    float saved_rpm = prefs.getFloat("rpm", 0.0f);
    if (n == sizeof(snap) && prefs.getBytes("baseline", &snap, n) == n) {
        /* 저장 당시 정격과 현재 정격이 다르면 baseline 은 무의미 → 거부 */
        if (saved_rpm > 0.0f
            && fabsf(saved_rpm - g_pipe.cfg.rated_rpm) < 0.5f)
            ok = em_pipeline_load_baseline(&g_pipe, &snap);
    }
    prefs.end();
    return ok;
}

static void baseline_clear()
{
    prefs.begin("edgealimi", false);
    prefs.remove("baseline");
    prefs.remove("rpm");
    prefs.end();
}

static void apply_rated_rpm(float rpm)
{
    /* 정격 변경 = 다른 설비 = 기존 baseline 무효.
     * 탐색범위/고조파 기준 재파생 + 상태 초기화 + 저장분 삭제. */
    float fs = g_pipe.cfg.sample_rate_hz;
    em_pipeline_init(&g_pipe, rpm);
    g_pipe.cfg.sample_rate_hz = fs;
    baseline_clear();
}

/* ---- HTTP 핸들러 ---- */
static void send_cors() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
}

/* 루트 = 감시 콘솔. 현장 사람은 브라우저에 ESP32 주소만 치면 된다.
 * same-origin 서빙이므로 CORS/mixed-content 문제가 원천적으로 없다. */
static void handle_ui()
{
    server.sendHeader("Content-Encoding", "gzip");
    server.send_P(200, "text/html",
                  (const char *)DASHBOARD_HTML_GZ, DASHBOARD_HTML_GZ_LEN);
}

/* PWA 자산: 폰 홈 화면 설치 지원 (manifest + 아이콘).
 * 서비스 워커는 http LAN 이 보안 컨텍스트가 아니라 등록되지 않으므로
 * ESP32 는 sw.js 를 제공하지 않는다 — 설치·전체화면 실행에는 불필요. */
static void handle_manifest()
{
    server.sendHeader("Content-Encoding", "gzip");
    server.send_P(200, "application/manifest+json",
                  (const char *)MANIFEST_JSON_GZ, MANIFEST_JSON_GZ_LEN);
}
static void handle_icon192()
{
    server.send_P(200, "image/png",
                  (const char *)ICON_192_PNG, ICON_192_PNG_LEN);
}
static void handle_icon_apple()
{
    server.send_P(200, "image/png",
                  (const char *)ICON_APPLE_PNG, ICON_APPLE_PNG_LEN);
}

static void handle_status()
{
    send_cors();
    String s = "EdgeAlimi | mode=";
    s += (int)g_pipe.mode;
    s += " | rated_rpm=";
    s += g_pipe.cfg.rated_rpm;
    s += " | fs=";
    s += g_pipe.cfg.sample_rate_hz;
    s += " | sensor_fault=";
    s += g_sensor_fault ? 1 : 0;
    s += "\nGET / (console) /data /health, POST /learn?n=120 /config?rpm=3000 /baseline/clear\n";
    server.send(200, "text/plain", s);
}

static void handle_data()
{
    send_cors();
    server.send(200, "application/json", g_json);
}

static void handle_health()
{
    send_cors();
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"uptime_s\":%lu,\"heap\":%u,\"i2c_err\":%lu,"
        "\"rssi\":%d,\"fs\":%.1f,\"seq\":%lu}",
        (unsigned long)((millis() - g_boot_ms) / 1000),
        (unsigned)ESP.getFreeHeap(),
        (unsigned long)g_i2c_err_total,
        WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
        (double)g_pipe.cfg.sample_rate_hz,
        (unsigned long)g_pipe.window_count);
    server.send(200, "application/json", buf);
}

static void handle_learn()
{
    send_cors();
    if (g_sensor_fault) {
        server.send(409, "application/json",
                    "{\"ok\":false,\"reason\":\"sensor_fault\"}");
        return;
    }
    int n = server.hasArg("n") ? server.arg("n").toInt() : LEARN_WINDOWS;
    em_pipeline_start_learning(&g_pipe, n);
    server.send(200, "application/json", "{\"ok\":true,\"learning\":true}");
}

static void handle_config()
{
    send_cors();
    if (server.hasArg("rpm")) {
        float rpm = server.arg("rpm").toFloat();
        if (rpm > 0) {
            apply_rated_rpm(rpm);
            server.send(200, "application/json",
                        "{\"ok\":true,\"note\":\"baseline cleared, relearn required\"}");
            return;
        }
    }
    server.send(400, "application/json", "{\"ok\":false}");
}

static void handle_baseline_clear()
{
    send_cors();
    baseline_clear();
    em_detector_init(&g_pipe.det);
    g_pipe.mode = EM_MODE_IDLE;
    server.send(200, "application/json", "{\"ok\":true}");
}

/* ---- Wi-Fi 자동 재연결 ---- */
static void wifi_maintain()
{
    static uint32_t last_try = 0;
    if (WiFi.status() == WL_CONNECTED) return;
    if (millis() - last_try < 10000) return;   /* 10초 간격 재시도 */
    last_try = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    g_boot_ms = millis();

    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(ALARM_PIN, LOW);

    g_sensor_fault = !mpu_init();
    if (g_sensor_fault)
        Serial.println("{\"error\":\"MPU-6050 not found\"}");

    em_pipeline_init(&g_pipe, RATED_RPM_DEFAULT);
    g_pipe.cfg.sample_rate_hz = SAMPLE_RATE_HZ;

    /* 재부팅 복원: 저장된 baseline 있으면 재학습 없이 감시 재개 */
    if (!g_sensor_fault && baseline_restore())
        Serial.println("{\"event\":\"baseline_restored\",\"mode\":\"monitoring\"}");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("{\"ip\":\""); Serial.print(WiFi.localIP()); Serial.println("\"}");
    }

    server.on("/",              HTTP_GET,  handle_ui);
    server.on("/status",        HTTP_GET,  handle_status);
    server.on("/manifest.json",  HTTP_GET,  handle_manifest);
    server.on("/icon-192.png",   HTTP_GET,  handle_icon192);
    server.on("/icon-512.png",   HTTP_GET,  handle_icon192);  /* 192 재사용 */
    server.on("/apple-touch-icon.png", HTTP_GET, handle_icon_apple);
    server.on("/data",          HTTP_GET,  handle_data);
    server.on("/health",        HTTP_GET,  handle_health);
    server.on("/learn",         HTTP_POST, handle_learn);
    server.on("/learn",         HTTP_GET,  handle_learn);   /* 브라우저 테스트 편의 */
    server.on("/config",        HTTP_POST, handle_config);
    server.on("/config",        HTTP_GET,  handle_config);
    server.on("/baseline/clear",HTTP_POST, handle_baseline_clear);
    server.begin();
}

/*
 * 윈도 하나 수집 — micros() 페이싱으로 균일 샘플링.
 * HTTP 는 수집 중 처리하지 않는다 (샘플 간격 지터 → 스펙트럼 왜곡 방지).
 * 반환: 실측 fs. bad_out: I2C 실패 샘플 수, clip_out: 클리핑 샘플 수.
 */
static float acquire_window(int *bad_out, int *clip_out)
{
    const uint32_t period_us = (uint32_t)(1000000.0f / SAMPLE_RATE_HZ);
    uint32_t next = micros();
    uint32_t t_start = next;
    int bad = 0, clip = 0;
    float last_good = 0.0f;
    for (int i = 0; i < EM_FFT_SIZE; i++) {
        while ((int32_t)(micros() - next) < 0) { }
        float v; int16_t raw;
        if (mpu_read_accel(&v, &raw)) {
            last_good = v;
            if (raw >= ACCEL_CLIP_RAW || raw <= -ACCEL_CLIP_RAW) clip++;
        } else {
            bad++;
            g_i2c_err_total++;
            v = last_good;   /* 단발 실패는 직전값 유지 (스파이크 방지) */
        }
        g_sig[i] = v;
        next += period_us;
    }
    *bad_out = bad;
    *clip_out = clip;
    uint32_t elapsed = micros() - t_start;
    return (float)EM_FFT_SIZE * 1000000.0f / (float)elapsed;
}

/* 판정 JSON 뒤에 하드웨어 상태 필드 부착 */
static void append_hw_fields()
{
    size_t len = strlen(g_json);
    if (len > 1 && g_json[len - 1] == '}') {
        snprintf(g_json + len - 1, sizeof(g_json) - len + 1,
                 ",\"sfault\":%d,\"clip\":%d}",
                 g_sensor_fault ? 1 : 0, g_clip ? 1 : 0);
    }
}

void loop()
{
    server.handleClient();
    wifi_maintain();

    /* 시리얼 명령 */
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'l' && !g_sensor_fault)
            em_pipeline_start_learning(&g_pipe, LEARN_WINDOWS);
        else if (c == 'r') {
            float rpm = Serial.parseFloat();
            if (rpm > 0) apply_rated_rpm(rpm);
        } else if (c == 'c') {
            baseline_clear();
            em_detector_init(&g_pipe.det);
            g_pipe.mode = EM_MODE_IDLE;
        }
    }

    int bad = 0, clip = 0;
    float fs_measured = acquire_window(&bad, &clip);

    /* 센서 장애 격리: 윈도의 2% 이상 I2C 실패 → 이 윈도로 판정 금지.
     * 센서 고장을 설비 이상으로 오인하는 것이 최악의 오경보다. */
    g_sensor_fault = (bad > EM_FFT_SIZE / 50);
    g_clip = (clip > EM_FFT_SIZE / 100);   /* 1% 이상 클리핑 → 스펙트럼 신뢰 불가 */

    if (g_sensor_fault) {
        snprintf(g_json, sizeof(g_json),
                 "{\"seq\":%lu,\"mode\":%d,\"sfault\":1,\"i2c_bad\":%d,"
                 "\"anomaly\":0,\"drift\":0,\"run\":0}",
                 (unsigned long)(g_pipe.window_count + 1),
                 (int)g_pipe.mode, bad);
        digitalWrite(ALARM_PIN, LOW);      /* 센서 장애는 설비 경보 아님 */
        Serial.println(g_json);
        return;
    }

    /* 실측 fs 반영 — FFT bin 축 왜곡 방지 (±2% 이상 벗어날 때만 갱신) */
    if (fabsf(fs_measured - g_pipe.cfg.sample_rate_hz)
        > 0.02f * g_pipe.cfg.sample_rate_hz)
        g_pipe.cfg.sample_rate_hz = fs_measured;

    em_mode_t prev_mode = g_pipe.mode;
    em_pipeline_process(&g_pipe, g_sig, &g_last);
    em_output_to_json(&g_last, g_json, sizeof(g_json));
    append_hw_fields();

    /* 학습 완료 순간 baseline 을 NVS 에 저장 → 재부팅 내성 확보 */
    if (prev_mode == EM_MODE_LEARNING && g_pipe.mode == EM_MODE_MONITORING)
        baseline_persist();

    /* 현장 경보 접점: 확정 이상에서만 구동 (instant_flag 아님) */
    digitalWrite(ALARM_PIN, g_last.verdict.is_anomaly ? HIGH : LOW);

    Serial.println(g_json);   /* 시리얼 백업 채널 (Pi 직결 시에도 사용 가능) */
}
