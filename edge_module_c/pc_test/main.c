/*
 * pc_test/main.c — 하드웨어 없이 C 코어 전체 검증
 *
 * signal_generator.py 와 동일한 개념의 합성 신호로:
 *   [1] 시나리오 A: 장시간 정상 신호에서 is_anomaly 오탐율
 *   [2] 시나리오 B: 결함 주입 시점 → 첫 is_anomaly 까지 감지 지연
 *       (미탐지 발생 시 그대로 기록 — 정직 보고 원칙)
 *   [3] 드리프트: 완만한 열화 시 drift_warning 발화 확인
 *   [4] 재-fit 후 original_means 보존 확인 (gap ⑥ 최우선 점검 항목)
 *   [5] RPM 변동 강건성: 회전수가 ±8% 흔들려도 고조파 대역 추종 확인
 *
 * 빌드: gcc -O2 -Wall -Wextra -o em_test main.c ../core/em_*.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../core/em_pipeline.h"
#include "../core/em_fft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- 합성 신호 생성기 (signal_generator.py 대응) ---------- */
typedef struct {
    float rotation_hz;      /* 실제 회전 주파수 */
    float amp_1x, amp_2x, amp_3x, amp_4x;
    float noise_amp;
    float hf_amp;           /* 고주파 광대역 성분 */
    double phase[4];
    unsigned rng;
} sim_t;

static float frand(unsigned *s)   /* [-1, 1) 균일 */
{
    *s = *s * 1664525u + 1013904223u;
    return ((float)(*s >> 8) / 8388608.0f) - 1.0f;
}

static void sim_init_normal(sim_t *g, float rot_hz, unsigned seed)
{
    memset(g, 0, sizeof(*g));
    g->rotation_hz = rot_hz;
    g->amp_1x = 0.30f;      /* 정상 기저 진동 */
    g->amp_2x = 0.08f;
    g->amp_3x = 0.05f;
    g->amp_4x = 0.03f;
    g->noise_amp = 0.05f;
    g->hf_amp = 0.02f;
    g->rng = seed;
}

static void sim_fill_window(sim_t *g, float fs, float *out, int n)
{
    double dt = 1.0 / (double)fs;
    for (int i = 0; i < n; i++) {
        double s = 0.0;
        double w = 2.0 * M_PI * (double)g->rotation_hz;
        g->phase[0] += w * dt;         s += g->amp_1x * sin(g->phase[0]);
        g->phase[1] += 2.0 * w * dt;   s += g->amp_2x * sin(g->phase[1] + 0.7);
        g->phase[2] += 3.0 * w * dt;   s += g->amp_3x * sin(g->phase[2] + 1.3);
        g->phase[3] += 4.0 * w * dt;   s += g->amp_4x * sin(g->phase[3] + 2.1);
        /* 고주파 성분: 240Hz 부근 협대역 + 백색잡음 */
        s += g->hf_amp * sin(2.0 * M_PI * 240.0 * (double)i * dt
                             + (double)frand(&g->rng) * 0.3);
        s += g->noise_amp * frand(&g->rng);
        out[i] = (float)s;
    }
}

/* 설비 정지: 회전 성분 소멸, 배경 노이즈만 남음 */
static void sim_power_off(sim_t *g)
{
    g->amp_1x = g->amp_2x = g->amp_3x = g->amp_4x = 0.0f;
    g->hf_amp = 0.0f;
}

/* 결함 주입 (테스트베드 실험과 동일한 물리 시나리오) */
static void inject_unbalance(sim_t *g)    { g->amp_1x *= 2.2f; }               /* 1x 급증 */
static void inject_misalignment(sim_t *g) { g->amp_2x *= 3.5f; }               /* 2x 급증 */
static void inject_looseness(sim_t *g)    { g->amp_2x *= 1.8f; g->amp_3x *= 3.0f; g->amp_4x *= 3.0f; }
static void inject_hf(sim_t *g)           { g->hf_amp *= 4.0f; }               /* 고주파 이상 */

/* ---------- 테스트 공통 ---------- */
#define RATED_RPM   3000.0f    /* 50Hz — 임의 예시. 다른 RPM 도 [5]에서 검증 */
#define FS          1000.0f
#define LEARN_WIN   120
#define A_WINDOWS   500

static float sigbuf[EM_FFT_SIZE];

static void run_learning(em_pipeline_t *p, sim_t *g)
{
    em_output_t o;
    em_pipeline_start_learning(p, LEARN_WIN);
    while (p->mode == EM_MODE_LEARNING) {
        sim_fill_window(g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(p, sigbuf, &o);
    }
}

static int fails = 0;
static void check(int ok, const char *msg)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) fails++;
}

/* ---------- [1] 시나리오 A: 오탐율 ---------- */
static void test_scenario_A(void)
{
    printf("\n[1] 시나리오 A — 정상 %d 윈도 오탐율\n", A_WINDOWS);
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 12345u);
    run_learning(&p, &g);

    int fp_instant = 0, fp_confirmed = 0;
    for (int w = 0; w < A_WINDOWS; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.instant_flag) fp_instant++;
        if (o.verdict.is_anomaly)   fp_confirmed++;
    }
    printf("  instant_flag 오탐: %d/%d (%.2f%%) — 확정 전 단독 판정\n",
           fp_instant, A_WINDOWS, 100.0 * fp_instant / A_WINDOWS);
    printf("  is_anomaly   오탐: %d/%d (%.2f%%) — confirm_window 확정 판정\n",
           fp_confirmed, A_WINDOWS, 100.0 * fp_confirmed / A_WINDOWS);
    check(fp_confirmed == 0, "확정 오탐 0건 (오경보 → 현장 신뢰 붕괴 방지)");
}

/* ---------- [2] 시나리오 B: 감지 지연 ---------- */
static void test_scenario_B_one(const char *name, void (*inject)(sim_t *))
{
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 777u);
    run_learning(&p, &g);

    /* 정상 30 윈도 → 결함 주입 → 최대 60 윈도 내 감지 관찰 */
    for (int w = 0; w < 30; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
    }
    inject(&g);
    int latency = -1;
    for (int w = 0; w < 60; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.is_anomaly) { latency = w + 1; break; }
    }
    float win_sec = (float)EM_FFT_SIZE / FS;
    if (latency > 0)
        printf("  %-14s 감지 지연 %2d 윈도 (~%.1f초)  이탈특징 z: "
               "[%.1f %.1f %.1f %.1f %.1f]\n",
               name, latency, latency * win_sec,
               (double)o.verdict.z[0], (double)o.verdict.z[1], (double)o.verdict.z[2],
               (double)o.verdict.z[3], (double)o.verdict.z[4]);
    else
        printf("  %-14s ** 미탐지 (60 윈도 내) — 정직 기록 **\n", name);
    check(latency > 0, name);
}

static void test_scenario_B(void)
{
    printf("\n[2] 시나리오 B — 결함 주입 → 감지 지연 (confirm_window=5, 4/5 확정)\n");
    printf("    * 확정 판정 구조상 최소 %d 윈도의 지연은 설계된 트레이드오프\n", 4);
    test_scenario_B_one("불평형(1x)",     inject_unbalance);
    test_scenario_B_one("정렬불량(2x)",   inject_misalignment);
    test_scenario_B_one("이완(2x/3x/4x)", inject_looseness);
    test_scenario_B_one("고주파 보조",    inject_hf);
}

/* ---------- [3] 드리프트 감지 ---------- */
static void test_drift(void)
{
    printf("\n[3] 드리프트 — 완만한 열화(1x 진폭 서서히 증가)\n");
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 999u);
    run_learning(&p, &g);

    int drift_at = -1;
    for (int w = 0; w < 300; w++) {
        g.amp_1x *= 1.002f;   /* 윈도당 0.2% — 급변 아님 */
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.drift_warning && drift_at < 0) drift_at = w + 1;
    }
    if (drift_at > 0)
        printf("  drift_warning 최초 발화: %d 윈도째 (누적 +%.0f%% 시점)\n",
               drift_at, (pow(1.002, drift_at) - 1.0) * 100.0);
    check(drift_at > 0, "완만한 열화에서 drift_warning 발화");
}

/* ---------- [4] 재-fit 후 original_means 보존 ---------- */
static void test_refit_preserves_original(void)
{
    printf("\n[4] 재-fit 후 original_means 보존 (드리프트 감지 무력화 방지)\n");
    em_pipeline_t p; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 4242u);
    run_learning(&p, &g);

    float saved[EM_NUM_FEATURES];
    memcpy(saved, p.det.original_means, sizeof(saved));

    /* 신호 특성을 바꾼 뒤 재-fit → mean 은 바뀌어야 하고
     * original_means 는 절대 바뀌면 안 됨 */
    g.amp_1x *= 1.5f;
    run_learning(&p, &g);

    int preserved = (memcmp(saved, p.det.original_means, sizeof(saved)) == 0);
    int mean_moved = (fabsf(p.det.mean[EM_F_H1_ENERGY]
                            - saved[EM_F_H1_ENERGY]) > 1e-6f);
    check(preserved,  "original_means 불변 (has_original_baseline 가드 동작)");
    check(mean_moved, "현재 baseline(mean)은 재-fit 반영");
}

/* ---------- [5] 범용성: 다른 RPM + 회전수 변동 추종 ---------- */
static void test_generic_rpm(void)
{
    printf("\n[5] 범용성 — 정격 1800RPM(30Hz) 설비 + 부하로 회전수 ±8%% 드리프트\n");
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, 1800.0f);          /* 설정 변경은 이 한 줄뿐 */
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 30.0f, 31337u);
    run_learning(&p, &g);

    /* 회전수를 서서히 27.6Hz 까지 내리며 정상 유지 → 오탐 없어야 함 */
    int fp = 0;
    for (int w = 0; w < 100; w++) {
        g.rotation_hz = 30.0f - 2.4f * (float)w / 100.0f;
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.is_anomaly) fp++;
    }
    printf("  회전수 추정 최종: %.2f Hz (실제 %.2f Hz), 점프 거부 %d회\n",
           (double)p.rot.hz, (double)g.rotation_hz, p.rot.rejected);
    check(fabsf(p.rot.hz - g.rotation_hz) < 0.5f, "회전수 추종 오차 < 0.5Hz");
    check(fp == 0, "RPM 드리프트 중 확정 오탐 0건 (상대 대역 배치 효과)");

    /* 같은 조건에서 불평형 주입 → 여전히 감지되는지 */
    inject_unbalance(&g);
    int detected = 0;
    for (int w = 0; w < 60 && !detected; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.is_anomaly) detected = 1;
    }
    check(detected, "RPM 변동 후에도 불평형 감지");
}

/* ---------- [6] JSON 직렬화 스모크 ---------- */
static void test_json(void)
{
    printf("\n[6] JSON 직렬화\n");
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 1u);
    run_learning(&p, &g);
    sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
    em_pipeline_process(&p, sigbuf, &o);
    char buf[512];
    int n = em_output_to_json(&o, buf, sizeof(buf));
    printf("  %s\n", buf);
    check(n > 0 && n < (int)sizeof(buf), "버퍼 내 직렬화 완료");
}

/* ---------- [7] 설비 정지 — 오경보 차단 (실무 치명 결함 수정 검증) ----------
 * 수정 전 동작: 정지 → RMS 폭락 → |z| 폭발 → "이상 확정" 오경보.
 * "설비를 껐다"가 경보로 이어지면 현장 신뢰가 무너진다. */
static void test_machine_stop(void)
{
    printf("\n[7] 설비 정지/재가동 — 정지는 이상이 아니다\n");
    em_pipeline_t p; em_output_t o; sim_t g;
    em_pipeline_init(&p, RATED_RPM);
    p.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 555u);
    run_learning(&p, &g);

    /* 정지 → 30 윈도 관찰: is_anomaly 절대 금지, running=false 전환 */
    sim_power_off(&g);
    int alarms = 0; int became_stopped = 0;
    for (int w = 0; w < 30; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.is_anomaly) alarms++;
        if (!o.running) became_stopped = 1;
    }
    check(became_stopped, "정지 판별 (running=false 전환)");
    check(alarms == 0,    "정지 중 이상 오경보 0건");

    /* 재가동 → 정상 복귀: 오경보 없이 감시 재개 */
    sim_init_normal(&g, 50.0f, 556u);
    int fp = 0; int resumed = 0;
    for (int w = 0; w < 30; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.running) resumed = 1;
        if (o.verdict.is_anomaly) fp++;
    }
    check(resumed,  "재가동 판별 (running=true 복귀)");
    check(fp == 0,  "재가동 직후 정상 신호 오경보 0건 (판정 버퍼 리셋 효과)");

    /* 재가동 후 결함 주입 → 여전히 감지 */
    inject_unbalance(&g);
    int detected = 0;
    for (int w = 0; w < 60 && !detected; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p, sigbuf, &o);
        if (o.verdict.is_anomaly) detected = 1;
    }
    check(detected, "재가동 후 결함 감지 능력 유지");
}

/* ---------- [8] 스냅샷 — 정전/재부팅 후 재학습 없이 감시 재개 ---------- */
static void test_snapshot(void)
{
    printf("\n[8] baseline 영속화 — 재부팅 시나리오\n");
    em_pipeline_t p1; em_output_t o; sim_t g;
    em_pipeline_init(&p1, RATED_RPM);
    p1.cfg.sample_rate_hz = FS;
    sim_init_normal(&g, 50.0f, 888u);
    run_learning(&p1, &g);

    em_snapshot_t snap;
    em_pipeline_save_baseline(&p1, &snap);

    /* "재부팅": 새 파이프라인에 스냅샷 로드 → 즉시 MONITORING */
    em_pipeline_t p2;
    em_pipeline_init(&p2, RATED_RPM);
    p2.cfg.sample_rate_hz = FS;
    check(em_pipeline_load_baseline(&p2, &snap), "스냅샷 로드 성공");
    check(p2.mode == EM_MODE_MONITORING, "로드 즉시 감시 모드 (재학습 생략)");
    check(memcmp(p1.det.original_means, p2.det.original_means,
                 sizeof(p1.det.original_means)) == 0,
          "original_means 까지 복원 (드리프트 기준점 유지)");

    /* 로드된 baseline 으로 정상은 통과, 결함은 감지 */
    int fp = 0;
    for (int w = 0; w < 50; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p2, sigbuf, &o);
        if (o.verdict.is_anomaly) fp++;
    }
    check(fp == 0, "복원 baseline 으로 정상 신호 오경보 0건");
    inject_misalignment(&g);
    int detected = 0;
    for (int w = 0; w < 60 && !detected; w++) {
        sim_fill_window(&g, FS, sigbuf, EM_FFT_SIZE);
        em_pipeline_process(&p2, sigbuf, &o);
        if (o.verdict.is_anomaly) detected = 1;
    }
    check(detected, "복원 baseline 으로 결함 감지");

    /* 손상 스냅샷은 거부 */
    snap.mean[0] += 1.0f;   /* checksum 불일치 유발 */
    em_pipeline_t p3;
    em_pipeline_init(&p3, RATED_RPM);
    check(!em_pipeline_load_baseline(&p3, &snap), "손상(checksum 불일치) 스냅샷 거부");
}

/* ---------- [9] 상대 std 하한 — 조용한 특징의 z 폭발 방지 ---------- */
static void test_rel_std_floor(void)
{
    printf("\n[9] 상대 std 하한 — 무분산 특징 오경보 방지\n");
    em_config_t cfg; em_config_default(&cfg, RATED_RPM);
    em_detector_t d; em_detector_init(&d);
    em_detector_fit_begin(&d);
    /* 한 특징이 학습 내내 완전 상수(분산 0)인 극단 케이스 */
    float f[EM_NUM_FEATURES] = {0.2f, 0.1f, 0.01f, 0.005f, 0.002f};
    for (int i = 0; i < 50; i++) em_detector_fit_add(&d, f);
    em_detector_fit_end(&d, &cfg);

    /* +1% 변동 → 상대 하한(2%) 덕에 z = 0.5 수준이어야 함 */
    float f2[EM_NUM_FEATURES];
    for (int i = 0; i < EM_NUM_FEATURES; i++) f2[i] = f[i] * 1.01f;
    em_verdict_t v;
    em_detector_update(&d, f2, &cfg, &v);
    float maxz = 0;
    for (int i = 0; i < EM_NUM_FEATURES; i++)
        if (fabsf(v.z[i]) > maxz) maxz = fabsf(v.z[i]);
    printf("  무분산 학습 후 +1%% 변동의 최대 |z| = %.2f (하한 없으면 사실상 무한대)\n",
           (double)maxz);
    check(maxz < 1.0f, "+1% 변동이 1σ 미만으로 억제");
    check(!v.instant_flag, "instant_flag 미발화");
}

int main(void)
{
    printf("=== 엣지알리미 C 코어 검증 (rated_rpm=%.0f, fs=%.0fHz, N=%d) ===\n",
           (double)RATED_RPM, (double)FS, EM_FFT_SIZE);
    test_scenario_A();
    test_scenario_B();
    test_drift();
    test_refit_preserves_original();
    test_generic_rpm();
    test_json();
    test_machine_stop();
    test_snapshot();
    test_rel_std_floor();
    printf("\n=== 결과: %s (실패 %d건) ===\n", fails ? "FAIL" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
