/*
  edgealimi_v2_realsensor.ino

  목적:
    파이썬으로 검증 완료한 최신 파이프라인(feature_extraction.py + baseline_detector.py,
    커밋 1ad5427 기준)을 실제 센서 기반으로 ESP32에 이식한 버전.

    v1(edgealimi_v1_simulated.ino)과의 차이:
      - 시뮬레이션 신호 대신 실제 MPU-6050에서 1kHz로 샘플링
      - ACTIVE_FEATURES를 파이썬 최신과 동일하게 5개로 갱신
        (rms, harmonic1/2/3_energy, high_freq_energy — kurtosis 제외됨)
      - 회전 주파수 추정 estimate_rotation_hz() 이식 (하모닉 밴드의 기준축)
      - 연속성 필터(confirm_window=5, confirm_ratio=0.6) 이식 → is_anomaly 확정 판정
      - 위험 정상화 방지(원본 baseline 보관 + 드리프트 경고) 이식
      - baseline을 NVS(Preferences)에 저장 → 재부팅해도 유지
      - 보조 채널: DS18B20(온도), SCT-013(전류 RMS), 홀센서(RPM) — 컴파일 스위치로 선택

  파이썬 대응 관계:
    sampleVibrationWindow()      <->  signal_generator.py 를 실제 센서 읽기로 교체한 것
    computeFFT()                 <->  feature_extraction.compute_fft (정규화 2/N 동일)
    estimateRotationHz()         <->  feature_extraction.estimate_rotation_hz
    extractFeatures()            <->  feature_extraction.extract_features
    fitBaseline()/judge()        <->  baseline_detector.BaselineDetector.fit()/judge()

  필요 라이브러리 (Library Manager):
    - arduinoFFT (v2.x)
    - OneWire, DallasTemperature  (ENABLE_DS18B20 사용 시)

  시리얼 명령 (115200bps):
    b : baseline 학습 (설비가 '정상 운전 중'일 때 실행. 60윈도우 ≈ 약 20초)
        - 최초 학습이면 원본 baseline으로도 기록됨
        - 이후 재실행하면 '재보정' — 현재 baseline은 갱신되지만 원본은 절대 안 바뀜
    s : 현재 baseline/원본 baseline/상태 출력
    x : 공장 초기화 (NVS의 baseline 전부 삭제 — 원본 포함. 새 설비로 옮길 때만)

  C/임베디드 제약으로 파이썬과 다른 점:
    - dict 대신 float 배열 + enum 인덱스 (FEAT_RMS 등)
    - deque(maxlen=N) 대신 원형 버퍼 + filledCount (미충족 상태 구분 필수!)
    - None 대신 NAN 사용 (rotation estimate의 "아직 없음" 표현)
    - 실제 가속도 신호에는 중력 DC 오프셋이 있으므로 윈도우 평균을 빼고 처리
      (파이썬 시뮬레이션 신호는 DC가 없어서 이 단계가 없었음 — 실센서 필수 단계)
*/

#include <Wire.h>
#include <arduinoFFT.h>
#include <Preferences.h>

// ==================== 파이썬 대조 검증 모드 ====================
// C/C++에는 numpy/scipy 같은 내장 모듈이 없어 FFT·통계를 전부 직접 구현했고,
// 파이썬은 float64 / ESP32는 float32 연산이라 값이 미세하게 달라질 수 있다.
// 1로 바꾸면: 센서 대신 파이썬이 내보낸 테스트 신호(test_vectors.h)를 입력으로
// 특징 추출·통계·판별을 돌리고, 파이썬 정답값과 허용 오차 내 일치하는지
// PASS/FAIL을 출력한다 (중간계획서 Ⅳ-6 "같은 입력에 같은 출력" 대조 검증).
// 정답 파일 갱신: repo 루트에서 `python -m tools.export_test_vectors`
// 검증 통과 후 0으로 되돌려 실센서 모드로 사용.
#define VALIDATION_MODE 1

#if VALIDATION_MODE
#include "test_vectors.h"
#endif

// ==================== 보조 채널 컴파일 스위치 (부품 배선 전이면 0으로) ====================
#define ENABLE_DS18B20  0   // 온도 (1-Wire, OneWire + DallasTemperature 라이브러리 필요)
#define ENABLE_SCT013   0   // 전류 (ADC, 버든저항 + 바이어스 회로 필요)
#define ENABLE_HALL     0   // RPM (홀센서 + 자석, GPIO 인터럽트)

#if ENABLE_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

// ==================== 핀 배치 ====================
const uint8_t PIN_I2C_SDA   = 21;   // MPU-6050 SDA
const uint8_t PIN_I2C_SCL   = 22;   // MPU-6050 SCL
const uint8_t PIN_DS18B20   = 4;    // 온도 센서 데이터 (4.7kΩ 풀업 필요)
const uint8_t PIN_SCT013    = 34;   // 전류 센서 ADC 입력 (입력 전용 핀)
const uint8_t PIN_HALL      = 27;   // 홀센서 펄스 입력
const uint8_t PIN_LED_ALERT = 2;    // 확정 이상 경보 LED (DevKit 내장 LED)

// ==================== 공통 파라미터 (파이썬 SAMPLE_RATE, WINDOW_SIZE와 동일) ====================
const uint16_t WINDOW_SIZE     = 256;      // FFT 포인트 수
const float    SAMPLE_RATE     = 1000.0f;  // Hz
const float    FAN_ROTATION_HZ = 50.0f;    // 정격 회전 주파수 (3000RPM). 대상 설비에 맞게 수정

// estimate_rotation_hz 파라미터 (파이썬 기본값과 동일)
const float ROT_SEARCH_LOW_HZ  = 30.0f;    // 정격 RPM 기준 탐색 하한
const float ROT_SEARCH_HIGH_HZ = 70.0f;    // 탐색 상한
const float ROT_MAX_JUMP_HZ    = 5.0f;     // 이보다 크게 튀면 노이즈로 보고 이전 값 유지

// 판별 파라미터 (파이썬 BaselineDetector 기본값과 동일)
const float    N_SIGMA           = 3.0f;
const uint8_t  CONFIRM_WINDOW    = 5;
const float    CONFIRM_RATIO     = 0.6f;
const float    DRIFT_WARNING_RATIO = 1.5f;
const uint16_t BASELINE_N_WINDOWS  = 60;

// 절대 임계값 (파이썬 ABSOLUTE_RMS_LIMIT=None에 대응. NAN이면 절대판정 비활성.
// TODO: 실제 설비 데이터시트 확보 후 g 단위 고정값으로 채울 자리)
const float ABSOLUTE_RMS_LIMIT = NAN;

// ==================== MPU-6050 레지스터 ====================
const uint8_t MPU_ADDR        = 0x68;
const uint8_t REG_PWR_MGMT_1  = 0x6B;
const uint8_t REG_CONFIG      = 0x1A;   // DLPF 설정
const uint8_t REG_ACCEL_CFG   = 0x1C;
const uint8_t REG_ACCEL_XOUT  = 0x3B;
// ±4g 레인지 기준 스케일 (데이터시트: 8192 LSB/g)
const float ACCEL_LSB_PER_G = 8192.0f;
// 진동 축 선택: 0=X, 1=Y, 2=Z. 설비 부착 방향(방사 방향)에 맞게 조정.
const uint8_t VIBRATION_AXIS = 2;

// ==================== 판별에 쓰는 특징 (파이썬 ACTIVE_FEATURES와 동일 순서) ====================
// kurtosis는 파이썬 최신에서 판별 지표에서 제외됨(베어링류 지표 + 임펄스성 아님) → 계산 자체를 생략
enum FeatureIndex {
  FEAT_RMS = 0,
  FEAT_H1,        // harmonic1_energy (1x 대역) — 불평형
  FEAT_H2,        // harmonic2_energy (2x 대역) — 정렬불량
  FEAT_H3,        // harmonic3_energy (3x 대역) — 이완
  FEAT_HF,        // high_freq_energy (250~400Hz) — 보조 지표 (베어링 정밀진단 아님)
  N_FEATURES
};
const char *FEATURE_NAMES[N_FEATURES] = {
  "rms", "harmonic1_energy", "harmonic2_energy", "harmonic3_energy", "high_freq_energy"
};

// ==================== FFT용 배열 (전역 고정 할당) ====================
float vReal[WINDOW_SIZE];
float vImag[WINDOW_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, WINDOW_SIZE, SAMPLE_RATE);

// ==================== 판별기 상태 (파이썬 BaselineDetector의 self.* 에 대응) ====================
float baselineMeans[N_FEATURES];      // self.means
float baselineStds[N_FEATURES];       // self.stds
float originalMeans[N_FEATURES];      // self.original_means — 위험 정상화 방지용 원본
bool  isFitted = false;               // self.fitted
bool  hasOriginalBaseline = false;    // *** 체크리스트 #2: 원본은 최초 1회만 기록하는 보호 플래그 ***

// 연속성 필터 이력 — 파이썬 deque(maxlen=confirm_window)의 원형 버퍼 구현
bool    suspicionHistory[CONFIRM_WINDOW];
uint8_t suspicionWriteIdx = 0;
uint8_t suspicionFilledCount = 0;     // *** 체크리스트 #1: N개가 안 채워진 상태를 구분 ***
                                      // (파이썬의 len(deque)==confirm_window 체크에 대응.
                                      //  이게 없으면 부팅 직후 쓰레기값까지 세어 판정이 왜곡됨)

// 회전 주파수 rolling estimate — 파이썬의 previous_estimate 체이닝. NAN = None
float rotationHzEstimate = NAN;

Preferences prefs;   // NVS 저장소 (baseline 영속화)

// ==================== 판별 결과 구조체 (파이썬 judge() 반환 dict에 대응) ====================
struct JudgeResult {
  bool  isSuspect;                    // 이번 윈도우만의 1차 의심
  bool  isAnomaly;                    // 연속성 필터 반영 확정 이상
  bool  featOut[N_FEATURES];          // 어떤 특징이 3σ 이탈했는지 (reasons에 대응)
  bool  absoluteLimitHit;             // 절대판정 (RMS 안전 한계 초과)
  bool  driftWarn[N_FEATURES];        // 원본 baseline 대비 드리프트 경고
  float suspicionRate;
};

// ==================== 함수 프로토타입 ====================
// Arduino IDE의 자동 프로토타입 생성은 구조체를 반환하는 함수에서 실패하는 경우가
// 있어 명시적으로 선언한다 (v1과 동일한 관례). 정의 순서와 무관하게 호출 가능해짐.
JudgeResult judge(const float *features);
void resetConfirmationState();
void updateConfirmation(bool isSuspect, bool *outIsAnomaly, float *outRate);
void saveBaselineToNVS();
void computeFFT(const float *inputSignal);
float estimateRotationHz(float previousEstimate);
void extractFeatures(const float *signal, float *outFeatures, bool independentEstimate);

// ==================== MPU-6050 로우레벨 ====================
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);              // 400kHz — 1kHz 샘플링 여유 확보 (읽기 1회 ≈ 0.2ms)
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;   // 센서 응답 없음

  mpuWriteReg(REG_PWR_MGMT_1, 0x01);  // 슬립 해제, 클럭 = 자이로 X PLL
  mpuWriteReg(REG_CONFIG, 0x00);      // DLPF=0 → 가속도 대역폭 260Hz (본 모듈의 유효 대역)
  mpuWriteReg(REG_ACCEL_CFG, 0x08);   // ±4g (설비 진동이 ±2g를 넘을 수 있어 여유 확보)
  delay(100);
  return true;
}

// 한 샘플 읽기: 선택 축 가속도를 g 단위로 반환
float mpuReadAccelG() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  int16_t raw[3];
  for (uint8_t i = 0; i < 3; i++) {
    raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return raw[VIBRATION_AXIS] / ACCEL_LSB_PER_G;
}

// ==================== 진동 윈도우 샘플링 (signal_generator를 실센서로 교체한 부분) ====================
// micros() 기반 등간격 1kHz 샘플링. 256샘플 = 약 256ms 소요.
void sampleVibrationWindow(float *outSignal) {
  const uint32_t intervalUs = (uint32_t)(1000000.0f / SAMPLE_RATE);
  uint32_t nextTick = micros();
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
    while ((int32_t)(micros() - nextTick) < 0) { /* 다음 샘플 시각까지 대기 */ }
    outSignal[i] = mpuReadAccelG();
    nextTick += intervalUs;
  }

  // 중력 DC 오프셋 제거 (실센서 필수 — 시뮬레이션 신호에는 없던 단계).
  // 부착 방향에 따라 선택 축에 최대 ±1g의 상수 성분이 실리는데, 이걸 안 빼면
  // RMS가 진동이 아니라 중력을 재게 되고 FFT의 DC bin도 오염된다.
  float mean = 0.0f;
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) mean += outSignal[i];
  mean /= WINDOW_SIZE;
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) outSignal[i] -= mean;
}

// ==================== FFT (파이썬 compute_fft에 대응) ====================
// 결과: vReal[0 .. WINDOW_SIZE/2-1]에 파이썬과 동일한 정규화(×2/N)의 magnitude가 채워짐.
// 파이썬은 창함수를 쓰지 않으므로 여기서도 쓰지 않는다 (v1의 Hamming 제거 —
// 파이썬 검증 수치와 교차 대조가 가능해야 이식 검증이 성립하기 때문).
void computeFFT(const float *inputSignal) {
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
    vReal[i] = inputSignal[i];
    vImag[i] = 0.0f;
  }
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  const float norm = 2.0f / WINDOW_SIZE;
  for (uint16_t i = 0; i < WINDOW_SIZE / 2; i++) vReal[i] *= norm;
}

// bin 인덱스 → 주파수(Hz). 해상도 = 1000/256 ≈ 3.9Hz
inline float binToHz(uint16_t i) { return i * (SAMPLE_RATE / WINDOW_SIZE); }

// ==================== 대역 에너지 (파이썬 frequency_band_energy에 대응) ====================
float frequencyBandEnergy(float centerHz, float bandWidthHz) {
  float sum = 0.0f;
  for (uint16_t i = 1; i < WINDOW_SIZE / 2; i++) {   // i=0(DC) 제외
    float freq = binToHz(i);
    if (freq >= centerHz - bandWidthHz && freq <= centerHz + bandWidthHz) sum += vReal[i];
  }
  return sum;
}

// ==================== 회전 주파수 추정 (파이썬 estimate_rotation_hz에 대응) ====================
// previousEstimate에 NAN을 넘기면 파이썬의 None과 동일하게 동작:
// 탐색범위 내 최대 피크를 그대로 채택. 그 외에는 max_jump 필터로 완만하게 갱신.
float estimateRotationHz(float previousEstimate) {
  float bestMag = -1.0f;
  float candidate = NAN;
  for (uint16_t i = 1; i < WINDOW_SIZE / 2; i++) {
    float freq = binToHz(i);
    if (freq < ROT_SEARCH_LOW_HZ || freq > ROT_SEARCH_HIGH_HZ) continue;
    if (vReal[i] > bestMag) { bestMag = vReal[i]; candidate = freq; }
  }
  if (isnan(candidate)) return previousEstimate;             // 탐색범위 내 피크 없음
  if (isnan(previousEstimate)) return candidate;             // 첫 추정
  if (fabsf(candidate - previousEstimate) > ROT_MAX_JUMP_HZ) return previousEstimate;
  return candidate;
}

// ==================== 특징 추출 (파이썬 extract_features에 대응) ====================
// 반환: outFeatures[N_FEATURES] 채움 + 갱신된 rotationHzEstimate (전역 체이닝)
// independentEstimate=true면 파이썬의 rotation_hz_estimate=None 호출에 대응
// (baseline 학습용 독립 윈도우). false면 연속 스트림 체이닝.
void extractFeatures(const float *signal, float *outFeatures, bool independentEstimate) {
  // RMS
  float sumSquares = 0.0f;
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) sumSquares += signal[i] * signal[i];
  outFeatures[FEAT_RMS] = sqrtf(sumSquares / WINDOW_SIZE);

  computeFFT(signal);

  float prev = independentEstimate ? NAN : rotationHzEstimate;
  float est = estimateRotationHz(prev);
  // 파이썬과 동일한 폴백: 추정 실패(None) 시 정격 회전주파수 사용
  if (isnan(est)) est = FAN_ROTATION_HZ;
  if (!independentEstimate) rotationHzEstimate = est;

  outFeatures[FEAT_H1] = frequencyBandEnergy(est * 1.0f, 5.0f);
  outFeatures[FEAT_H2] = frequencyBandEnergy(est * 2.0f, 5.0f);
  outFeatures[FEAT_H3] = frequencyBandEnergy(est * 3.0f, 5.0f);
  outFeatures[FEAT_HF] = frequencyBandEnergy(325.0f, 75.0f);   // 250~400Hz
}

// ==================== Baseline 학습 (파이썬 fit()에 대응) ====================
// 설비가 '정상 운전 중'일 때 시리얼 'b' 명령으로 실행.
// 60윈도우 × 256ms ≈ 15초 + 오버헤드. 학습 중에는 판별 중단.
void fitBaseline() {
  // 60×5 float = 1.2KB — 스택 대신 static으로 (ESP32 태스크 스택 보호)
  static float featureLog[BASELINE_N_WINDOWS][N_FEATURES];
  static float windowSignal[WINDOW_SIZE];

  Serial.println(F("=== Baseline 학습 시작 — 설비가 정상 운전 중인지 확인하세요 ==="));
  Serial.println(F("(작업자 접촉·적재 등 현실 변수를 일부러 포함시키면 기준선이 더 견고해짐)"));

  for (uint16_t w = 0; w < BASELINE_N_WINDOWS; w++) {
    sampleVibrationWindow(windowSignal);
    // 파이썬 build_normal_baseline과 동일: 학습 윈도우는 서로 독립 추정 (체이닝 없음)
    extractFeatures(windowSignal, featureLog[w], true);
    if ((w + 1) % 10 == 0) {
      Serial.print(F("  진행: "));
      Serial.print(w + 1);
      Serial.print(F("/"));
      Serial.println(BASELINE_N_WINDOWS);
    }
  }

  for (uint8_t k = 0; k < N_FEATURES; k++) {
    float sum = 0.0f;
    for (uint16_t w = 0; w < BASELINE_N_WINDOWS; w++) sum += featureLog[w][k];
    float mean = sum / BASELINE_N_WINDOWS;

    float sumSqDiff = 0.0f;
    for (uint16_t w = 0; w < BASELINE_N_WINDOWS; w++) {
      float d = featureLog[w][k] - mean;
      sumSqDiff += d * d;
    }
    baselineMeans[k] = mean;
    baselineStds[k] = sqrtf(sumSqDiff / BASELINE_N_WINDOWS);   // 파이썬 np.std와 동일 (모표준편차, n으로 나눔)
  }

  // *** 원본 baseline은 최초 1회만 기록 (파이썬 `if not self.fitted:` 에 대응) ***
  // 재보정(fit 재호출) 때 이 블록을 건너뛰지 않으면 위험 정상화 방지 기능이
  // 통째로 무력화된다 — 서서히 나빠진 상태를 항상 "최신 원본"과 비교하게 되므로.
  if (!hasOriginalBaseline) {
    for (uint8_t k = 0; k < N_FEATURES; k++) originalMeans[k] = baselineMeans[k];
    hasOriginalBaseline = true;
    Serial.println(F("[최초 학습] 원본 baseline으로 기록됨 (이후 재보정에도 보존)"));
  } else {
    Serial.println(F("[재보정] 현재 baseline 갱신 — 원본 baseline은 유지됨"));
  }

  isFitted = true;
  resetConfirmationState();    // 새 baseline이므로 연속성 필터 이력도 초기화
  saveBaselineToNVS();

  Serial.println(F("Baseline 학습 완료:"));
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    Serial.print(F("  "));
    Serial.print(FEATURE_NAMES[k]);
    Serial.print(F(": 평균="));
    Serial.print(baselineMeans[k], 5);
    Serial.print(F(" 표준편차="));
    Serial.println(baselineStds[k], 5);
  }
}

// ==================== 연속성 필터 (파이썬 deque + judge()의 3번 블록에 대응) ====================
void resetConfirmationState() {
  suspicionWriteIdx = 0;
  suspicionFilledCount = 0;
}

// 이력에 이번 판정을 추가하고, 확정 이상 여부와 의심 비율을 계산
void updateConfirmation(bool isSuspect, bool *outIsAnomaly, float *outRate) {
  suspicionHistory[suspicionWriteIdx] = isSuspect;
  suspicionWriteIdx = (suspicionWriteIdx + 1) % CONFIRM_WINDOW;
  if (suspicionFilledCount < CONFIRM_WINDOW) suspicionFilledCount++;

  uint8_t suspectCount = 0;
  for (uint8_t i = 0; i < suspicionFilledCount; i++) {
    if (suspicionHistory[i]) suspectCount++;
  }
  float rate = (float)suspectCount / suspicionFilledCount;

  // 파이썬: len(deque)==confirm_window AND rate>=confirm_ratio
  // filledCount 조건이 빠지면 부팅 직후 1~2개 의심만으로 확정이 떠버림 (체크리스트 #1)
  *outIsAnomaly = (suspicionFilledCount == CONFIRM_WINDOW) && (rate >= CONFIRM_RATIO);
  *outRate = rate;
}

// ==================== 판별 (파이썬 judge()에 대응) ====================
JudgeResult judge(const float *features) {
  JudgeResult r = {};

  // 1) 상대적 판정 (3σ 규칙)
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    float lower = baselineMeans[k] - N_SIGMA * baselineStds[k];
    float upper = baselineMeans[k] + N_SIGMA * baselineStds[k];
    r.featOut[k] = (features[k] < lower || features[k] > upper);
    if (r.featOut[k]) r.isSuspect = true;
  }

  // 2) 절대적 판정 (데이터시트 고정값 — 현재 비활성, TODO)
  if (!isnan(ABSOLUTE_RMS_LIMIT) && features[FEAT_RMS] > ABSOLUTE_RMS_LIMIT) {
    r.absoluteLimitHit = true;
    r.isSuspect = true;
  }

  // 3) 연속성 필터 → 확정 이상
  updateConfirmation(r.isSuspect, &r.isAnomaly, &r.suspicionRate);

  // 4) 위험 정상화 감시 (원본 baseline 대비 드리프트)
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    r.driftWarn[k] = false;
    float orig = originalMeans[k];
    if (fabsf(orig) < 1e-9f) continue;                 // 파이썬과 동일: 0 근처 원본은 스킵
    if (features[k] / orig >= DRIFT_WARNING_RATIO) r.driftWarn[k] = true;
  }

  return r;
}

// ==================== NVS 영속화 (파이썬에는 없는 실배포용 추가 기능) ====================
// 원본 baseline이 재부팅으로 사라지면 위험 정상화 방지가 리셋되므로 반드시 저장.
void saveBaselineToNVS() {
  prefs.begin("edgealimi", false);
  prefs.putBytes("means", baselineMeans, sizeof(baselineMeans));
  prefs.putBytes("stds", baselineStds, sizeof(baselineStds));
  prefs.putBytes("origMeans", originalMeans, sizeof(originalMeans));
  prefs.putBool("fitted", isFitted);
  prefs.putBool("hasOrig", hasOriginalBaseline);
  prefs.end();
  Serial.println(F("[NVS] baseline 저장 완료 (재부팅 후에도 유지)"));
}

void loadBaselineFromNVS() {
  prefs.begin("edgealimi", true);
  if (prefs.getBool("fitted", false) &&
      prefs.getBytesLength("means") == sizeof(baselineMeans)) {
    prefs.getBytes("means", baselineMeans, sizeof(baselineMeans));
    prefs.getBytes("stds", baselineStds, sizeof(baselineStds));
    prefs.getBytes("origMeans", originalMeans, sizeof(originalMeans));
    isFitted = true;
    hasOriginalBaseline = prefs.getBool("hasOrig", false);
    Serial.println(F("[NVS] 저장된 baseline 로드됨 — 즉시 감시 시작 가능"));
  } else {
    Serial.println(F("[NVS] 저장된 baseline 없음 — 'b' 명령으로 학습 필요"));
  }
  prefs.end();
}

void factoryResetNVS() {
  prefs.begin("edgealimi", false);
  prefs.clear();
  prefs.end();
  isFitted = false;
  hasOriginalBaseline = false;
  resetConfirmationState();
  rotationHzEstimate = NAN;
  Serial.println(F("[초기화] baseline 전부 삭제됨 (원본 포함). 새 설비 기준으로 재학습하세요."));
}

// ==================== 보조 채널: 온도 (DS18B20) ====================
#if ENABLE_DS18B20
OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);

float readTemperatureC() {
  tempSensor.requestTemperatures();
  return tempSensor.getTempCByIndex(0);
}
#endif

// ==================== 보조 채널: 전류 RMS (SCT-013) ====================
#if ENABLE_SCT013
// 회로 전제: SCT-013 출력 → 버든저항 → 1.65V 바이어스(분압) → PIN_SCT013
// SCT_CAL: 클램프 모델·버든저항 값에 따른 A/V 환산 계수 — 실측 보정 필요 (TODO)
const float SCT_CAL = 30.0f;

float readCurrentRmsA() {
  const uint16_t N_SAMPLES = 400;                    // 60Hz 기준 여러 사이클 커버 (~20ms×수회)
  float sumSq = 0.0f, mean = 0.0f;
  static uint16_t buf[N_SAMPLES];
  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    buf[i] = analogRead(PIN_SCT013);
    delayMicroseconds(50);
  }
  for (uint16_t i = 0; i < N_SAMPLES; i++) mean += buf[i];
  mean /= N_SAMPLES;
  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    float d = buf[i] - mean;                         // DC 바이어스 제거
    sumSq += d * d;
  }
  float rmsCounts = sqrtf(sumSq / N_SAMPLES);
  return rmsCounts * (3.3f / 4095.0f) * SCT_CAL;
}
#endif

// ==================== 보조 채널: RPM (홀센서) ====================
#if ENABLE_HALL
volatile uint32_t hallPulseCount = 0;
uint32_t hallLastReadMs = 0;
const uint8_t MAGNETS_PER_REV = 1;                   // 회전체에 부착한 자석 개수

void IRAM_ATTR onHallPulse() { hallPulseCount++; }

// 마지막 호출 이후 평균 회전 주파수(Hz). FFT 추정(rotationHzEstimate)과 상호 검증용.
float readHallRotationHz() {
  uint32_t now = millis();
  uint32_t elapsed = now - hallLastReadMs;
  if (elapsed < 200) return NAN;                     // 너무 짧은 구간은 신뢰 불가
  noInterrupts();
  uint32_t pulses = hallPulseCount;
  hallPulseCount = 0;
  interrupts();
  hallLastReadMs = now;
  return (pulses / (float)MAGNETS_PER_REV) / (elapsed / 1000.0f);
}
#endif

// ==================== 결과 출력 ====================
void printJudgeResult(const float *features, const JudgeResult &r) {
  Serial.print(F("판정: 1차의심="));
  Serial.print(r.isSuspect ? F("O") : F("-"));
  Serial.print(F(" 확정이상="));
  Serial.print(r.isAnomaly ? F("O") : F("-"));
  Serial.print(F(" (의심비율="));
  Serial.print(r.suspicionRate, 2);
  Serial.print(F(", 회전추정="));
  Serial.print(rotationHzEstimate, 1);
  Serial.println(F("Hz)"));

  // '진단 없는 경보' 방지: 어떤 지표가 얼마나 벗어났는지 함께 출력
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    if (r.featOut[k]) {
      Serial.print(F("  - "));
      Serial.print(FEATURE_NAMES[k]);
      Serial.print(F(" 정상범위 이탈: 관측="));
      Serial.print(features[k], 5);
      Serial.print(F(" / 정상범위 "));
      Serial.print(baselineMeans[k] - N_SIGMA * baselineStds[k], 5);
      Serial.print(F("~"));
      Serial.println(baselineMeans[k] + N_SIGMA * baselineStds[k], 5);
    }
  }
  if (r.absoluteLimitHit) Serial.println(F("  - [절대판정] RMS 안전 한계 초과!"));
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    if (r.driftWarn[k]) {
      Serial.print(F("  [드리프트 경고] "));
      Serial.print(FEATURE_NAMES[k]);
      Serial.print(F(" 원본 대비 "));
      Serial.print(features[k] / originalMeans[k], 2);
      Serial.println(F("배 — 서서히 나빠지는 중일 수 있음 (위험 정상화 주의)"));
    }
  }
}

// ==================== 시리얼 명령 처리 ====================
void handleSerialCommand() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  while (Serial.available()) Serial.read();          // 잔여 입력(개행 등) 비움

  switch (cmd) {
    case 'b':
      fitBaseline();
      break;
    case 's':
      Serial.print(F("상태: fitted="));
      Serial.print(isFitted ? F("Y") : F("N"));
      Serial.print(F(" 원본보유="));
      Serial.println(hasOriginalBaseline ? F("Y") : F("N"));
      if (isFitted) {
        for (uint8_t k = 0; k < N_FEATURES; k++) {
          Serial.print(F("  "));
          Serial.print(FEATURE_NAMES[k]);
          Serial.print(F(": 현재평균="));
          Serial.print(baselineMeans[k], 5);
          Serial.print(F(" 원본평균="));
          Serial.println(originalMeans[k], 5);
        }
      }
      break;
    case 'x':
      factoryResetNVS();
      break;
    default:
      break;                                          // 개행 등 무시
  }
}

// ==================== 파이썬 대조 검증 (VALIDATION_MODE=1일 때만 컴파일) ====================
#if VALIDATION_MODE

// 상대 오차 비교. float32 vs float64 정밀도 차이 + arduinoFFT vs scipy 구현 차이를
// 감안한 허용 오차. 이보다 크게 벌어지면 이식 버그로 봐야 한다.
const float REL_TOLERANCE = 0.02f;   // 2%
const float ABS_FLOOR     = 1e-4f;   // 0 근처 값의 상대오차 폭발 방지

bool nearlyEqual(float cVal, float pyVal) {
  float diff = fabsf(cVal - pyVal);
  float scale = fabsf(pyVal);
  if (scale < ABS_FLOOR) return diff < ABS_FLOOR;
  return (diff / scale) < REL_TOLERANCE;
}

void runValidation() {
  uint16_t passCount = 0, failCount = 0;
  float features[N_FEATURES];

  Serial.println(F("\n===== 파이썬 대조 검증 시작 ====="));
  Serial.println(F("(입력: 파이썬이 내보낸 신호 / 정답: 파이썬 float64 계산값 / 허용 상대오차 2%)"));

  // --- 검증 1: 특징 추출 (FFT + 대역 에너지 + RMS + 회전 추정) ---
  Serial.println(F("\n[검증 1] 특징 추출 — numpy/scipy 없이 직접 구현한 부분"));
  for (uint16_t w = 0; w < TV_N_WINDOWS; w++) {
    // 파이썬과 동일하게 독립 추정 (rotation_hz_estimate=None 호출에 대응)
    extractFeatures(TV_SIGNALS[w], features, true);

    Serial.print(F("  ["));
    Serial.print(TV_LABELS[w]);
    Serial.println(F("]"));
    for (uint8_t k = 0; k < N_FEATURES; k++) {
      bool ok = nearlyEqual(features[k], TV_EXPECTED_FEATURES[w][k]);
      ok ? passCount++ : failCount++;
      Serial.print(ok ? F("    PASS ") : F("    FAIL "));
      Serial.print(FEATURE_NAMES[k]);
      Serial.print(F(": C="));
      Serial.print(features[k], 6);
      Serial.print(F(" / Python="));
      Serial.println(TV_EXPECTED_FEATURES[w][k], 6);
    }
    // 회전 추정은 같은 bin에 떨어져야 하므로 사실상 정확히 일치해야 함
    computeFFT(TV_SIGNALS[w]);
    float rotEst = estimateRotationHz(NAN);
    if (isnan(rotEst)) rotEst = FAN_ROTATION_HZ;
    bool rotOk = fabsf(rotEst - TV_EXPECTED_ROTATION_HZ[w]) < 0.01f;
    rotOk ? passCount++ : failCount++;
    Serial.print(rotOk ? F("    PASS ") : F("    FAIL "));
    Serial.print(F("rotation_hz: C="));
    Serial.print(rotEst, 4);
    Serial.print(F(" / Python="));
    Serial.println(TV_EXPECTED_ROTATION_HZ[w], 4);
  }

  // --- 검증 2: baseline 통계 (평균/표준편차 — np.mean/np.std 직접 구현 부분) ---
  Serial.println(F("\n[검증 2] baseline 평균/표준편차 계산"));
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    float sum = 0.0f;
    for (uint16_t w = 0; w < TV_N_BASELINE; w++) sum += TV_BASELINE_FEATURES[w][k];
    float mean = sum / TV_N_BASELINE;
    float sumSqDiff = 0.0f;
    for (uint16_t w = 0; w < TV_N_BASELINE; w++) {
      float d = TV_BASELINE_FEATURES[w][k] - mean;
      sumSqDiff += d * d;
    }
    float stdDev = sqrtf(sumSqDiff / TV_N_BASELINE);

    bool meanOk = nearlyEqual(mean, TV_EXPECTED_MEANS[k]);
    bool stdOk = nearlyEqual(stdDev, TV_EXPECTED_STDS[k]);
    meanOk ? passCount++ : failCount++;
    stdOk ? passCount++ : failCount++;
    Serial.print((meanOk && stdOk) ? F("  PASS ") : F("  FAIL "));
    Serial.print(FEATURE_NAMES[k]);
    Serial.print(F(": 평균 C="));
    Serial.print(mean, 6);
    Serial.print(F("/Py="));
    Serial.print(TV_EXPECTED_MEANS[k], 6);
    Serial.print(F("  표준편차 C="));
    Serial.print(stdDev, 6);
    Serial.print(F("/Py="));
    Serial.println(TV_EXPECTED_STDS[k], 6);
  }

  // --- 검증 3: 판별 로직 (3σ judge — 불리언은 정확히 일치해야 함) ---
  // 파이썬이 계산한 특징값을 그대로 넣어 판별만 격리 검증한다.
  // (C가 자체 추출한 특징으로 판별하면 경계 근처에서 정밀도 차이로 갈릴 수 있음)
  Serial.println(F("\n[검증 3] 3-sigma 판별 로직 (is_suspect)"));
  for (uint8_t k = 0; k < N_FEATURES; k++) {
    baselineMeans[k] = TV_EXPECTED_MEANS[k];
    baselineStds[k] = TV_EXPECTED_STDS[k];
  }
  isFitted = true;
  for (uint16_t w = 0; w < TV_N_WINDOWS; w++) {
    resetConfirmationState();
    JudgeResult r = judge(TV_EXPECTED_FEATURES[w]);
    bool ok = (r.isSuspect == TV_EXPECTED_SUSPECT[w]);
    ok ? passCount++ : failCount++;
    Serial.print(ok ? F("  PASS ") : F("  FAIL "));
    Serial.print(TV_LABELS[w]);
    Serial.print(F(": C="));
    Serial.print(r.isSuspect ? F("의심") : F("정상"));
    Serial.print(F(" / Python="));
    Serial.println(TV_EXPECTED_SUSPECT[w] ? F("의심") : F("정상"));
  }

  Serial.println(F("\n===== 검증 결과 ====="));
  Serial.print(F("PASS: "));
  Serial.print(passCount);
  Serial.print(F(" / FAIL: "));
  Serial.println(failCount);
  if (failCount == 0) {
    Serial.println(F("전체 통과 — 파이썬과 동일 동작 확인. VALIDATION_MODE를 0으로 되돌려 실센서 모드로 사용하세요."));
  } else {
    Serial.println(F("실패 항목 있음 — 이식 코드의 해당 부분을 파이썬과 다시 대조할 것."));
    Serial.println(F("(자주 나오는 원인: FFT 정규화(2/N) 누락, 대역 경계 부등호, DC bin 포함 여부, n vs n-1 분산)"));
  }
}
#endif  // VALIDATION_MODE

// ==================== 메인 ====================
float currentWindow[WINDOW_SIZE];
float currentFeatures[N_FEATURES];

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWrite(PIN_LED_ALERT, LOW);

#if VALIDATION_MODE
  Serial.println(F("=== 엣지알리미 ESP32 v2 — 파이썬 대조 검증 모드 ==="));
  Serial.println(F("(센서 불필요. 보드만 있으면 실행됨)"));
  runValidation();
  return;   // 검증 모드에서는 센서 초기화·감시 루프를 돌지 않음
#endif

  Serial.println(F("=== 엣지알리미 ESP32 v2 (실센서) ==="));
  Serial.println(F("명령: b=baseline 학습, s=상태, x=초기화"));

  if (!mpuInit()) {
    Serial.println(F("[오류] MPU-6050 응답 없음 — 배선(SDA=21, SCL=22, VCC=3.3V) 확인 후 리셋"));
    while (true) { delay(1000); }
  }
  Serial.println(F("MPU-6050 초기화 완료 (DLPF 260Hz, ±4g)"));

#if ENABLE_DS18B20
  tempSensor.begin();
#endif
#if ENABLE_SCT013
  analogReadResolution(12);
#endif
#if ENABLE_HALL
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallPulse, FALLING);
  hallLastReadMs = millis();
#endif

  loadBaselineFromNVS();
}

void loop() {
#if VALIDATION_MODE
  delay(1000);   // 검증은 setup()에서 1회 완료. 재실행하려면 보드 리셋.
  return;
#endif

  handleSerialCommand();

  if (!isFitted) {
    delay(100);
    return;                                           // baseline 없으면 감시 불가
  }

  // 1윈도우 수집(≈256ms) → 특징 추출 → 판별. 연속 스트림이므로 회전 추정 체이닝.
  sampleVibrationWindow(currentWindow);
  extractFeatures(currentWindow, currentFeatures, false);
  JudgeResult result = judge(currentFeatures);

  printJudgeResult(currentFeatures, result);
  digitalWrite(PIN_LED_ALERT, result.isAnomaly ? HIGH : LOW);

#if ENABLE_HALL
  // RPM 이중화: 홀센서 실측 vs FFT 추정 상호 검증 (중간계획서 Ⅱ-2 ②)
  float hallHz = readHallRotationHz();
  if (!isnan(hallHz) && !isnan(rotationHzEstimate)) {
    Serial.print(F("  RPM 교차검증: 홀센서="));
    Serial.print(hallHz, 1);
    Serial.print(F("Hz / FFT추정="));
    Serial.print(rotationHzEstimate, 1);
    Serial.print(F("Hz"));
    if (fabsf(hallHz - rotationHzEstimate) > rotationHzEstimate * 0.2f) {
      Serial.print(F("  <- 20% 이상 불일치, 추정 신뢰도 점검 필요"));
    }
    Serial.println();
  }
#endif
#if ENABLE_DS18B20
  Serial.print(F("  온도: "));
  Serial.print(readTemperatureC(), 1);
  Serial.println(F("C"));
#endif
#if ENABLE_SCT013
  Serial.print(F("  전류 RMS: "));
  Serial.print(readCurrentRmsA(), 2);
  Serial.println(F("A"));
#endif
}
