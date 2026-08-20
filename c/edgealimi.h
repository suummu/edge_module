/*
  edgealimi.h — 엣지알리미 판별 파이프라인의 순수 C(C99) 구현.

  왜 .ino(C++)와 별도로 순수 C 버전이 있는가:
    - ESP32의 공식 프레임워크(ESP-IDF)는 C 기반이라, Arduino를 벗어나
      ESP-IDF로 갈 때 이 파일들을 컴포넌트로 그대로 사용 가능
    - arduinoFFT(C++ 템플릿) 의존을 제거하고 FFT까지 직접 구현했으므로
      외부 라이브러리가 전혀 없음 — PC에서도 gcc로 컴파일·검증 가능
    - .ino 버전과 동일한 로직·동일한 파라미터 (파이썬 src/ 가 단일 출처)

  파이썬 대응 관계:
    ea_compute_fft_magnitudes()  <->  feature_extraction.compute_fft
    ea_estimate_rotation_hz()    <->  feature_extraction.estimate_rotation_hz
    ea_extract_features()        <->  feature_extraction.extract_features
    ea_detector_fit()/judge()    <->  baseline_detector.BaselineDetector.fit()/judge()

  사용 예 (ESP-IDF의 app_main 또는 PC 테스트에서):
    ea_detector_t det;
    ea_detector_init(&det);
    ea_detector_fit(&det, baseline_features, 60);      // 정상 데이터로 학습
    float feats[EA_N_FEATURES];
    float rot = EA_NO_ESTIMATE;
    ea_extract_features(window, feats, &rot, false);   // 연속 스트림은 rot 체이닝
    ea_judge_result_t r = ea_detector_judge(&det, feats);
    if (r.is_anomaly) { ... }
*/

#ifndef EDGEALIMI_H
#define EDGEALIMI_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 공통 파라미터 (파이썬과 동일) ==================== */
#define EA_WINDOW_SIZE      256      /* FFT 포인트 수 (2의 거듭제곱 필수) */
#define EA_SAMPLE_RATE      1000.0f  /* Hz */
#define EA_FAN_ROTATION_HZ  50.0f    /* 정격 회전 주파수 — 대상 설비에 맞게 수정 */
#define EA_HALF_SIZE        (EA_WINDOW_SIZE / 2)

/* estimate_rotation_hz 파라미터 */
#define EA_ROT_SEARCH_LOW_HZ   30.0f
#define EA_ROT_SEARCH_HIGH_HZ  70.0f
#define EA_ROT_MAX_JUMP_HZ     5.0f

/* 판별 파라미터 (파이썬 BaselineDetector 기본값과 동일) */
#define EA_N_SIGMA              3.0f
#define EA_CONFIRM_WINDOW       5
#define EA_CONFIRM_RATIO        0.6f
#define EA_DRIFT_WARNING_RATIO  1.5f

/* 절대 임계값 — NAN이면 비활성 (파이썬 ABSOLUTE_RMS_LIMIT=None에 대응) */
#define EA_ABSOLUTE_RMS_LIMIT   NAN

/* 회전 추정 "아직 없음" 표현 (파이썬 None에 대응) */
#define EA_NO_ESTIMATE          NAN

/* ==================== 판별에 쓰는 특징 (파이썬 ACTIVE_FEATURES와 동일 순서) ==================== */
typedef enum {
    EA_FEAT_RMS = 0,
    EA_FEAT_H1,      /* harmonic1_energy — 불평형 (1x) */
    EA_FEAT_H2,      /* harmonic2_energy — 정렬불량 (2x) */
    EA_FEAT_H3,      /* harmonic3_energy — 이완 (3x) */
    EA_FEAT_HF,      /* high_freq_energy — 250~400Hz 보조 지표 */
    EA_N_FEATURES
} ea_feature_index_t;

extern const char *EA_FEATURE_NAMES[EA_N_FEATURES];

/* ==================== 판별기 상태 (파이썬 BaselineDetector의 self.* 대응) ==================== */
typedef struct {
    float means[EA_N_FEATURES];
    float stds[EA_N_FEATURES];
    float original_means[EA_N_FEATURES];  /* 위험 정상화 방지용 원본 — 최초 fit에서만 기록 */
    bool  fitted;
    bool  has_original_baseline;          /* 원본 보호 플래그 (재보정 시 덮어쓰기 방지) */

    /* 연속성 필터 — 파이썬 deque(maxlen=N)의 원형 버퍼 구현 */
    bool    suspicion_history[EA_CONFIRM_WINDOW];
    uint8_t suspicion_write_idx;
    uint8_t suspicion_filled_count;       /* N개 미충족 상태 구분 (len(deque) 체크 대응) */
} ea_detector_t;

typedef struct {
    bool  is_suspect;                     /* 이번 윈도우만의 1차 의심 */
    bool  is_anomaly;                     /* 연속성 필터 반영 확정 이상 */
    bool  feat_out[EA_N_FEATURES];        /* 어떤 특징이 3σ 이탈했는지 */
    bool  drift_warn[EA_N_FEATURES];      /* 원본 baseline 대비 드리프트 경고 */
    bool  absolute_limit_hit;
    float suspicion_rate;
} ea_judge_result_t;

/* ==================== 신호 처리 ==================== */

/* 윈도우 평균(DC) 제거 — 실센서 신호의 중력 오프셋 제거용.
   시뮬레이션/테스트 벡터에는 DC가 없으므로 호출자가 실센서일 때만 적용. */
void ea_remove_dc(float *signal, uint16_t n);

/* FFT: 시간영역 신호(EA_WINDOW_SIZE 샘플) -> 정규화(x2/N)된 magnitude EA_HALF_SIZE개.
   radix-2 반복형(비재귀) 구현, 외부 라이브러리 없음. 재진입 불가(내부 정적 버퍼). */
void ea_compute_fft_magnitudes(const float *signal, float *out_magnitudes);

/* bin 인덱스 -> Hz */
static inline float ea_bin_to_hz(uint16_t i) {
    return i * (EA_SAMPLE_RATE / EA_WINDOW_SIZE);
}

/* 회전 주파수 rolling estimate. previous_estimate에 EA_NO_ESTIMATE(NAN)를 넘기면
   파이썬의 None과 동일 — 탐색범위 내 최대 피크를 그대로 채택. */
float ea_estimate_rotation_hz(const float *magnitudes, float previous_estimate);

/* 특정 대역(center±band_width)의 magnitude 합. DC(bin 0) 제외. */
float ea_frequency_band_energy(const float *magnitudes, float center_hz, float band_width_hz);

/* 특징 추출: out_features[EA_N_FEATURES] 채움.
   rotation_estimate_io: in/out — 연속 스트림이면 이전 값을 넘기고 갱신값을 받는다(체이닝).
   independent=true면 이전 값을 무시하고 독립 추정 (baseline 학습 윈도우용). */
void ea_extract_features(const float *signal, float *out_features,
                         float *rotation_estimate_io, bool independent);

/* ==================== 판별기 ==================== */

void ea_detector_init(ea_detector_t *det);

/* baseline 학습. features: n_windows x EA_N_FEATURES 행렬.
   최초 호출에서만 original_means를 기록하고, 재호출(재보정)은 갱신하지 않는다. */
void ea_detector_fit(ea_detector_t *det,
                     const float features[][EA_N_FEATURES], uint16_t n_windows);

ea_judge_result_t ea_detector_judge(ea_detector_t *det,
                                    const float features[EA_N_FEATURES]);

void ea_detector_reset_confirmation(ea_detector_t *det);

#ifdef __cplusplus
}
#endif

#endif /* EDGEALIMI_H */
