/*
 * em_config.h — 엣지알리미 공통 설정
 *
 * [범용성 원칙]
 *  - 테스트 데이터가 없어도 어떤 회전 설비에든 붙일 수 있도록,
 *    설비 의존 값은 전부 이 구조체 하나로 모은다.
 *  - 필수 입력은 명판 정격 RPM(rated_rpm) 하나. 나머지는 전부
 *    rated_rpm 또는 rotation_hz_estimate 기준의 "상대값"으로 파생된다.
 *  - 고정 Hz 하드코딩 금지 (기존 파이썬의 search_range_hz=(30,70)
 *    하드코딩 문제를 여기서 구조적으로 해결).
 */
#ifndef EM_CONFIG_H
#define EM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 컴파일 타임 고정 상수 (메모리 정적 할당용) ---- */
#define EM_FFT_SIZE          1024     /* 2의 거듭제곱 필수 */
#define EM_NUM_BINS          (EM_FFT_SIZE / 2)
#define EM_NUM_FEATURES      5
#define EM_MAX_CONFIRM_WIN   16       /* confirm_window 상한 */
#define EM_MAX_DRIFT_WIN     120      /* drift 롤링 윈도 상한 */
#define EM_STD_FLOOR         1e-9f    /* 0-분산 특징 나눗셈 보호 */

/* ACTIVE_FEATURES — baseline_detector.py 와 동일한 순서/의미.
 * 이 배열이 유일한 진실(single source of truth)이며,
 * 시각화·비교 모듈은 전부 이 인덱스를 import 해서 쓴다.
 * (파이썬에서 reference_comparison/visualize 가 로컬 리스트를
 *  재정의하다 어긋났던 문제의 C측 재발 방지책) */
typedef enum {
    EM_F_RMS = 0,          /* 전체 진동 에너지 (시간영역) */
    EM_F_H1_ENERGY,        /* 1x 대역 — 불평형(unbalance) */
    EM_F_H2_ENERGY,        /* 2x 대역 — 정렬불량(misalignment) */
    EM_F_H3_ENERGY,        /* 3x 대역 — 기계적 이완(looseness, 2x/3x/4x 계열) */
    EM_F_HF_ENERGY         /* 고주파 대역 — 보조 지표.
                            * 주의: MPU-6050 대역폭(~1kHz) 한계로
                            * 베어링 정밀 진단이 아님. 조기경보 보조용. */
} em_feature_id_t;

extern const char *EM_FEATURE_NAMES[EM_NUM_FEATURES];

/* ---- 런타임 설정 ---- */
typedef struct {
    /* 신호 취득 */
    float sample_rate_hz;      /* 실측 샘플링 주파수 (ESP32에서 실측치로 갱신) */

    /* 설비 명판 정보 — 유일한 필수 사용자 입력 */
    float rated_rpm;           /* 명판 정격 RPM. 예: 3000 → 50Hz */

    /* 회전수 추정(rotation_hz_estimate) — gap ① */
    float rot_search_lo_ratio; /* 탐색 하한 = 정격Hz * lo_ratio */
    float rot_search_hi_ratio; /* 탐색 상한 = 정격Hz * hi_ratio */
    float rot_max_jump_hz;     /* 이전 추정 대비 이 이상 점프 → 노이즈로 무시 */

    /* 고조파 대역 (rotation_hz_estimate 기준 상대 배치) */
    float harmonic_bw_hz;      /* 각 고조파 대역 반폭(±) */
    float hf_cutoff_ratio;     /* 고주파 대역 시작 = 정격Hz * ratio (예: 4.5) */

    /* 3-시그마 판정 */
    float sigma_threshold;     /* 기본 3.0 */
    int   anomaly_min_features;/* 동시 이탈 최소 특징 수 (오경보 저감) */
    int   confirm_window;      /* 최근 N회 판정 버퍼 크기 */
    int   confirm_threshold;   /* N회 중 M회 이상 이탈 → is_anomaly */

    /* 드리프트 감시 (구조만 확정, 파라미터는 실데이터 후 튜닝) */
    int   drift_window;        /* 롤링 평균 윈도 크기 */
    float drift_sigma;         /* 롤링평균이 원본 baseline 대비 이탈 허용 σ */

    /* --- 실무 투입용 파라미터 --- */
    float min_rel_std;         /* std 하한 = mean * 이 비율.
                                * 조용한 대역(에너지 극소·분산 극소)의
                                * z 폭발(사소한 변동→수백 σ) 오경보 방지 */
    float run_snr_threshold;   /* 회전 피크 SNR 이 이 값 미만 → 설비 정지로 판별.
                                * 정지 설비에 3σ 판정을 하면 "설비를 껐다"가
                                * "이상"으로 오경보되는 것을 구조적으로 차단 */
    int   run_hysteresis;      /* 가동/정지 상태 전환에 필요한 연속 윈도 수 */
    int   warmup_windows;      /* 학습 시작 시 버리는 기동 과도 구간 윈도 수 */
} em_config_t;

/* 합리적 기본값으로 초기화. rated_rpm 만 설비에 맞게 바꾸면 동작. */
void em_config_default(em_config_t *cfg, float rated_rpm);

/* 파생값 헬퍼 */
static inline float em_rated_hz(const em_config_t *c) {
    return c->rated_rpm / 60.0f;
}

#ifdef __cplusplus
}
#endif
#endif /* EM_CONFIG_H */
