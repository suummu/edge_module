/*
 * em_detector.h — 정상 baseline 3-시그마 검출기 (baseline_detector.py 의 C 포팅)
 *
 * [ESP32 포팅 체크리스트 반영 사항 — gap ⑥]
 *  1. Python deque(maxlen=confirm_window) → C 원형 버퍼:
 *     write_index 모듈로 순환 + filled_count 로 초기 채움 상태 추적.
 *     (filled_count 없이 읽으면 가동 초기 confirm_window 사이클 동안
 *      미초기화 슬롯을 읽는 버그 발생 — 여기서 구조적으로 차단)
 *  2. Python 의 `if not self.fitted: self.original_means = ...` →
 *     C 에서는 has_original_baseline 불리언 플래그로 명시 보호.
 *     ★ 포팅 후 최우선 점검 항목: 재-fit 때마다 original_means 가
 *       덮어써지면 드리프트 감지 전체가 조용히 무력화된다.
 *       em_detector_fit_end() 는 이 플래그가 false 일 때 단 1회만 기록한다.
 *
 * 학습 통계는 Welford 온라인 알고리즘 사용 (합/제곱합 방식의
 * float 누적 오차 방지 — 장시간 baseline 수집에서 중요).
 */
#ifndef EM_DETECTOR_H
#define EM_DETECTOR_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* baseline 통계 */
    float mean[EM_NUM_FEATURES];
    float stdv[EM_NUM_FEATURES];
    bool  fitted;

    /* Welford 누적기 (fit 진행 중) */
    int    fit_n;
    double w_mean[EM_NUM_FEATURES];
    double w_m2[EM_NUM_FEATURES];

    /* 드리프트 기준점 — 최초 fit 시 단 1회 기록 */
    float original_means[EM_NUM_FEATURES];
    bool  has_original_baseline;

    /* 판정 확인 원형 버퍼 (deque 대체) */
    uint8_t flag_buf[EM_MAX_CONFIRM_WIN];
    int     flag_write_idx;
    int     flag_filled;      /* ★ 초기 채움 추적 */

    /* 드리프트 롤링 윈도 원형 버퍼 */
    float   drift_buf[EM_MAX_DRIFT_WIN][EM_NUM_FEATURES];
    int     drift_write_idx;
    int     drift_filled;
} em_detector_t;

typedef struct {
    float z[EM_NUM_FEATURES];   /* 특징별 z-score */
    int   n_deviated;           /* |z| > sigma_threshold 특징 수 */
    bool  instant_flag;         /* 이번 윈도 단독 판정 */
    bool  is_anomaly;           /* confirm_window 확정 판정 */
    bool  drift_warning;        /* 완만한 baseline 이탈 경고 */
} em_verdict_t;

/*
 * 스냅샷 — baseline 영속화 (ESP32 NVS / 파일 공용, 순수 C 직렬화)
 * 정전·재부팅 후 재학습 없이 감시를 재개하기 위한 실무 필수 기능.
 * magic/version/특징 수/체크섬 검증으로 손상·버전 불일치 로드를 거부.
 */
#define EM_SNAPSHOT_MAGIC   0x454D4231u   /* "EMB1" */
#define EM_SNAPSHOT_VERSION 2u   /* [v2] baseline 이 log 공간 + 비율 특징 —
                                  * v1 스냅샷(선형·절대 에너지)과 비호환이므로
                                  * 버전 승급으로 구버전 로드를 거부한다 */

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t n_features;
    float    mean[EM_NUM_FEATURES];
    float    stdv[EM_NUM_FEATURES];
    float    original_means[EM_NUM_FEATURES];
    uint8_t  has_original_baseline;
    uint8_t  fitted;
    uint16_t _pad;
    uint32_t checksum;    /* checksum 필드 제외 전 바이트 합 */
} em_snapshot_t;

void em_detector_init(em_detector_t *d);

/* --- 학습 (정상 데이터만) --- */
void em_detector_fit_begin(em_detector_t *d);
void em_detector_fit_add(em_detector_t *d, const float *features);
/* 반환: 학습 성공 여부 (표본 2개 미만이면 실패).
 * cfg->min_rel_std 로 상대 std 하한 적용 — 조용한 특징의 z 폭발 방지 */
bool em_detector_fit_end(em_detector_t *d, const em_config_t *cfg);

/* 판정/드리프트 원형 버퍼만 리셋 (baseline 유지).
 * 설비 정지→재가동 등 상태 전환 시 낡은 플래그 오염 방지용 */
void em_detector_clear_runtime(em_detector_t *d);

/* --- 영속화 --- */
void em_detector_save(const em_detector_t *d, em_snapshot_t *snap);
/* 반환: 검증 통과 및 복원 성공 여부 */
bool em_detector_load(em_detector_t *d, const em_snapshot_t *snap);

/* --- 추론 --- */
void em_detector_update(em_detector_t *d,
                        const float *features,
                        const em_config_t *cfg,
                        em_verdict_t *out);

#ifdef __cplusplus
}
#endif
#endif
