/* em_detector.h — 정상 baseline 3-시그마 검출기 (baseline_detector.py 의 C 포팅)
 *  1. Python deque(maxlen=confirm_window) → C 원형 버퍼:
 *     write_index 모듈로 순환 + filled_count 로 초기화 안된 메모리 읽어서 버그 차단
 *  2. Python 의 if not self.fitted: self.original_means
 *     C 에서는 has_original_baseline 불리언 플래그로 명시 보호.
 *     재 학습 과정에서 초기값 덮어씌워지는 이슈 막기 위함
 * 학습 통계는 Welford 온라인 알고리즘 사용 */
#ifndef EM_DETECTOR_H
#define EM_DETECTOR_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // baseline 통계 
    float mean[EM_NUM_FEATURES];
    float stdv[EM_NUM_FEATURES];
    bool  fitted; // 학습이 한번이라도 완료되었는지 여부

    // Welford 누적기 (fit 진행 중) 
    int    fit_n;                    // 현재까지 수집된 샘플 수
    double w_mean[EM_NUM_FEATURES];  // 현재 평균
    double w_m2[EM_NUM_FEATURES];    // 편차 제곱의 

    // 드리프트 기준점 — 최초 fit 시 단 1회 기록 
    float original_means[EM_NUM_FEATURES];
    bool  has_original_baseline;     // True

    // 판정 확인 원형 버퍼 (deque 대체) 소프트웨어 디바운싱
    uint8_t flag_buf[EM_MAX_CONFIRM_WIN];
    int     flag_write_idx;
    int     flag_filled;      // 초기 채움 추적 

    // 드리프트 롤링 윈도 원형 버퍼 
    float   drift_buf[EM_MAX_DRIFT_WIN][EM_NUM_FEATURES];  // 이동 평균(Rolling Mean)
    int     drift_write_idx;
    int     drift_filled;
} em_detector_t;

typedef struct {
    float z[EM_NUM_FEATURES];   // 특징별 z-score 
    int   n_deviated;           // |z| > sigma_threshold 특징 수 
    bool  instant_flag;         // 이번 윈도 단독 판정 
    bool  is_anomaly;           // confirm_window 확정 판정 
    bool  drift_warning;        // 완만한 baseline 이탈 경고 
} em_verdict_t;

/* baseline Snapshot (ESP32 NVS / 파일 공용, 순수 C 직렬화)
 * 정전·재부팅 후 재학습 없이 감시를 재개를 위한 비휘발성 메모리 백업
 * magic/version/특징 수/체크섬 검증으로 손상·버전 불일치 로드를 거부 */
#define EM_SNAPSHOT_MAGIC   0x454D4231u   /* "EMB1" */
#define EM_SNAPSHOT_VERSION 1u

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
    uint32_t checksum;    // 데이터 무결성 검증용
} em_snapshot_t;

void em_detector_init(em_detector_t *d);

// 학습 (정상 데이터만) 
void em_detector_fit_begin(em_detector_t *d);
void em_detector_fit_add(em_detector_t *d, const float *features);
// 반환: 학습 성공 여부 (표본 2개 미만이면 실패) 실시간으로 한 개씩 학습
bool em_detector_fit_end(em_detector_t *d, const em_config_t *cfg);
void em_detector_clear_runtime(em_detector_t *d);
// 설비 Off -> On일때 호출(이전 발생 노이즈 데이터 포함 X)

// 영속화 
void em_detector_save(const em_detector_t *d, em_snapshot_t *snap);
// 반환: 검증 통과 및 복원 성공 여부 
bool em_detector_load(em_detector_t *d, const em_snapshot_t *snap);

// 추론 
void em_detector_update(em_detector_t *d,
                        const float *features,
                        const em_config_t *cfg,
                        em_verdict_t *out);

#ifdef __cplusplus
}
#endif
#endif
