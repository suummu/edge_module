/*
 * em_pipeline.h — 전체 처리 흐름 통합
 *
 *   샘플 윈도(EM_FFT_SIZE) → FFT → 가동/정지 판별 → rotation_hz 추정
 *   → 특징 추출 → (학습: fit_add / 감시: 3σ 판정)
 *
 * ESP32 스케치와 PC 테스트 하니스가 동일하게 이 API 만 호출한다.
 *
 * [가동/정지 상태 머신 — 실무 오경보 차단의 핵심]
 *  정지된 설비는 스펙트럼이 평탄 → 회전 피크 SNR 로 판별.
 *  정지 중에는 3σ 판정과 학습을 모두 중단한다:
 *   - "설비를 껐다"가 "이상 확정"으로 오경보되는 것을 차단
 *   - 정지/기동 과도 구간 데이터가 baseline 을 오염시키는 것을 차단
 *  전환은 run_hysteresis 연속 윈도로 확정 (플래핑 방지),
 *  재가동 시 판정 버퍼를 리셋해 낡은 플래그 오염을 막는다.
 */
#ifndef EM_PIPELINE_H
#define EM_PIPELINE_H

#include "em_config.h"
#include "em_rotation.h"
#include "em_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EM_MODE_IDLE = 0,
    EM_MODE_LEARNING,     /* 정상 baseline 수집 중 */
    EM_MODE_MONITORING    /* 3σ 감시 중 */
} em_mode_t;

typedef struct {
    em_config_t   cfg;
    em_rotation_t rot;
    em_detector_t det;
    em_mode_t     mode;
    int           learn_target;   /* 목표 학습 윈도 수 */
    int           learn_count;    /* 현재 수집 수 */
    int           warmup_left;    /* 학습 전 버릴 기동 과도 윈도 수 */
    uint32_t      window_count;   /* 처리한 총 윈도 수 (seq) */

    /* 가동/정지 상태 머신 */
    bool          running;
    int           run_streak;
    int           stop_streak;
} em_pipeline_t;

typedef struct {
    uint32_t    seq;              /* 윈도 일련번호 — 수집 누락 감지용 */
    float       rotation_hz;
    float       snr;              /* 회전 피크 SNR (가동 판별 근거) */
    bool        running;          /* 설비 가동 판별 결과 */
    float       features[EM_NUM_FEATURES];
    em_verdict_t verdict;         /* MONITORING & running 에서만 유효 */
    em_mode_t   mode;
    int         learn_progress;   /* 0~100 (%) */
} em_output_t;

void em_pipeline_init(em_pipeline_t *p, float rated_rpm);

/* 정상 baseline 학습 시작. warmup_windows 만큼 버린 뒤
 * n_windows 개 수집(가동 중 윈도만) 후 자동 MONITORING 전환 */
void em_pipeline_start_learning(em_pipeline_t *p, int n_windows);

/* 윈도 하나 처리. signal 은 내부에서 파괴됨(창 적용). */
void em_pipeline_process(em_pipeline_t *p, float *signal, em_output_t *out);

/* --- baseline 영속화 (부팅 시 재학습 생략) --- */
void em_pipeline_save_baseline(const em_pipeline_t *p, em_snapshot_t *snap);
bool em_pipeline_load_baseline(em_pipeline_t *p, const em_snapshot_t *snap);

/* 결과를 JSON 한 줄로 직렬화 (시리얼/HTTP 공용). 반환: 문자열 길이 */
int em_output_to_json(const em_output_t *o, char *buf, int buflen);

#ifdef __cplusplus
}
#endif
#endif
