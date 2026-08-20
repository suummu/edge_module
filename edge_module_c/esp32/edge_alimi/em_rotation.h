/*
 * em_rotation.h — rotation_hz_estimate (gap ①) + 가동 판별 SNR
 *
 * 역할: "판별 특징"이 아니라 모든 고조파 대역 배치의 기준축.
 *  - deprecated 된 peak_freq 와 구분됨: 전 대역 최대 피크가 아니라
 *    명판 정격 RPM 파생 탐색범위 내에서만 최대 피크를 찾는다.
 *  - 이전 추정치 대비 max_jump_hz 초과 점프는 노이즈로 간주하고
 *    이전 값을 유지한다 (부하 변동에 의한 완만한 드리프트만 추종).
 *  - 단, 거부만 하고 해제 조건이 없으면 가변전압/VFD 로 회전수를 한 번에
 *    크게 바꿨을 때 이후 모든 후보가 계속 거부되어 기준축이 옛 값에 영구
 *    고정된다(하모닉 대역이 엉뚱한 중심에 놓여 특징값 전체가 무의미해짐).
 *    노이즈는 산발적이고 실제 속도 변화는 지속된다는 성질로 둘을 가른다 —
 *    같은 자리에 rot_relock_windows 회 연속 나타난 후보는 새 운전점으로
 *    수용한다(re-lock).
 *  - A안 한계 (문서화 필수): PLC 없이 진동 신호 자체의 1x 성분으로
 *    RPM 을 근사하므로, 실시간 부하 급변은 반영 한계가 있다.
 *
 * [실무 추가] SNR = 탐색대역 내 피크 / (피크 주변 제외 대역 평균).
 *  설비가 돌고 있으면 1x 피크가 대역 평균을 크게 상회하고,
 *  정지 상태면 스펙트럼이 평탄해 SNR 이 낮다 → 가동/정지 판별 근거.
 *  정지 판별 시 회전 추정은 갱신하지 않는다 (노이즈 피크 추종 방지).
 */
#ifndef EM_ROTATION_H
#define EM_ROTATION_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float hz;         /* 현재 확정 추정치 */
    bool  valid;      /* 최초 추정 완료 여부 */
    float pending_hz; /* 거부 중인 후보 (연속성 확인용) */
    int   rejected;   /* 같은 자리에서 연속 거부된 횟수.
                       * 정상 추종 시 0으로 리셋되므로 누적 카운터가 아니라
                       * "지금 락이 걸려 있나"를 나타내는 상태값이다. */
} em_rotation_t;

void  em_rotation_init(em_rotation_t *rt);

/* 탐색범위 내 피크 후보와 SNR 측정 (상태 변경 없음).
 * 반환: 후보 주파수(Hz). snr_out 에 피크/대역평균 비 기록. */
float em_rotation_measure(const float *mag,
                          const em_config_t *cfg,
                          float *snr_out);

/* 후보를 점프 필터에 통과시켜 추정치 갱신. 반환: 현재 추정 Hz */
float em_rotation_accept(em_rotation_t *rt,
                         float candidate_hz,
                         const em_config_t *cfg);

#ifdef __cplusplus
}
#endif
#endif
