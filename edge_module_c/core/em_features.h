/*
 * em_features.h — ACTIVE_FEATURES 추출
 *
 * 설계 원칙: 3대 타겟 결함(불평형/정렬불량/이완)에 직접 대응하는
 * 특징만 유지한다. peak_freq / peak_magnitude / kurtosis 는
 * 판별용으로 deprecated (파이썬 gap ⑤ 결정과 동일).
 *
 * 고조파 대역은 고정 Hz 가 아니라 rotation_hz_estimate 를
 * 기준축으로 상대 배치한다 → 부하로 RPM 이 흔들려도 대역이
 * 고조파를 놓치지 않는다.
 */
#ifndef EM_FEATURES_H
#define EM_FEATURES_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * mag[EM_NUM_BINS] 스펙트럼 + rotation_hz 로 특징 벡터 채움.
 *  - rms 는 em_fft_spectrum() 반환값을 그대로 전달받아 기록.
 *  - out[EM_NUM_FEATURES] 순서는 em_feature_id_t 와 동일.
 */
void em_extract_features(const float *mag,
                         float rotation_hz,
                         float rms,
                         const em_config_t *cfg,
                         float *out);

#ifdef __cplusplus
}
#endif
#endif
