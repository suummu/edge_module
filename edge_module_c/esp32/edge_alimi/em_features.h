/*
 * em_features.h — ACTIVE_FEATURES 추출
 * 설계 원칙: 3대 타겟 결함(불평형/정렬불량/이완)에 직접 대응, peak_freq / peak_magnitude / kurtosis 는 판별용으로 deprecated
 * 고주파 대역은 고정 Hz 가 아니라 rotation_hz_estimate 를 기준축으로 상대 배치한다 ( 하드코딩을 진행했을때 슬립 현상에 대응 못하는 경우 방지)
 * → 부하로 RPM 이 흔들려도 대역이 고주파를 놓치지 않는다(상대 배치 설계) */
#ifndef EM_FEATURES_H
#define EM_FEATURES_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* mag배열 + rotation_hz(현재 모터 회전수)
 *  - rms 는 em_fft_spectrum() 반환값
 *  - out 배열은 em_feature_id_t(0번 RMS, 1번 1X 에너지...)순서 */
void em_extract_features(const float *mag,
                         float rotation_hz, // 시스템 추정 모터의 현재 1회전 주파수
                         float rms,
                         const em_config_t *cfg,
                         float *out);

#ifdef __cplusplus
}
#endif
#endif
