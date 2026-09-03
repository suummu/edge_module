/*
 * em_fft.h — in-place radix-2 실수 FFT (float)
 *
 * 외부 라이브러리(arduinoFFT, scipy) 의존 제거 → PC/ESP32 동일 결과 보장.
 * N=1024, float 기준 ESP32(240MHz, FPU)에서 수 ms 수준.
 */
#ifndef EM_FFT_H
#define EM_FFT_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 시간영역 신호 → 진폭 스펙트럼.
 *  - signal[EM_FFT_SIZE] : 원시 샘플 (내부에서 평균 제거 + Hann 창 적용, 파괴적)
 *  - mag[EM_NUM_BINS]    : 출력. mag[k] = |X_k| * (2/N) / hann_coherent_gain
 *                          → 정현파 진폭에 대응하는 스케일 (파이썬 구현과 동일 규약)
 * 반환: 창 적용 전(평균 제거 후) 신호의 RMS — EM_F_RMS 특징으로 사용.
 */
float em_fft_spectrum(float *signal, float *mag);

/* bin 해상도 (Hz/bin) */
static inline float em_bin_hz(const em_config_t *c) {
    return c->sample_rate_hz / (float)EM_FFT_SIZE;
}

#ifdef __cplusplus
}
#endif
#endif
