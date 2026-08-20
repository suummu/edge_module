/* em_fft.h — in-place radix-2 실수 FFT (float)
 * 외부 라이브러리(arduinoFFT, scipy) 의존 제거 → PC/ESP32 동일 결과 보장.
 * N=1024, float 기준 ESP32(240MHz, FPU)에서 수 ms 수준.  */
#ifndef EM_FFT_H
#define EM_FFT_H

#include "em_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*  시간영역 신호 → 진폭 스펙트럼.
 *  - signal[EM_FFT_SIZE] : 원시 샘플 신호 중심 0으로 설정 (내부에서 평균 제거 + Hann 창 적용 주파수 번짐 현상 억) 
 *  - mag[EM_NUM_BINS]    : 출력. mag[k] = |X_k| * (2/N) / hann_coherent_gain
 *                          → 실제 물리적 진동 크기에 맞춰 보정
 * 반환: 창 적용 전(평균 제거 후) 신호의 RMS — EM_F_RMS 특징으로 사용. */
float em_fft_spectrum(float *signal, float *mag);

/* float *signal: 1024개의 쌩 진동 데이터가 들어있는 배열
 * float *mag: FFT 결과(주파수별 진폭)512개 담을 빈 배열
 * 반환형 float: FFT 진행 전 RMS 값 계산해서 리턴 (이중 배열 순환 방지) */

// FFT 결과 배열(mag)의 인덱스 1칸(Bin)이 실제 물리적으로 몇 HZ인지 계산
static inline float em_bin_hz(const em_config_t *c) {
    return c->sample_rate_hz / (float)EM_FFT_SIZE; // 샘플링 주파수 / FFT 크기
}

#ifdef __cplusplus
}
#endif
#endif
