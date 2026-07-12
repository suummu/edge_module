#include "em_fft.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 복소 작업 버퍼 — 정적 할당 (ESP32 스택 보호) */
static float s_re[EM_FFT_SIZE];
static float s_im[EM_FFT_SIZE];

/* Hann 창 코히어런트 게인 = 0.5 (진폭 보정용) */
#define HANN_COHERENT_GAIN 0.5f

static void fft_inplace(float *re, float *im, int n)
{
    /* 비트 반전 재배열 */
    int j = 0;
    for (int i = 0; i < n - 1; i++) {
        if (i < j) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
        int m = n >> 1;
        while (j >= m && m > 0) { j -= m; m >>= 1; }
        j += m;
    }
    /* 버터플라이 */
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wr = cosf(ang), wi = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            int half = len >> 1;
            for (int k = 0; k < half; k++) {
                int a = i + k, b = i + k + half;
                float tr = cr * re[b] - ci * im[b];
                float ti = cr * im[b] + ci * re[b];
                re[b] = re[a] - tr;  im[b] = im[a] - ti;
                re[a] += tr;         im[a] += ti;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

float em_fft_spectrum(float *signal, float *mag)
{
    const int n = EM_FFT_SIZE;

    /* 1) DC(평균) 제거 — 센서 오프셋/중력 성분 제거 */
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += signal[i];
    mean /= (float)n;

    /* 2) RMS (창 적용 전, 평균 제거 후) */
    float ss = 0.0f;
    for (int i = 0; i < n; i++) {
        signal[i] -= mean;
        ss += signal[i] * signal[i];
    }
    float rms = sqrtf(ss / (float)n);

    /* 3) Hann 창 + 복소 버퍼 적재 */
    for (int i = 0; i < n; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1)));
        s_re[i] = signal[i] * w;
        s_im[i] = 0.0f;
    }

    fft_inplace(s_re, s_im, n);

    /* 4) 진폭 스펙트럼 (단측, 진폭 규약) */
    const float scale = (2.0f / (float)n) / HANN_COHERENT_GAIN;
    for (int k = 0; k < EM_NUM_BINS; k++) {
        mag[k] = sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]) * scale;
    }
    mag[0] *= 0.5f; /* DC는 단측 2배 보정 대상 아님 */

    return rms;
}
