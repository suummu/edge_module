/*
  edgealimi.c — 엣지알리미 판별 파이프라인의 순수 C(C99) 구현.
  자세한 설계 배경과 파이썬 대응 관계는 edgealimi.h 참고.

  numpy/scipy가 없는 환경이므로 FFT까지 직접 구현했다.
  FFT는 radix-2 DIT(decimation-in-time) 반복형 — 재귀 없음(스택 안전),
  동적 할당 없음(임베디드 안전), N=EA_WINDOW_SIZE 고정.
*/

#include "edgealimi.h"

/* M_PI는 POSIX 확장이라 strict C99(-std=c99)에서는 정의되지 않을 수 있음 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const char *EA_FEATURE_NAMES[EA_N_FEATURES] = {
    "rms", "harmonic1_energy", "harmonic2_energy", "harmonic3_energy", "high_freq_energy"
};

/* ==================== DC 제거 ==================== */
void ea_remove_dc(float *signal, uint16_t n)
{
    float mean = 0.0f;
    for (uint16_t i = 0; i < n; i++) mean += signal[i];
    mean /= n;
    for (uint16_t i = 0; i < n; i++) signal[i] -= mean;
}

/* ==================== FFT (radix-2 DIT, 반복형) ==================== */
/* 내부 작업 버퍼 — 동적 할당 대신 정적 할당 (임베디드 관례. 재진입 불가에 유의) */
static float fft_re[EA_WINDOW_SIZE];
static float fft_im[EA_WINDOW_SIZE];

/* 비트 반전: N=2^m 기준으로 인덱스의 하위 m비트를 뒤집는다 */
static uint16_t bit_reverse(uint16_t x, uint16_t log2n)
{
    uint16_t r = 0;
    for (uint16_t i = 0; i < log2n; i++) {
        r = (uint16_t)((r << 1) | (x & 1));
        x >>= 1;
    }
    return r;
}

void ea_compute_fft_magnitudes(const float *signal, float *out_magnitudes)
{
    const uint16_t n = EA_WINDOW_SIZE;

    /* log2(n) 계산 (n은 2의 거듭제곱이어야 함) */
    uint16_t log2n = 0;
    while ((1u << log2n) < n) log2n++;

    /* 1) 비트 반전 순서로 입력 복사 */
    for (uint16_t i = 0; i < n; i++) {
        uint16_t j = bit_reverse(i, log2n);
        fft_re[j] = signal[i];
        fft_im[j] = 0.0f;
    }

    /* 2) 버터플라이 연산: 길이 2, 4, 8, ... n 순으로 병합 */
    for (uint16_t stage = 1; stage <= log2n; stage++) {
        uint16_t m = (uint16_t)(1u << stage);       /* 현재 병합 블록 길이 */
        uint16_t half = (uint16_t)(m >> 1);
        /* 회전 인자(twiddle): w = exp(-2*pi*i/m), 순방향 FFT이므로 음의 지수 */
        float w_step_re = cosf(-2.0f * (float)M_PI / (float)m);
        float w_step_im = sinf(-2.0f * (float)M_PI / (float)m);

        for (uint16_t block = 0; block < n; block += m) {
            float w_re = 1.0f, w_im = 0.0f;
            for (uint16_t k = 0; k < half; k++) {
                uint16_t top = (uint16_t)(block + k);
                uint16_t bot = (uint16_t)(block + k + half);

                float t_re = w_re * fft_re[bot] - w_im * fft_im[bot];
                float t_im = w_re * fft_im[bot] + w_im * fft_re[bot];

                fft_re[bot] = fft_re[top] - t_re;
                fft_im[bot] = fft_im[top] - t_im;
                fft_re[top] += t_re;
                fft_im[top] += t_im;

                /* w *= w_step (복소 곱) */
                float next_w_re = w_re * w_step_re - w_im * w_step_im;
                w_im = w_re * w_step_im + w_im * w_step_re;
                w_re = next_w_re;
            }
        }
    }

    /* 3) magnitude + 파이썬과 동일한 정규화 (x2/N), 양의 주파수 절반만 */
    const float norm = 2.0f / (float)n;
    for (uint16_t i = 0; i < EA_HALF_SIZE; i++) {
        out_magnitudes[i] = sqrtf(fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i]) * norm;
    }
}

/* ==================== 대역 에너지 ==================== */
float ea_frequency_band_energy(const float *magnitudes, float center_hz, float band_width_hz)
{
    float sum = 0.0f;
    for (uint16_t i = 1; i < EA_HALF_SIZE; i++) {   /* i=0(DC) 제외 */
        float freq = ea_bin_to_hz(i);
        if (freq >= center_hz - band_width_hz && freq <= center_hz + band_width_hz) {
            sum += magnitudes[i];
        }
    }
    return sum;
}

/* ==================== 회전 주파수 추정 ==================== */
float ea_estimate_rotation_hz(const float *magnitudes, float previous_estimate)
{
    float best_mag = -1.0f;
    float candidate = EA_NO_ESTIMATE;
    for (uint16_t i = 1; i < EA_HALF_SIZE; i++) {
        float freq = ea_bin_to_hz(i);
        if (freq < EA_ROT_SEARCH_LOW_HZ || freq > EA_ROT_SEARCH_HIGH_HZ) continue;
        if (magnitudes[i] > best_mag) {
            best_mag = magnitudes[i];
            candidate = freq;
        }
    }
    if (isnan(candidate)) return previous_estimate;              /* 탐색범위 내 피크 없음 */
    if (isnan(previous_estimate)) return candidate;              /* 첫 추정 */
    if (fabsf(candidate - previous_estimate) > EA_ROT_MAX_JUMP_HZ) return previous_estimate;
    return candidate;
}

/* ==================== 특징 추출 ==================== */
static float feature_magnitudes[EA_HALF_SIZE];   /* FFT 결과 버퍼 (정적 할당) */

void ea_extract_features(const float *signal, float *out_features,
                         float *rotation_estimate_io, bool independent)
{
    /* RMS */
    float sum_squares = 0.0f;
    for (uint16_t i = 0; i < EA_WINDOW_SIZE; i++) sum_squares += signal[i] * signal[i];
    out_features[EA_FEAT_RMS] = sqrtf(sum_squares / EA_WINDOW_SIZE);

    ea_compute_fft_magnitudes(signal, feature_magnitudes);

    float prev = independent ? EA_NO_ESTIMATE : *rotation_estimate_io;
    float est = ea_estimate_rotation_hz(feature_magnitudes, prev);
    /* 파이썬과 동일한 폴백: 추정 실패(None) 시 정격 회전주파수 */
    if (isnan(est)) est = EA_FAN_ROTATION_HZ;
    if (!independent) *rotation_estimate_io = est;

    out_features[EA_FEAT_H1] = ea_frequency_band_energy(feature_magnitudes, est * 1.0f, 5.0f);
    out_features[EA_FEAT_H2] = ea_frequency_band_energy(feature_magnitudes, est * 2.0f, 5.0f);
    out_features[EA_FEAT_H3] = ea_frequency_band_energy(feature_magnitudes, est * 3.0f, 5.0f);
    out_features[EA_FEAT_HF] = ea_frequency_band_energy(feature_magnitudes, 325.0f, 75.0f);
}

/* ==================== 판별기 ==================== */
void ea_detector_init(ea_detector_t *det)
{
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        det->means[k] = 0.0f;
        det->stds[k] = 0.0f;
        det->original_means[k] = 0.0f;
    }
    det->fitted = false;
    det->has_original_baseline = false;
    ea_detector_reset_confirmation(det);
}

void ea_detector_fit(ea_detector_t *det,
                     const float features[][EA_N_FEATURES], uint16_t n_windows)
{
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        float sum = 0.0f;
        for (uint16_t w = 0; w < n_windows; w++) sum += features[w][k];
        float mean = sum / n_windows;

        float sum_sq_diff = 0.0f;
        for (uint16_t w = 0; w < n_windows; w++) {
            float d = features[w][k] - mean;
            sum_sq_diff += d * d;
        }
        det->means[k] = mean;
        det->stds[k] = sqrtf(sum_sq_diff / n_windows);  /* np.std와 동일 (n으로 나눔) */
    }

    /* 원본 baseline은 최초 1회만 기록 (파이썬 `if not self.fitted:` 대응).
       이 보호가 없으면 재보정 때마다 원본이 덮여서 위험 정상화 방지가 무력화됨. */
    if (!det->has_original_baseline) {
        for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
            det->original_means[k] = det->means[k];
        }
        det->has_original_baseline = true;
    }

    det->fitted = true;
    ea_detector_reset_confirmation(det);   /* 새 baseline이므로 이력 초기화 */
}

void ea_detector_reset_confirmation(ea_detector_t *det)
{
    det->suspicion_write_idx = 0;
    det->suspicion_filled_count = 0;
}

ea_judge_result_t ea_detector_judge(ea_detector_t *det,
                                    const float features[EA_N_FEATURES])
{
    ea_judge_result_t r;
    r.is_suspect = false;
    r.is_anomaly = false;
    r.absolute_limit_hit = false;
    r.suspicion_rate = 0.0f;

    /* 1) 상대적 판정 (3σ 규칙) */
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        float lower = det->means[k] - EA_N_SIGMA * det->stds[k];
        float upper = det->means[k] + EA_N_SIGMA * det->stds[k];
        r.feat_out[k] = (features[k] < lower || features[k] > upper);
        if (r.feat_out[k]) r.is_suspect = true;
    }

    /* 2) 절대적 판정 (데이터시트 고정값 — NAN이면 비활성) */
    if (!isnan(EA_ABSOLUTE_RMS_LIMIT) && features[EA_FEAT_RMS] > EA_ABSOLUTE_RMS_LIMIT) {
        r.absolute_limit_hit = true;
        r.is_suspect = true;
    }

    /* 3) 연속성 필터 — 원형 버퍼에 기록 후 확정 여부 계산 */
    det->suspicion_history[det->suspicion_write_idx] = r.is_suspect;
    det->suspicion_write_idx = (uint8_t)((det->suspicion_write_idx + 1) % EA_CONFIRM_WINDOW);
    if (det->suspicion_filled_count < EA_CONFIRM_WINDOW) det->suspicion_filled_count++;

    uint8_t suspect_count = 0;
    for (uint8_t i = 0; i < det->suspicion_filled_count; i++) {
        if (det->suspicion_history[i]) suspect_count++;
    }
    r.suspicion_rate = (float)suspect_count / det->suspicion_filled_count;

    /* 파이썬: len(deque)==confirm_window AND rate>=ratio.
       filled_count 조건이 없으면 부팅 직후 1~2개만으로 확정이 떠버린다. */
    r.is_anomaly = (det->suspicion_filled_count == EA_CONFIRM_WINDOW)
                   && (r.suspicion_rate >= EA_CONFIRM_RATIO);

    /* 4) 위험 정상화 감시 (원본 baseline 대비 드리프트) */
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        r.drift_warn[k] = false;
        float orig = det->original_means[k];
        if (fabsf(orig) < 1e-9f) continue;
        if (features[k] / orig >= EA_DRIFT_WARNING_RATIO) r.drift_warn[k] = true;
    }

    return r;
}
