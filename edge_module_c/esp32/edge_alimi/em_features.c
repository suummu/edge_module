#include "em_features.h"
#include "em_fft.h"
#include <math.h>

/* [lo_hz, hi_hz] 대역 에너지(진폭 제곱합) */
static float band_energy(const float *mag, float lo_hz, float hi_hz, float bin_hz)
{
    int lo = (int)floorf(lo_hz / bin_hz);
    int hi = (int)ceilf (hi_hz / bin_hz);
    if (lo < 1) lo = 1;
    if (hi > EM_NUM_BINS - 1) hi = EM_NUM_BINS - 1;
    float e = 0.0f;
    for (int k = lo; k <= hi; k++) e += mag[k] * mag[k];
    return e;
}

void em_extract_features(const float *mag,
                         float rotation_hz,
                         float rms,
                         const em_config_t *cfg,
                         float *out)
{
    const float bin_hz = em_bin_hz(cfg);
    const float bw = cfg->harmonic_bw_hz;

    /* 회전수 추정이 아직 없으면 명판 정격으로 폴백 (콜드스타트 안전) */
    float f1 = (rotation_hz > 0.0f) ? rotation_hz : em_rated_hz(cfg);

    out[EM_F_RMS] = rms;   /* 유일한 절대량 특징 — 가동/정지·절대한계 참조용 */

    /* [v2] 전체 스펙트럼 에너지 (DC 제외) — 비율 정규화의 분모.
     * ml/features_real.py 의 spectrum_total_energy 와 동일 정의. */
    float e_total = 0.0f;
    for (int k = 1; k < EM_NUM_BINS; k++) e_total += mag[k] * mag[k];
    if (e_total < 1e-20f) e_total = 1e-20f;

    /* 1x — 불평형(unbalance) */
    out[EM_F_H1_ENERGY] = band_energy(mag, 1.0f * f1 - bw, 1.0f * f1 + bw, bin_hz) / e_total;
    /* 2x — 정렬불량(misalignment) */
    out[EM_F_H2_ENERGY] = band_energy(mag, 2.0f * f1 - bw, 2.0f * f1 + bw, bin_hz) / e_total;
    /* 3x — 기계적 이완(looseness). 이완은 2x/3x/4x 계열로 나타나므로
     * H2 와 함께 상승하는 패턴이 이완의 시그니처가 된다. */
    out[EM_F_H3_ENERGY] = band_energy(mag, 3.0f * f1 - bw, 3.0f * f1 + bw, bin_hz) / e_total;

    /* 고주파 대역 — 보조 지표 (베어링 정밀진단 아님).
     * [v2] 상한을 Nyquist 가 아니라 센서 유효대역(sensor_bw_hz - 5Hz 여유)으로
     * 제한 — DLPF 감쇠 구간의 센서 노이즈를 재지 않는다 (gap ⑤).
     * 하한은 명판 정격 기준 고정(회전수 노이즈로 경계가 흔들리는 것 방지). */
    float hf_lo = em_rated_hz(cfg) * cfg->hf_cutoff_ratio;
    float hf_hi = cfg->sample_rate_hz * 0.5f;
    float bw_cap = cfg->sensor_bw_hz - 5.0f;
    if (bw_cap > 0.0f && hf_hi > bw_cap) hf_hi = bw_cap;
    out[EM_F_HF_ENERGY] = band_energy(mag, hf_lo, hf_hi, bin_hz) / e_total;
}
