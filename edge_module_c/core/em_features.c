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

    out[EM_F_RMS]       = rms;

    /* 1x — 불평형(unbalance) */
    out[EM_F_H1_ENERGY] = band_energy(mag, 1.0f * f1 - bw, 1.0f * f1 + bw, bin_hz);
    /* 2x — 정렬불량(misalignment) */
    out[EM_F_H2_ENERGY] = band_energy(mag, 2.0f * f1 - bw, 2.0f * f1 + bw, bin_hz);
    /* 3x — 기계적 이완(looseness). 이완은 2x/3x/4x 계열로 나타나므로
     * H2 와 함께 상승하는 패턴이 이완의 시그니처가 된다. */
    out[EM_F_H3_ENERGY] = band_energy(mag, 3.0f * f1 - bw, 3.0f * f1 + bw, bin_hz);

    /* 고주파 대역 — 보조 지표.
     * [정직성 주석 / gap ④] MPU-6050 (I2C, 실효 ~수백 Hz 대역) 으로는
     * 베어링 결함의 본진(1~20kHz 충격, 포락선 분석 필요)을 진단할 수 없다.
     * 이 특징은 "고주파 쪽 에너지가 평소와 달라졌다"는 조기경보 보조
     * 신호일 뿐, 베어링 정밀 진단 기능이 아니다.
     *
     * [의도된 예외] 이 경계(hf_lo)만은 실측 rotation_hz 가 아니라
     * 명판 정격(rated_hz) 기준 고정이다 — H1/H2/H3 상대 배치 원칙의
     * 유일한 예외. 고주파 대역은 "평소 대비 총 에너지 변화"를 보는
     * 광대역 지표라 경계가 회전수 추정 노이즈를 따라 흔들리면 오히려
     * baseline 분산만 커진다. 단, 인버터로 정격 대비 크게 감속 상시
     * 운전하는 설비에서는 4x 고조파와 이 경계 사이 간극이 벌어짐을 유의. */
    float hf_lo = em_rated_hz(cfg) * cfg->hf_cutoff_ratio;
    float nyq   = cfg->sample_rate_hz * 0.5f;
    out[EM_F_HF_ENERGY] = band_energy(mag, hf_lo, nyq, bin_hz);
}
