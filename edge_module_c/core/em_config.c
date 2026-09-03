#include "em_config.h"

const char *EM_FEATURE_NAMES[EM_NUM_FEATURES] = {
    "rms",
    "harmonic1_ratio",
    "harmonic2_ratio",
    "harmonic3_ratio",
    "high_freq_ratio"
};

void em_config_default(em_config_t *cfg, float rated_rpm)
{
    cfg->sample_rate_hz      = 1000.0f;
    cfg->rated_rpm           = rated_rpm;

    cfg->rot_search_lo_ratio = 0.6f;
    cfg->rot_search_hi_ratio = 1.4f;
    cfg->rot_max_jump_hz     = 5.0f;
    cfg->rot_relock_windows  = 5;      /* 약 5초간 유지되면 새 운전점 */

    /* [v2] 반폭 = 물리 허용오차 5Hz + Hann 주엽 2bin (1024pt@1kHz ≈ 0.977Hz/bin) */
    cfg->harmonic_bw_hz      = 5.0f + 2.0f * (1000.0f / (float)EM_FFT_SIZE);
    cfg->hf_cutoff_ratio     = 4.5f;   /* 4x 고조파 위쪽부터 고주파 취급 */
    cfg->sensor_bw_hz        = 260.0f; /* MPU-6050 DLPF=0 가속도 대역 */

    cfg->sigma_threshold     = 3.0f;
    cfg->anomaly_min_features= 1;
    cfg->confirm_window      = 5;
    cfg->confirm_threshold   = 4;

    cfg->drift_window        = 60;
    cfg->drift_sigma         = 2.0f;

    cfg->min_rel_std         = 0.02f;  /* std >= mean 의 2% */
    cfg->run_snr_threshold   = 5.0f;   /* 피크/대역평균 비 */
    cfg->run_hysteresis      = 3;
    cfg->warmup_windows      = 5;
}
