#include "em_detector.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

void em_detector_init(em_detector_t *d)
{
    memset(d, 0, sizeof(*d));
    /* memset(0) 으로 fitted=false, has_original_baseline=false,
     * filled_count 류 전부 0 — 의도된 초기 상태와 일치 */
}

void em_detector_fit_begin(em_detector_t *d)
{
    d->fit_n = 0;
    for (int i = 0; i < EM_NUM_FEATURES; i++) {
        d->w_mean[i] = 0.0;
        d->w_m2[i]   = 0.0;
    }
}

void em_detector_fit_add(em_detector_t *d, const float *features)
{
    d->fit_n++;
    for (int i = 0; i < EM_NUM_FEATURES; i++) {
        double x = (double)features[i];
        double delta = x - d->w_mean[i];
        d->w_mean[i] += delta / (double)d->fit_n;
        d->w_m2[i]   += delta * (x - d->w_mean[i]);
    }
}

bool em_detector_fit_end(em_detector_t *d, const em_config_t *cfg)
{
    if (d->fit_n < 2) return false;

    for (int i = 0; i < EM_NUM_FEATURES; i++) {
        d->mean[i] = (float)d->w_mean[i];
        float var  = (float)(d->w_m2[i] / (double)(d->fit_n - 1));
        d->stdv[i] = sqrtf(var);
        /* 상대 std 하한: 학습 구간이 유난히 조용했던 특징이
         * 사소한 변동에 z 수백을 찍는 오경보 방지.
         * (예: 조용한 고조파 대역 — 분산이 0 에 수렴할 수 있음) */
        float rel_floor = fabsf(d->mean[i]) * (cfg ? cfg->min_rel_std : 0.0f);
        if (d->stdv[i] < rel_floor)    d->stdv[i] = rel_floor;
        if (d->stdv[i] < EM_STD_FLOOR) d->stdv[i] = EM_STD_FLOOR;
    }

    /* ★ original_means 는 최초 fit 에서 단 1회만 기록.
     * 재-fit(baseline 갱신)이 일어나도 드리프트 기준점은 보존된다.
     * 이 조건이 사라지면 드리프트 감지가 조용히 무력화됨. */
    if (!d->has_original_baseline) {
        memcpy(d->original_means, d->mean, sizeof(d->original_means));
        d->has_original_baseline = true;
    }

    d->fitted = true;

    /* 판정/드리프트 버퍼는 새 baseline 기준으로 리셋 */
    em_detector_clear_runtime(d);
    return true;
}

void em_detector_clear_runtime(em_detector_t *d)
{
    d->flag_write_idx  = 0;
    d->flag_filled     = 0;
    d->drift_write_idx = 0;
    d->drift_filled    = 0;
}

/* checksum 필드 직전까지의 바이트 해시 */
static uint32_t snap_checksum(const em_snapshot_t *s)
{
    const uint8_t *p = (const uint8_t *)s;
    size_t n = offsetof(em_snapshot_t, checksum);
    uint32_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = acc * 31u + p[i];
    return acc;
}

void em_detector_save(const em_detector_t *d, em_snapshot_t *snap)
{
    memset(snap, 0, sizeof(*snap));
    snap->magic      = EM_SNAPSHOT_MAGIC;
    snap->version    = EM_SNAPSHOT_VERSION;
    snap->n_features = EM_NUM_FEATURES;
    memcpy(snap->mean,           d->mean,           sizeof(snap->mean));
    memcpy(snap->stdv,           d->stdv,           sizeof(snap->stdv));
    memcpy(snap->original_means, d->original_means, sizeof(snap->original_means));
    snap->has_original_baseline = d->has_original_baseline ? 1 : 0;
    snap->fitted                = d->fitted ? 1 : 0;
    snap->checksum = snap_checksum(snap);
}

bool em_detector_load(em_detector_t *d, const em_snapshot_t *snap)
{
    if (snap->magic != EM_SNAPSHOT_MAGIC)        return false;
    if (snap->version != EM_SNAPSHOT_VERSION)    return false;
    if (snap->n_features != EM_NUM_FEATURES)     return false;
    if (snap->checksum != snap_checksum(snap))   return false;
    if (!snap->fitted)                           return false;

    em_detector_init(d);
    memcpy(d->mean,           snap->mean,           sizeof(d->mean));
    memcpy(d->stdv,           snap->stdv,           sizeof(d->stdv));
    memcpy(d->original_means, snap->original_means, sizeof(d->original_means));
    d->has_original_baseline = snap->has_original_baseline != 0;
    d->fitted = true;
    /* std 하한 재보증 (구버전 스냅샷 방어) */
    for (int i = 0; i < EM_NUM_FEATURES; i++)
        if (d->stdv[i] < EM_STD_FLOOR) d->stdv[i] = EM_STD_FLOOR;
    return true;
}

void em_detector_update(em_detector_t *d,
                        const float *features,
                        const em_config_t *cfg,
                        em_verdict_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!d->fitted) return;   /* 학습 전에는 판정하지 않음 */

    int cw = cfg->confirm_window;
    if (cw > EM_MAX_CONFIRM_WIN) cw = EM_MAX_CONFIRM_WIN;
    if (cw < 1) cw = 1;
    int dw = cfg->drift_window;
    if (dw > EM_MAX_DRIFT_WIN) dw = EM_MAX_DRIFT_WIN;
    if (dw < 1) dw = 1;

    /* 1) z-score */
    int n_dev = 0;
    for (int i = 0; i < EM_NUM_FEATURES; i++) {
        out->z[i] = (features[i] - d->mean[i]) / d->stdv[i];
        if (fabsf(out->z[i]) > cfg->sigma_threshold) n_dev++;
    }
    out->n_deviated  = n_dev;
    out->instant_flag = (n_dev >= cfg->anomaly_min_features);

    /* 2) 확정 판정 — 원형 버퍼 (filled_count 로 초기 구간 보호) */
    d->flag_buf[d->flag_write_idx] = out->instant_flag ? 1 : 0;
    d->flag_write_idx = (d->flag_write_idx + 1) % cw;
    if (d->flag_filled < cw) d->flag_filled++;

    int hits = 0;
    for (int k = 0; k < d->flag_filled; k++) hits += d->flag_buf[k];
    /* 버퍼가 아직 confirm_window 만큼 차지 않은 초기 구간에는
     * 채워진 만큼만 집계 — 미초기화 슬롯을 읽지 않는다. */
    out->is_anomaly = (d->flag_filled >= cw) && (hits >= cfg->confirm_threshold);

    /* 3) 드리프트 감시 — 롤링 평균 vs original baseline
     * (구조 확정, drift_sigma/drift_window 는 실기기 데이터 확보 후 튜닝) */
    memcpy(d->drift_buf[d->drift_write_idx], features,
           sizeof(float) * EM_NUM_FEATURES);
    d->drift_write_idx = (d->drift_write_idx + 1) % dw;
    if (d->drift_filled < dw) d->drift_filled++;

    if (d->has_original_baseline && d->drift_filled >= dw) {
        for (int i = 0; i < EM_NUM_FEATURES; i++) {
            float acc = 0.0f;
            for (int k = 0; k < d->drift_filled; k++)
                acc += d->drift_buf[k][i];
            float roll = acc / (float)d->drift_filled;
            float dz = fabsf(roll - d->original_means[i]) / d->stdv[i];
            if (dz > cfg->drift_sigma) {
                out->drift_warning = true;
                break;
            }
        }
    }
}
