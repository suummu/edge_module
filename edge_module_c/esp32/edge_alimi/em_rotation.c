#include "em_rotation.h"
#include "em_fft.h"
#include <math.h>

void em_rotation_init(em_rotation_t *rt)
{
    rt->hz = 0.0f;
    rt->valid = false;
    rt->pending_hz = 0.0f;
    rt->rejected = 0;
}

float em_rotation_measure(const float *mag,
                          const em_config_t *cfg,
                          float *snr_out)
{
    const float bin_hz = em_bin_hz(cfg);
    const float nominal = em_rated_hz(cfg);

    int lo = (int)floorf(nominal * cfg->rot_search_lo_ratio / bin_hz);
    int hi = (int)ceilf (nominal * cfg->rot_search_hi_ratio / bin_hz);
    if (lo < 1) lo = 1;                       /* DC 제외 */
    if (hi > EM_NUM_BINS - 2) hi = EM_NUM_BINS - 2;
    if (hi <= lo + 4) {                       /* 설정 오류 방어 */
        if (snr_out) *snr_out = 0.0f;
        return nominal;
    }

    /* 탐색범위 내 최대 피크 */
    int pk = lo;
    for (int k = lo; k <= hi; k++)
        if (mag[k] > mag[pk]) pk = k;

    /* SNR: 피크 / (피크 ±2 bin 제외한 대역 평균) */
    float acc = 0.0f; int cnt = 0;
    for (int k = lo; k <= hi; k++) {
        if (k >= pk - 2 && k <= pk + 2) continue;
        acc += mag[k]; cnt++;
    }
    float floor_avg = (cnt > 0) ? acc / (float)cnt : 0.0f;
    if (snr_out)
        *snr_out = (floor_avg > 1e-20f) ? mag[pk] / floor_avg : 0.0f;

    /* 포물선 보간으로 sub-bin 정밀도 확보 (bin 해상도 ~1Hz 보완) */
    float y0 = mag[pk - 1], y1 = mag[pk], y2 = mag[pk + 1];
    float denom = (y0 - 2.0f * y1 + y2);
    float delta = 0.0f;
    if (fabsf(denom) > 1e-12f) {
        delta = 0.5f * (y0 - y2) / denom;
        if (delta > 0.5f) delta = 0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }
    return ((float)pk + delta) * bin_hz;
}

float em_rotation_accept(em_rotation_t *rt,
                         float candidate_hz,
                         const em_config_t *cfg)
{
    if (!rt->valid) {                    /* 최초 추정 — 무조건 채택 */
        rt->hz = candidate_hz;
        rt->valid = true;
        return rt->hz;
    }

    if (fabsf(candidate_hz - rt->hz) <= cfg->rot_max_jump_hz) {
        rt->hz = candidate_hz;
        rt->rejected = 0;                /* 정상 추종 — 누적 거부 해제 */
        rt->pending_hz = 0.0f;
        return rt->hz;
    }

    /* 급격한 점프 → 일단 거부하되, 같은 자리에 연속으로 나타나는지 본다.
     * 노이즈는 매번 다른 곳에 튀고, 실제 속도 변화는 한 자리에 머문다. */
    if (rt->rejected > 0 &&
        fabsf(candidate_hz - rt->pending_hz) <= cfg->rot_max_jump_hz)
        rt->rejected++;
    else
        rt->rejected = 1;
    rt->pending_hz = candidate_hz;

    if (rt->rejected >= cfg->rot_relock_windows) {
        rt->hz = candidate_hz;           /* 지속된 변화 — 새 운전점 수용 */
        rt->rejected = 0;
        rt->pending_hz = 0.0f;
    }
    return rt->hz;
}
