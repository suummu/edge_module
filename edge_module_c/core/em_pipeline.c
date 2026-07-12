#include "em_pipeline.h"
#include "em_fft.h"
#include "em_features.h"
#include <stdio.h>
#include <string.h>

void em_pipeline_init(em_pipeline_t *p, float rated_rpm)
{
    memset(p, 0, sizeof(*p));
    em_config_default(&p->cfg, rated_rpm);
    em_rotation_init(&p->rot);
    em_detector_init(&p->det);
    p->mode = EM_MODE_IDLE;
    p->running = false;
}

void em_pipeline_start_learning(em_pipeline_t *p, int n_windows)
{
    if (n_windows < 2) n_windows = 2;
    p->learn_target = n_windows;
    p->learn_count = 0;
    p->warmup_left = p->cfg.warmup_windows;
    em_detector_fit_begin(&p->det);
    p->mode = EM_MODE_LEARNING;
}

/* 가동/정지 상태 머신. 반환: 상태가 정지→가동으로 전환됐는지 */
static bool update_run_state(em_pipeline_t *p, float snr)
{
    bool started = false;
    if (snr >= p->cfg.run_snr_threshold) {
        p->run_streak++;
        p->stop_streak = 0;
        if (!p->running && p->run_streak >= p->cfg.run_hysteresis) {
            p->running = true;
            started = true;
        }
    } else {
        p->stop_streak++;
        p->run_streak = 0;
        if (p->running && p->stop_streak >= p->cfg.run_hysteresis)
            p->running = false;
    }
    return started;
}

void em_pipeline_process(em_pipeline_t *p, float *signal, em_output_t *out)
{
    static float mag[EM_NUM_BINS];   /* 정적 — ESP32 스택 보호 */

    memset(out, 0, sizeof(*out));
    p->window_count++;
    out->seq = p->window_count;

    float rms = em_fft_spectrum(signal, mag);

    /* 1) 가동 판별 — 정지 중엔 회전 추정 갱신 금지 (노이즈 피크 추종 방지) */
    float snr = 0.0f;
    float cand = em_rotation_measure(mag, &p->cfg, &snr);
    out->snr = snr;
    bool just_started = update_run_state(p, snr);
    out->running = p->running;

    float rot = p->rot.hz;
    if (p->running)
        rot = em_rotation_accept(&p->rot, cand, &p->cfg);
    out->rotation_hz = rot;

    /* 재가동 시 낡은 판정 플래그 리셋 */
    if (just_started)
        em_detector_clear_runtime(&p->det);

    em_extract_features(mag, rot, rms, &p->cfg, out->features);

    /* 2) 정지 중에는 학습도 판정도 하지 않는다 */
    if (!p->running) {
        out->mode = p->mode;
        out->learn_progress = (p->mode == EM_MODE_LEARNING && p->learn_target)
            ? (int)(100.0f * (float)p->learn_count / (float)p->learn_target)
            : (p->mode == EM_MODE_MONITORING ? 100 : 0);
        return;
    }

    if (p->mode == EM_MODE_LEARNING) {
        if (p->warmup_left > 0) {
            p->warmup_left--;   /* 기동 과도 구간 폐기 — baseline 오염 방지 */
        } else {
            em_detector_fit_add(&p->det, out->features);
            p->learn_count++;
            if (p->learn_count >= p->learn_target) {
                if (em_detector_fit_end(&p->det, &p->cfg))
                    p->mode = EM_MODE_MONITORING;
                else
                    p->mode = EM_MODE_IDLE;   /* 표본 부족 — 재시도 필요 */
            }
        }
        out->learn_progress = (int)(100.0f * (float)p->learn_count
                                    / (float)p->learn_target);
    } else if (p->mode == EM_MODE_MONITORING) {
        em_detector_update(&p->det, out->features, &p->cfg, &out->verdict);
        out->learn_progress = 100;
    }
    out->mode = p->mode;
}

void em_pipeline_save_baseline(const em_pipeline_t *p, em_snapshot_t *snap)
{
    em_detector_save(&p->det, snap);
}

bool em_pipeline_load_baseline(em_pipeline_t *p, const em_snapshot_t *snap)
{
    if (!em_detector_load(&p->det, snap)) return false;
    p->mode = EM_MODE_MONITORING;
    return true;
}

int em_output_to_json(const em_output_t *o, char *buf, int buflen)
{
    /* 특징명은 EM_FEATURE_NAMES 를 그대로 사용 —
     * 시각화 쪽 키 이름과의 불일치(파이썬에서 겪은 부분 전파 버그) 원천 차단 */
    int n = snprintf(buf, (size_t)buflen,
        "{\"seq\":%lu,\"mode\":%d,\"learn\":%d,\"run\":%d,"
        "\"rot_hz\":%.3f,\"snr\":%.1f,"
        "\"f\":{\"%s\":%.6g,\"%s\":%.6g,\"%s\":%.6g,\"%s\":%.6g,\"%s\":%.6g},"
        "\"z\":[%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"n_dev\":%d,\"flag\":%d,\"anomaly\":%d,\"drift\":%d}",
        (unsigned long)o->seq, (int)o->mode, o->learn_progress,
        o->running ? 1 : 0,
        (double)o->rotation_hz, (double)o->snr,
        EM_FEATURE_NAMES[0], (double)o->features[0],
        EM_FEATURE_NAMES[1], (double)o->features[1],
        EM_FEATURE_NAMES[2], (double)o->features[2],
        EM_FEATURE_NAMES[3], (double)o->features[3],
        EM_FEATURE_NAMES[4], (double)o->features[4],
        (double)o->verdict.z[0], (double)o->verdict.z[1],
        (double)o->verdict.z[2], (double)o->verdict.z[3],
        (double)o->verdict.z[4],
        o->verdict.n_deviated,
        o->verdict.instant_flag ? 1 : 0,
        o->verdict.is_anomaly ? 1 : 0,
        o->verdict.drift_warning ? 1 : 0);
    return n;
}
