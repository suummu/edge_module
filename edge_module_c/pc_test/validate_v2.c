/*
 * validate_v2.c — v2 이식 무결성 검증 (파이썬 float64 정답 대 C float32 실측)
 *
 * 빌드: gcc -O2 -I../core validate_v2.c ../core/em_*.c -lm -o validate_v2
 * 판정: 상대오차를 float32 이론 한계(eps*sqrt(N) 스케일)와 함께 보고하고,
 *       max rel err 가 1e-3 이내면 PASS (창함수·비율 나눗셈 누적 감안).
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "em_config.h"
#include "em_fft.h"
#include "em_rotation.h"
#include "em_features.h"
#include "em_detector.h"
#include "v2_vectors.h"

static double rel_err(double c, double py)
{
    double scale = fabs(py);
    if (scale < 1e-9) return fabs(c - py) < 1e-9 ? 0.0 : 1.0;
    return fabs(c - py) / scale;
}

int main(void)
{
    em_config_t cfg;
    em_config_default(&cfg, V2_RATED_HZ * 60.0f);

    static float sig[V2_WINDOW];
    static float mag[EM_NUM_BINS];
    float feats[EM_NUM_FEATURES];

    double worst = 0.0;
    const char *worst_at = "";
    int fail = 0;

    printf("=== v2 이식 대조 검증 (C float32 vs Python float64) ===\n");
    for (int c = 0; c < V2_N_CASES; c++) {
        memcpy(sig, V2_SIGNALS[c], sizeof(sig));
        float rms = em_fft_spectrum(sig, mag);
        float snr = 0.0f;
        float rot = em_rotation_measure(mag, &cfg, &snr);
        em_extract_features(mag, rot, rms, &cfg, feats);

        double e_rot = rel_err(rot, V2_EXPECT_ROT[c]);
        if (e_rot > worst) { worst = e_rot; worst_at = "rotation"; }
        printf("[%-14s] rot C=%.4f / Py=%.4f (rel %.2e)\n",
               V2_LABELS[c], rot, V2_EXPECT_ROT[c], e_rot);

        for (int i = 0; i < EM_NUM_FEATURES; i++) {
            double e = rel_err(feats[i], V2_EXPECT_FEATURES[c][i]);
            if (e > worst) { worst = e; worst_at = EM_FEATURE_NAMES[i]; }
            if (e > 1e-3) {
                fail++;
                printf("  FAIL %s: C=%.8g Py=%.8g rel=%.2e\n",
                       EM_FEATURE_NAMES[i], feats[i],
                       V2_EXPECT_FEATURES[c][i], e);
            }
            /* log 공간 값도 대조 (판정이 실제로 쓰는 값) */
            double zc = log((feats[i] < 0 ? 0 : feats[i]) + 1e-6);
            double ez = fabs(zc - V2_EXPECT_Z[c][i]);
            if (ez > 1e-3) {
                fail++;
                printf("  FAIL z(%s): C=%.6f Py=%.6f diff=%.2e\n",
                       EM_FEATURE_NAMES[i], zc, V2_EXPECT_Z[c][i], ez);
            }
        }
    }

    double f32_limit = 1.19e-7 * sqrt((double)V2_WINDOW);
    printf("\nmax rel err  : %.3e  (%s)\n", worst, worst_at);
    printf("float32 limit: %.3e  (eps*sqrt(N), N=%d)\n", f32_limit, V2_WINDOW);
    printf("tolerance    : 1.0e-03\n");
    printf("%s\n", fail == 0 ? "전체 PASS — 파이썬 v2와 동일 동작"
                             : "FAIL 있음 — 이식 재점검 필요");
    return fail == 0 ? 0 : 1;
}
