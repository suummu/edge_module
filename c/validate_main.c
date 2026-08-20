/*
  validate_main.c — 순수 C 구현(edgealimi.c)을 PC에서 파이썬 정답값과 대조 검증.

  ESP32 없이도 gcc만 있으면 이식 정합성을 확인할 수 있다:
    (repo 루트에서)
    gcc -std=c99 -O2 -Wall c/validate_main.c c/edgealimi.c -o validate -lm
    ./validate
    -> 종료 코드 0 = 전체 통과, 1 = 실패 항목 있음

  정답 파일(test_vectors.h)은 tools/export_test_vectors.py가 생성하며
  .ino의 VALIDATION_MODE와 동일한 3단계 검증을 수행한다:
    1. 특징 추출 (직접 구현한 FFT + 대역 에너지 + RMS + 회전 추정)
    2. baseline 평균/표준편차 계산
    3. 3σ 판별 불리언 (정확히 일치해야 함)
*/

#include <stdio.h>
#include <stdlib.h>

#include "edgealimi.h"
#include "../esp32/edgealimi_v2_realsensor/test_vectors.h"

/* 허용 오차: float32 vs float64 정밀도 + FFT 구현 차이 감안 (.ino와 동일 기준) */
static const float REL_TOLERANCE = 0.02f;
static const float ABS_FLOOR = 1e-4f;

static int pass_count = 0;
static int fail_count = 0;

static bool nearly_equal(float c_val, float py_val)
{
    float diff = fabsf(c_val - py_val);
    float scale = fabsf(py_val);
    if (scale < ABS_FLOOR) return diff < ABS_FLOOR;
    return (diff / scale) < REL_TOLERANCE;
}

static void check(bool ok, const char *what, float c_val, float py_val)
{
    if (ok) {
        pass_count++;
    } else {
        fail_count++;
        printf("  FAIL %s: C=%.6f / Python=%.6f\n", what, c_val, py_val);
    }
}

int main(void)
{
    float features[EA_N_FEATURES];
    float magnitudes[EA_HALF_SIZE];

    printf("===== edgealimi 순수 C 구현 - 파이썬 대조 검증 =====\n");
    printf("(허용 상대오차 %.0f%%, 판별 불리언은 정확 일치 요구)\n\n", REL_TOLERANCE * 100);

    /* --- 검증 1: 특징 추출 --- */
    printf("[검증 1] 특징 추출 (자체 구현 FFT + 대역 에너지 + RMS + 회전 추정)\n");
    for (uint16_t w = 0; w < TV_N_WINDOWS; w++) {
        float rot_io = EA_NO_ESTIMATE;
        ea_extract_features(TV_SIGNALS[w], features, &rot_io, true);

        char what[96];
        for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
            snprintf(what, sizeof(what), "[%s] %s", TV_LABELS[w], EA_FEATURE_NAMES[k]);
            check(nearly_equal(features[k], TV_EXPECTED_FEATURES[w][k]),
                  what, features[k], TV_EXPECTED_FEATURES[w][k]);
        }

        /* 회전 추정은 같은 FFT bin에 떨어져야 하므로 사실상 정확히 일치 */
        ea_compute_fft_magnitudes(TV_SIGNALS[w], magnitudes);
        float rot = ea_estimate_rotation_hz(magnitudes, EA_NO_ESTIMATE);
        if (isnan(rot)) rot = EA_FAN_ROTATION_HZ;
        snprintf(what, sizeof(what), "[%s] rotation_hz", TV_LABELS[w]);
        check(fabsf(rot - TV_EXPECTED_ROTATION_HZ[w]) < 0.01f,
              what, rot, TV_EXPECTED_ROTATION_HZ[w]);
    }

    /* --- 검증 2: baseline 통계 --- */
    printf("[검증 2] baseline 평균/표준편차\n");
    ea_detector_t det;
    ea_detector_init(&det);
    ea_detector_fit(&det, TV_BASELINE_FEATURES, TV_N_BASELINE);
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        char what[64];
        snprintf(what, sizeof(what), "%s mean", EA_FEATURE_NAMES[k]);
        check(nearly_equal(det.means[k], TV_EXPECTED_MEANS[k]),
              what, det.means[k], TV_EXPECTED_MEANS[k]);
        snprintf(what, sizeof(what), "%s std", EA_FEATURE_NAMES[k]);
        check(nearly_equal(det.stds[k], TV_EXPECTED_STDS[k]),
              what, det.stds[k], TV_EXPECTED_STDS[k]);
    }

    /* --- 검증 3: 3σ 판별 (파이썬 특징값을 그대로 넣어 판별 로직만 격리) --- */
    printf("[검증 3] 3-sigma 판별 (is_suspect)\n");
    for (uint8_t k = 0; k < EA_N_FEATURES; k++) {
        det.means[k] = TV_EXPECTED_MEANS[k];
        det.stds[k] = TV_EXPECTED_STDS[k];
    }
    for (uint16_t w = 0; w < TV_N_WINDOWS; w++) {
        ea_detector_reset_confirmation(&det);
        ea_judge_result_t r = ea_detector_judge(&det, TV_EXPECTED_FEATURES[w]);
        char what[96];
        snprintf(what, sizeof(what), "[%s] is_suspect", TV_LABELS[w]);
        check(r.is_suspect == TV_EXPECTED_SUSPECT[w],
              what, (float)r.is_suspect, (float)TV_EXPECTED_SUSPECT[w]);
    }

    /* --- 검증 4: 상태관리 (C 포팅 체크리스트 — 파이썬엔 대응 벡터가 없어 자체 검증) --- */
    printf("[검증 4] 상태관리 — 연속성 필터 미충족 상태 / 원본 baseline 보존\n");

    /* 4a. 부팅 직후 필터가 안 채워졌을 때는 확정 이상이 절대 뜨면 안 됨 */
    ea_detector_reset_confirmation(&det);
    bool premature_anomaly = false;
    for (uint8_t i = 0; i < EA_CONFIRM_WINDOW - 1; i++) {
        ea_judge_result_t r = ea_detector_judge(&det, TV_EXPECTED_FEATURES[5]); /* 심각한 고장 */
        if (r.is_anomaly) premature_anomaly = true;
    }
    check(!premature_anomaly, "필터 미충족 시 확정 억제", premature_anomaly, 0.0f);

    /* 5번째 윈도우부터는 확정이 떠야 함 (5개 전부 의심 = 비율 1.0 >= 0.6) */
    ea_judge_result_t r5 = ea_detector_judge(&det, TV_EXPECTED_FEATURES[5]);
    check(r5.is_anomaly, "필터 충족 시 확정 격상", (float)r5.is_anomaly, 1.0f);

    /* 4b. 재보정(fit 재호출)해도 원본 baseline은 안 바뀌어야 함 */
    float orig_before = det.original_means[EA_FEAT_RMS];
    /* 두 번째 fit: 일부러 다른 데이터(2배 스케일)로 재보정 */
    static float shifted[60][EA_N_FEATURES];
    for (uint16_t w = 0; w < TV_N_BASELINE; w++)
        for (uint8_t k = 0; k < EA_N_FEATURES; k++)
            shifted[w][k] = TV_BASELINE_FEATURES[w][k] * 2.0f;
    ea_detector_fit(&det, shifted, TV_N_BASELINE);
    check(det.original_means[EA_FEAT_RMS] == orig_before,
          "재보정 후 원본 baseline 보존", det.original_means[EA_FEAT_RMS], orig_before);
    check(nearly_equal(det.means[EA_FEAT_RMS], TV_EXPECTED_MEANS[EA_FEAT_RMS] * 2.0f),
          "재보정 후 현재 baseline 갱신", det.means[EA_FEAT_RMS],
          TV_EXPECTED_MEANS[EA_FEAT_RMS] * 2.0f);

    /* --- 결과 --- */
    printf("\n===== 결과: PASS %d / FAIL %d =====\n", pass_count, fail_count);
    if (fail_count == 0) {
        printf("전체 통과 - 순수 C 구현이 파이썬과 동일하게 동작함.\n");
        return 0;
    }
    printf("실패 항목 있음 - FFT 정규화(2/N), 대역 경계 부등호, DC bin, n vs n-1 분산부터 의심할 것.\n");
    return 1;
}
