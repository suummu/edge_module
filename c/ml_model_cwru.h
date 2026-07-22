/*
 * ml_model_cwru.h  (자동 생성 - ml/export_c.py, 직접 수정 금지)
 *
 * CWRU 실데이터로 학습된 모델 2종:
 *   1) predict_fault_type(): depth<=4 결정트리 고장 분류기 (if-else 변환)
 *   2) BASELINE_MEAN/STD: 3-sigma 이상탐지 baseline 상수
 *
 * 입력 features[] 순서 (features_real.FEATURE_NAMES와 동일):
 *   rms, kurtosis, harmonic1_ratio, harmonic2_ratio, harmonic3_ratio, high_freq_ratio
 * 주의: ratio 특징은 (대역 에너지 / DC 제외 전체 스펙트럼 에너지 합)이다.
 */
#ifndef ML_MODEL_CWRU_H
#define ML_MODEL_CWRU_H

#define ML_N_FEATURES 6

#define FEAT_RMS 0
#define FEAT_KURTOSIS 1
#define FEAT_HARMONIC1_RATIO 2
#define FEAT_HARMONIC2_RATIO 3
#define FEAT_HARMONIC3_RATIO 4
#define FEAT_HIGH_FREQ_RATIO 5

#define LABEL_BALL 0
#define LABEL_INNER_RACE 1
#define LABEL_NORMAL 2
#define LABEL_OUTER_RACE 3

/* 3-sigma baseline (판별 특징 순서: rms, kurtosis, harmonic1_ratio, harmonic2_ratio, harmonic3_ratio, high_freq_ratio) */
static const float BASELINE_MEAN[ML_N_FEATURES] = { 0.06594883189f, -0.1025816953f, 0.00904005448f, 0.008290359673f, 0.03281413622f, 0.7275342167f };
static const float BASELINE_STD[ML_N_FEATURES]  = { 0.00287407631f, 0.1278966304f, 0.003957410607f, 0.002315659746f, 0.009848015637f, 0.02091407009f };

/* 결정트리 고장 분류기 (라벨 반환) */
static inline int predict_fault_type(const float features[ML_N_FEATURES]) {
    if (features[FEAT_HARMONIC3_RATIO] <= 0.005641427939f) {
        if (features[FEAT_RMS] <= 0.4757173657f) {
            if (features[FEAT_HIGH_FREQ_RATIO] <= 0.8993645906f) {
                return LABEL_OUTER_RACE;
            } else {
                if (features[FEAT_RMS] <= 0.1591754705f) {
                    return LABEL_BALL;
                } else {
                    return LABEL_INNER_RACE;
                }
            }
        } else {
            return LABEL_OUTER_RACE;
        }
    } else {
        return LABEL_NORMAL;
    }
}

#endif /* ML_MODEL_CWRU_H */
