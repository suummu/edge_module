/*
 * ml_model_mafaulda.h  (자동 생성 - ml/export_c.py, 직접 수정 금지)
 *
 * MAFAULDA 실데이터로 학습된 모델 2종:
 *   1) predict_fault_type(): depth<=4 결정트리 고장 분류기 (if-else 변환)
 *   2) BASELINE_MEAN/STD: 3-sigma 이상탐지 baseline 상수
 *
 * 입력 features[] 순서 (features_real.FEATURE_NAMES와 동일):
 *   rms, kurtosis, harmonic1_ratio, harmonic2_ratio, harmonic3_ratio, high_freq_ratio
 * 주의: ratio 특징은 (대역 에너지 / DC 제외 전체 스펙트럼 에너지 합)이다.
 */
#ifndef ML_MODEL_MAFAULDA_H
#define ML_MODEL_MAFAULDA_H

#define ML_N_FEATURES 6

#define FEAT_RMS 0
#define FEAT_KURTOSIS 1
#define FEAT_HARMONIC1_RATIO 2
#define FEAT_HARMONIC2_RATIO 3
#define FEAT_HARMONIC3_RATIO 4
#define FEAT_HIGH_FREQ_RATIO 5

#define LABEL_IMBALANCE 0
#define LABEL_MISALIGNMENT 1
#define LABEL_NORMAL 2

/* 3-sigma baseline (판별 특징 순서: rms, kurtosis, harmonic1_ratio, harmonic2_ratio, harmonic3_ratio, high_freq_ratio) */
static const float BASELINE_MEAN[ML_N_FEATURES] = { 0.412532612f, -0.1246177421f, 0.01228413882f, 0.003986581928f, 0.004396381908f, 0.6670915726f };
static const float BASELINE_STD[ML_N_FEATURES]  = { 0.08835325647f, 0.4709081334f, 0.006211194815f, 0.001327655319f, 0.0007896469332f, 0.03574993902f };

/* 결정트리 고장 분류기 (라벨 반환) */
static inline int predict_fault_type(const float features[ML_N_FEATURES]) {
    if (features[FEAT_HARMONIC1_RATIO] <= 0.0234843269f) {
        if (features[FEAT_HIGH_FREQ_RATIO] <= 0.6963450313f) {
            if (features[FEAT_RMS] <= 0.3044121861f) {
                return LABEL_MISALIGNMENT;
            } else {
                if (features[FEAT_KURTOSIS] <= -0.7735003531f) {
                    return LABEL_IMBALANCE;
                } else {
                    return LABEL_NORMAL;
                }
            }
        } else {
            if (features[FEAT_HARMONIC3_RATIO] <= 0.004117492819f) {
                if (features[FEAT_HARMONIC1_RATIO] <= 0.01153592765f) {
                    return LABEL_MISALIGNMENT;
                } else {
                    return LABEL_NORMAL;
                }
            } else {
                if (features[FEAT_HIGH_FREQ_RATIO] <= 0.7076977789f) {
                    return LABEL_MISALIGNMENT;
                } else {
                    return LABEL_MISALIGNMENT;
                }
            }
        }
    } else {
        if (features[FEAT_HIGH_FREQ_RATIO] <= 0.584550947f) {
            return LABEL_MISALIGNMENT;
        } else {
            if (features[FEAT_HIGH_FREQ_RATIO] <= 0.5944113433f) {
                if (features[FEAT_RMS] <= 0.3561909199f) {
                    return LABEL_IMBALANCE;
                } else {
                    return LABEL_MISALIGNMENT;
                }
            } else {
                return LABEL_IMBALANCE;
            }
        }
    }
}

#endif /* ML_MODEL_MAFAULDA_H */
