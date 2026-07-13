"""
export_c.py

목적:
    학습된 결정트리(고장 분류)와 3-sigma baseline(이상탐지)을
    ESP32에서 그대로 쓸 수 있는 C 헤더 파일로 내보낸다.

    - 결정트리: if-else 중첩 코드로 변환. 라이브러리 의존성 0, 부동소수점
      비교 몇 번이면 끝이라 ESP32 계산 부하도 무시 가능한 수준.
      (원시 특징으로 학습했으므로 스케일러 상수도 필요 없음)
    - 3-sigma baseline: 특징별 (mean, std) 상수 배열. 판정 로직은 이미
      c/edge_pipeline.c에 이식돼 있는 것과 같은 구조이므로 상수만 공급.

    입력 특징 순서는 features_real.FEATURE_NAMES와 동일해야 한다 (헤더에 명시).

사용법:
    python -m ml.export_c cwru
    python -m ml.export_c mafaulda
"""

import sys
from pathlib import Path

import joblib

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ml.features_real import FEATURE_NAMES  # noqa: E402
from ml.train_models import MODEL_DIR  # noqa: E402

C_DIR = Path(__file__).resolve().parent.parent / "c"


def tree_to_c(tree, class_names, indent="    "):
    """sklearn 결정트리 -> 중첩 if-else C 코드 문자열."""
    t = tree.tree_

    def recurse(node, depth):
        pad = indent * (depth + 1)
        if t.feature[node] < 0:  # leaf
            cls = class_names[t.value[node][0].argmax()]
            return f"{pad}return LABEL_{cls.upper()};\n"
        name = FEATURE_NAMES[t.feature[node]]
        thr = t.threshold[node]
        code = f"{pad}if (features[FEAT_{name.upper()}] <= {thr:.10g}f) {{\n"
        code += recurse(t.children_left[node], depth + 1)
        code += f"{pad}}} else {{\n"
        code += recurse(t.children_right[node], depth + 1)
        code += f"{pad}}}\n"
        return code

    return recurse(0, 0)


def export(dataset):
    tree = joblib.load(MODEL_DIR / f"{dataset}_decision_tree_d4.joblib")
    detector = joblib.load(MODEL_DIR / f"{dataset}_baseline_detector.joblib")
    class_names = list(tree.classes_)

    feat_enum = "\n".join(f"#define FEAT_{n.upper()} {i}" for i, n in enumerate(FEATURE_NAMES))
    label_enum = "\n".join(f"#define LABEL_{c.upper()} {i}" for i, c in enumerate(class_names))
    means = ", ".join(f"{detector.means[f]:.10g}f" for f in detector.features_to_check)
    stds = ", ".join(f"{detector.stds[f]:.10g}f" for f in detector.features_to_check)
    check_list = ", ".join(detector.features_to_check)

    header = f"""/*
 * ml_model_{dataset}.h  (자동 생성 - ml/export_c.py, 직접 수정 금지)
 *
 * {dataset.upper()} 실데이터로 학습된 모델 2종:
 *   1) predict_fault_type(): depth<=4 결정트리 고장 분류기 (if-else 변환)
 *   2) BASELINE_MEAN/STD: 3-sigma 이상탐지 baseline 상수
 *
 * 입력 features[] 순서 (features_real.FEATURE_NAMES와 동일):
 *   {", ".join(FEATURE_NAMES)}
 * 주의: ratio 특징은 (대역 에너지 / DC 제외 전체 스펙트럼 에너지 합)이다.
 */
#ifndef ML_MODEL_{dataset.upper()}_H
#define ML_MODEL_{dataset.upper()}_H

#define ML_N_FEATURES {len(FEATURE_NAMES)}

{feat_enum}

{label_enum}

/* 3-sigma baseline (판별 특징 순서: {check_list}) */
static const float BASELINE_MEAN[ML_N_FEATURES] = {{ {means} }};
static const float BASELINE_STD[ML_N_FEATURES]  = {{ {stds} }};

/* 결정트리 고장 분류기 (라벨 반환) */
static inline int predict_fault_type(const float features[ML_N_FEATURES]) {{
{tree_to_c(tree, class_names)}}}

#endif /* ML_MODEL_{dataset.upper()}_H */
"""
    C_DIR.mkdir(exist_ok=True)
    out = C_DIR / f"ml_model_{dataset}.h"
    out.write_text(header, encoding="utf-8")
    print(f"내보내기 완료: {out}")
    return out


if __name__ == "__main__":
    for dataset in sys.argv[1:] or ["cwru", "mafaulda"]:
        try:
            export(dataset)
        except FileNotFoundError as e:
            print(f"{dataset}: 모델 파일 없음 - 먼저 train_models를 실행 ({e})")
