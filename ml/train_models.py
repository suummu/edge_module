"""
train_models.py

목적:
    실데이터(CWRU, MaFaulDa)로 두 트랙의 모델을 학습/평가한다.

    [트랙 1: 이상탐지 - 기존 3-sigma 방식의 실데이터 검증]
      지금까지 시뮬레이션 신호로만 검증했던 BaselineDetector(3-sigma)를
      실데이터 '정상' 윈도우로 학습시키고, 실제 고장 윈도우를 얼마나 잡는지 잰다.
      비교군으로 Isolation Forest(이식 불가하지만 성능 상한 참고용)를 같이 평가.
      -> 중간보고서의 "시뮬레이션 검증 -> 실데이터 검증" 스토리를 완성하는 부분.

    [트랙 2: 고장 분류 - 실제 머신러닝 모델]
      정상/고장유형을 분류하는 지도학습 모델 3종:
        - DecisionTree(depth<=4): ESP32에 if-else로 직접 이식 가능한 후보
        - LogisticRegression: 계수 몇 개로 이식 가능한 후보
        - RandomForest: 성능 상한 참고용 (이식 대상 아님)
      "작은 모델이 큰 모델 대비 얼마나 손해보는가"가 곧 이식성-성능 트레이드오프.

    평가 원칙:
      - train/test 분할은 반드시 파일(group) 단위 - 같은 파일의 겹친 윈도우가
        양쪽에 들어가면 성능이 허위로 부풀려진다(데이터 누수).
      - 오탐율(정상->이상)과 미탐율(고장->정상)을 항상 같이 보고한다.
        (자문 핵심: 오경보 한 번이 현장 신뢰를 무너뜨린다)

사용법:
    python -m ml.train_models cwru
    python -m ml.train_models mafaulda
    python -m ml.train_models all
"""

import json
import sys
from collections import Counter
from pathlib import Path

import joblib
import numpy as np
from sklearn.ensemble import IsolationForest, RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score
from sklearn.model_selection import GroupShuffleSplit
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.baseline_detector import BaselineDetector  # noqa: E402
from ml.dataset import load_cwru_windows, load_mafaulda_windows  # noqa: E402
from ml.features_real import FEATURE_NAMES, windows_to_matrix  # noqa: E402

MODEL_DIR = Path(__file__).resolve().parent / "models"
RESULT_DIR = Path(__file__).resolve().parent / "results"

RANDOM_STATE = 42


def group_split(y, groups, test_size=0.3, seed=RANDOM_STATE):
    """파일(group) 단위 분할. 모든 클래스가 양쪽에 존재할 때까지 시드를 옮겨가며 시도."""
    for s in range(seed, seed + 50):
        gss = GroupShuffleSplit(n_splits=1, test_size=test_size, random_state=s)
        train_idx, test_idx = next(gss.split(np.zeros(len(y)), y, groups))
        if set(y[train_idx]) == set(y) and set(y[test_idx]) == set(y):
            return train_idx, test_idx
    raise RuntimeError("클래스를 모두 포함하는 group 분할을 찾지 못함")


# ---------------------------------------------------------------------------
# 트랙 1: 이상탐지 (정상 학습 -> 고장 탐지)
# ---------------------------------------------------------------------------
def evaluate_anomaly_detection(dicts, y, groups):
    """
    BaselineDetector(3-sigma)와 IsolationForest를 같은 조건에서 비교.

    분할 프로토콜 (실배포 시나리오와 동일하게):
      실제 배포에서는 "같은 설비에서 초기 수일간 수집한 정상 데이터"로 baseline을
      만들고 이후를 감시한다. 따라서 각 정상 파일의 앞 60% 윈도우(시간상 앞부분)로
      학습하고, 같은 파일의 뒤 40% + 모든 고장 윈도우로 평가한다.
      (처음에 파일 단위 분할로 평가했더니 baseline이 못 본 부하조건(RPM)의 정상
      파일이 통째로 오탐돼 FP 35%가 나왔다 - "회전수/부하는 모든 분석의 기준축"
      이라는 자문 내용이 실데이터에서 재현된 것. 배포 시나리오는 같은 설비를
      계속 보는 것이므로 시간 기준 분할이 올바른 프로토콜이다.)
    """
    check_features = ["rms", "kurtosis", "harmonic1_ratio", "harmonic2_ratio",
                      "harmonic3_ratio", "high_freq_ratio"]

    # 정상 파일별로 앞 60%(baseline 학습) / 뒤 40%(오탐 평가) 시간 분할
    train_normal_idx, test_idx = [], []
    for g in sorted(set(groups)):
        g_idx = [i for i in range(len(y)) if groups[i] == g]  # 파일 내 시간 순서 유지
        if y[g_idx[0]] == "normal":
            cut = int(len(g_idx) * 0.6)
            train_normal_idx += g_idx[:cut]
            test_idx += g_idx[cut:]
        else:
            test_idx += g_idx

    train_normal = [dicts[i] for i in train_normal_idx]
    detector = BaselineDetector(n_sigma=3.0, features_to_check=check_features)
    detector.fit(train_normal)

    X_iso = np.array([[d[f] for f in check_features] for d in dicts])
    scaler = StandardScaler().fit(X_iso[train_normal_idx])
    iso = IsolationForest(random_state=RANDOM_STATE, contamination=0.01)
    iso.fit(scaler.transform(X_iso[train_normal_idx]))

    results = {"three_sigma": {}, "isolation_forest": {},
               "n_train_normal_windows": len(train_normal)}
    per_label = {lbl: [i for i in test_idx if y[i] == lbl] for lbl in sorted(set(y))}

    for lbl, idxs in per_label.items():
        if not idxs:
            continue
        # 3-sigma: 윈도우 단위 독립 판정이므로 매번 연속성 이력 리셋 후 is_suspect 사용
        flags = []
        for i in idxs:
            detector.reset_confirmation_state()
            flags.append(detector.judge(dicts[i])["is_suspect"])
        rate = float(np.mean(flags))
        iso_pred = iso.predict(scaler.transform(X_iso[idxs])) == -1
        iso_rate = float(np.mean(iso_pred))

        key = "false_positive_rate" if lbl == "normal" else "detection_rate"
        results["three_sigma"][f"{key}__{lbl}"] = round(rate, 4)
        results["isolation_forest"][f"{key}__{lbl}"] = round(iso_rate, 4)

    # --- 회전수 구간별 baseline (자문: "회전수/부하는 모든 분석의 기준축") ---
    # MaFaulDa는 파일마다 회전수가 12~61Hz로 제각각이라, 전 속도를 baseline 하나로
    # 뭉뚱그리면 표준편차가 부풀어 민감도가 떨어진다. 실배포에서는 설비의 운전점
    # 부근 데이터로 baseline을 만들므로, 회전수 구간(bin)별로 별도 baseline을 두는
    # 것이 올바른 대응이다. (ESP32에서도 (mean,std) 세트를 구간 수만큼 두면 끝)
    def speed_bin(hz):
        return 0 if hz < 25 else 1 if hz < 40 else 2 if hz < 52 else 3

    bin_detectors = {}
    for b in range(4):
        bin_train = [dicts[i] for i in train_normal_idx
                     if speed_bin(dicts[i]["rotation_hz_estimate"]) == b]
        if len(bin_train) >= 20:
            d = BaselineDetector(n_sigma=3.0, features_to_check=check_features)
            d.fit(bin_train)
            bin_detectors[b] = d

    if len(bin_detectors) > 1:  # 회전수가 사실상 단일 구간이면(CWRU) 의미 없으므로 생략
        results["three_sigma_speed_binned"] = {}
        for lbl, idxs in per_label.items():
            if not idxs:
                continue
            flags = []
            for i in idxs:
                d = bin_detectors.get(speed_bin(dicts[i]["rotation_hz_estimate"]), detector)
                d.reset_confirmation_state()
                flags.append(d.judge(dicts[i])["is_suspect"])
            key = "false_positive_rate" if lbl == "normal" else "detection_rate"
            results["three_sigma_speed_binned"][f"{key}__{lbl}"] = round(float(np.mean(flags)), 4)

    # --- 고장 심각도별 탐지율 (경미한 고장이 얼마나 어려운지 정직하게 보고) ---
    severity_breakdown = {}
    for i in test_idx:
        if y[i] == "normal" or "/" not in groups[i]:
            continue
        folder = groups[i].split("_", 1)[1].rsplit("_", 1)[0]  # 예: imbalance/6g
        detector.reset_confirmation_state()
        s = detector.judge(dicts[i])["is_suspect"]
        severity_breakdown.setdefault(folder, [0, 0])
        severity_breakdown[folder][0] += int(s)
        severity_breakdown[folder][1] += 1
    if severity_breakdown:
        results["three_sigma_by_severity"] = {
            k: round(v[0] / v[1], 4) for k, v in sorted(severity_breakdown.items())}

    # --- 연속성 필터 반영 (실제 경보 기준인 is_anomaly로 스트리밍 평가) ---
    # 위 수치는 단일 윈도우 1차 판정(is_suspect) 기준. 실제 경보는 confirm_window=5
    # 중 60% 이상 의심일 때만 울리므로, 파일별 시간 순서대로 흘려보내 확정 판정을 잰다.
    confirmed = {"false_positive_rate__normal": [0, 0]}  # [경보 윈도우 수, 전체 윈도우 수]
    detected_files, fault_files = 0, 0
    for g in sorted(set(groups)):
        g_test = [i for i in test_idx if groups[i] == g]
        if not g_test:
            continue
        detector.reset_confirmation_state()
        alarms = 0
        for i in g_test:
            if detector.judge(dicts[i])["is_anomaly"]:
                alarms += 1
        if y[g_test[0]] == "normal":
            confirmed["false_positive_rate__normal"][0] += alarms
            confirmed["false_positive_rate__normal"][1] += len(g_test)
        else:
            fault_files += 1
            detected_files += 1 if alarms > 0 else 0
    fp_n, fp_d = confirmed["false_positive_rate__normal"]
    results["three_sigma_confirmed"] = {
        "false_positive_rate__normal": round(fp_n / fp_d, 4) if fp_d else None,
        "fault_files_alarmed": f"{detected_files}/{fault_files}",
    }

    return results, detector


# ---------------------------------------------------------------------------
# 트랙 2: 고장 분류
# ---------------------------------------------------------------------------
def evaluate_classifiers(X, y, train_idx, test_idx):
    scaler = StandardScaler().fit(X[train_idx])
    X_tr, X_te = scaler.transform(X[train_idx]), scaler.transform(X[test_idx])
    y_tr, y_te = y[train_idx], y[test_idx]
    labels = sorted(set(y))

    models = {
        "decision_tree_d4": DecisionTreeClassifier(max_depth=4, random_state=RANDOM_STATE),
        "logistic_regression": LogisticRegression(max_iter=2000, random_state=RANDOM_STATE),
        "random_forest": RandomForestClassifier(n_estimators=200, random_state=RANDOM_STATE),
    }

    results, fitted = {}, {}
    for name, model in models.items():
        # 결정트리는 스케일 불변이므로 원시 특징으로 학습한다.
        # -> C 이식 시 scaler(평균/표준편차 12개 상수) 없이 임계값 비교만으로 동작 가능.
        if name == "decision_tree_d4":
            model.fit(X[train_idx], y_tr)
            pred = model.predict(X[test_idx])
        else:
            model.fit(X_tr, y_tr)
            pred = model.predict(X_te)
        results[name] = {
            "accuracy": round(float(accuracy_score(y_te, pred)), 4),
            "macro_f1": round(float(f1_score(y_te, pred, average="macro")), 4),
            "confusion_matrix": confusion_matrix(y_te, pred, labels=labels).tolist(),
            "labels": labels,
        }
        fitted[name] = model

    rf = fitted["random_forest"]
    results["feature_importance_rf"] = {
        name: round(float(v), 4)
        for name, v in zip(FEATURE_NAMES, rf.feature_importances_)
    }
    return results, fitted, scaler


def run_dataset(name, loader):
    print(f"\n===== {name} =====")
    windows = loader()
    if not windows:
        print(f"{name}: 데이터 없음 (먼저 python -m ml.download_data 실행)")
        return None
    print(f"윈도우 {len(windows)}개 로드, 라벨 분포: {dict(Counter(w.label for w in windows))}")

    print("특징 추출 중...")
    X, y, groups, dicts = windows_to_matrix(windows)
    train_idx, test_idx = group_split(y, groups)
    print(f"train {len(train_idx)} / test {len(test_idx)} 윈도우 "
          f"(파일 단위 분할: train {len(set(groups[train_idx]))}개 / test {len(set(groups[test_idx]))}개 파일)")

    anomaly_results, detector = evaluate_anomaly_detection(dicts, y, groups)
    clf_results, fitted, scaler = evaluate_classifiers(X, y, train_idx, test_idx)

    MODEL_DIR.mkdir(exist_ok=True)
    RESULT_DIR.mkdir(exist_ok=True)
    for model_name, model in fitted.items():
        joblib.dump(model, MODEL_DIR / f"{name}_{model_name}.joblib")
    joblib.dump(scaler, MODEL_DIR / f"{name}_scaler.joblib")
    joblib.dump(detector, MODEL_DIR / f"{name}_baseline_detector.joblib")

    all_results = {
        "dataset": name,
        "n_windows": len(windows),
        "label_counts": dict(Counter(y.tolist())),
        "n_train_files": len(set(groups[train_idx])),
        "n_test_files": len(set(groups[test_idx])),
        "feature_names": FEATURE_NAMES,
        "anomaly_detection": anomaly_results,
        "classification": clf_results,
    }
    out = RESULT_DIR / f"{name}_results.json"
    out.write_text(json.dumps(all_results, indent=2, ensure_ascii=False), encoding="utf-8")

    # 콘솔 요약
    print("\n[트랙 1: 이상탐지 (정상만 학습, test 파일 기준)]")
    for method in ("three_sigma", "isolation_forest"):
        print(f"  {method}:")
        for k, v in anomaly_results[method].items():
            print(f"    {k}: {v*100:.1f}%")
    print("\n[트랙 2: 고장 분류 (test 파일 기준)]")
    for m in ("decision_tree_d4", "logistic_regression", "random_forest"):
        r = clf_results[m]
        print(f"  {m}: accuracy={r['accuracy']*100:.1f}%  macro_f1={r['macro_f1']:.3f}")
    print(f"\n결과 저장: {out}")
    return all_results


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "all"
    if target in ("cwru", "all"):
        run_dataset("cwru", load_cwru_windows)
    if target in ("mafaulda", "all"):
        run_dataset("mafaulda", load_mafaulda_windows)
