"""
export_test_vectors.py

목적:
    파이썬(numpy/scipy, float64)으로 계산한 "정답"을 C 헤더 파일(test_vectors.h)로
    내보내서, ESP32 이식 코드(float32, 직접 구현한 FFT/통계)가 같은 입력에 같은
    출력을 내는지 기기 위에서 자동 대조 검증할 수 있게 한다.

    왜 필요한가:
      C/C++에는 numpy.fft, scipy.stats 같은 내장 모듈이 없어 전부 직접 구현했고,
      파이썬은 64비트(double) 연산 / ESP32는 32비트(float) 연산이라 결과값이
      미세하게 달라질 수 있다. "로직이 같다"는 주장만으로는 부족하고,
      실제 수치가 허용 오차 안에서 일치함을 기기에서 확인해야 이식 검증이 성립한다.
      (중간계획서 Ⅳ-6 "같은 입력에 같은 출력을 내는지 파이썬 코드와 대조 검증" 항목의 구현)

    내보내는 것:
      1. 테스트 신호 6개 윈도우 (정상 3 + 고장 3, 각 256샘플) — 입력
      2. 각 윈도우의 기대 특징값 5개(ACTIVE_FEATURES)와 기대 회전주파수 추정 — 출력 정답
      3. baseline 학습용 특징 행렬(60윈도우 x 5특징)과 기대 평균/표준편차 — 통계 계산 정답
      4. 각 테스트 윈도우의 기대 1차 의심(is_suspect) 판정 — 판별 로직 정답

    사용법 (repo 루트에서):
      python -m tools.export_test_vectors
      -> esp32/edgealimi_v2_realsensor/test_vectors.h 생성/갱신
      -> .ino의 VALIDATION_MODE를 1로 바꿔 업로드하면 기기에서 PASS/FAIL 출력
"""

import os

import numpy as np

from src import signal_generator as gen
from src import feature_extraction as feat
from src.baseline_detector import BaselineDetector, ACTIVE_FEATURES, build_normal_baseline

WINDOW_SIZE = gen.WINDOW_SIZE

# ESP32 쪽 enum FeatureIndex 순서와 반드시 일치해야 한다
assert ACTIVE_FEATURES == [
    "rms", "harmonic1_energy", "harmonic2_energy", "harmonic3_energy", "high_freq_energy"
], "ACTIVE_FEATURES 순서가 바뀜 — .ino의 enum FeatureIndex도 함께 갱신할 것"

# 테스트 케이스: (라벨, 신호 생성 함수 인자)
TEST_CASES = [
    ("normal seed=99",        dict(faulty=False, seed=99)),
    ("normal seed=1234",      dict(faulty=False, seed=1234)),
    ("normal seed=42",        dict(faulty=False, seed=42)),
    ("faulty s=0.3 seed=99",  dict(faulty=True, seed=99, fault_strength=0.3)),
    ("faulty s=0.5 seed=2024", dict(faulty=True, seed=2024, fault_strength=0.5)),
    ("faulty s=0.9 seed=7",   dict(faulty=True, seed=7, fault_strength=0.9)),
]


def make_signal(faulty, seed, fault_strength=None):
    if faulty:
        _, sig = gen.generate_faulty_signal(seed=seed, fault_strength=fault_strength)
    else:
        _, sig = gen.generate_normal_signal(seed=seed)
    return sig[:WINDOW_SIZE]


def fmt_float_array(values, per_line=8, indent="  "):
    """C float 배열 리터럴 생성 (float32로 반올림해 f 접미사 부여)."""
    vals32 = np.asarray(values, dtype=np.float32)
    lines = []
    for i in range(0, len(vals32), per_line):
        chunk = ", ".join(f"{v:.9g}f" for v in vals32[i:i + per_line])
        lines.append(indent + chunk)
    return ",\n".join(lines)


def main():
    # --- 1) 테스트 신호 + 기대 특징값/회전추정 ---
    signals = []
    expected_features = []      # [n][5]
    expected_rotation = []      # [n]
    for label, kwargs in TEST_CASES:
        sig = make_signal(**kwargs)
        # 파이썬 쪽도 float32로 낮춘 신호로 계산한다 — 입력을 완전히 동일하게 맞춰서
        # 남는 차이가 "연산 정밀도와 FFT 구현 차이"만 되도록 격리하기 위함.
        sig32 = sig.astype(np.float32).astype(np.float64)
        f = feat.extract_features(sig32, rotation=None)  # 독립 추정 (체이닝 없음)
        signals.append(sig32)
        expected_features.append([f[k] for k in ACTIVE_FEATURES])
        expected_rotation.append(f["rotation_hz_estimate"])

    # --- 2) baseline 특징 행렬 + 기대 평균/표준편차 ---
    baseline_dicts = build_normal_baseline(n_windows=60)
    baseline_matrix = [[d[k] for k in ACTIVE_FEATURES] for d in baseline_dicts]

    detector = BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                                 features_to_check=ACTIVE_FEATURES)
    detector.fit(baseline_dicts)
    expected_means = [detector.means[k] for k in ACTIVE_FEATURES]
    expected_stds = [detector.stds[k] for k in ACTIVE_FEATURES]

    # --- 3) 기대 판별 결과 (1차 의심) ---
    # 주의: C 쪽 판별 검증은 "임베드된 파이썬 특징값"으로 judge를 돌려 불리언을 비교한다.
    # (C가 자체 추출한 특징값으로 judge하면 경계 근처에서 정밀도 차이로 갈릴 수 있어,
    #  판별 로직 자체의 검증과 특징 추출의 검증을 분리하는 것)
    expected_suspect = []
    for feats in expected_features:
        detector.reset_confirmation_state()
        fd = dict(zip(ACTIVE_FEATURES, feats))
        result = detector.judge(fd)
        expected_suspect.append(result["is_suspect"])

    # --- 4) 헤더 파일 생성 ---
    out_path = os.path.join(os.path.dirname(__file__), "..",
                            "esp32", "edgealimi_v2_realsensor", "test_vectors.h")
    out_path = os.path.abspath(out_path)

    n = len(TEST_CASES)
    parts = []
    parts.append("// test_vectors.h — tools/export_test_vectors.py가 자동 생성. 직접 수정 금지.")
    parts.append("// 파이썬(float64, numpy/scipy)으로 계산한 정답값. ESP32 이식 코드의 대조 검증용.")
    parts.append("#pragma once")
    parts.append("")
    parts.append(f"const uint16_t TV_N_WINDOWS = {n};")
    parts.append(f"const uint16_t TV_N_BASELINE = 60;")
    parts.append("")

    labels = ", ".join(f'"{label}"' for label, _ in TEST_CASES)
    parts.append(f"const char *TV_LABELS[{n}] = {{ {labels} }};")
    parts.append("")

    parts.append(f"const float TV_SIGNALS[{n}][{WINDOW_SIZE}] = {{")
    for sig in signals:
        parts.append("{\n" + fmt_float_array(sig) + "\n},")
    parts.append("};")
    parts.append("")

    parts.append(f"const float TV_EXPECTED_FEATURES[{n}][5] = {{")
    for feats in expected_features:
        parts.append("  { " + ", ".join(f"{v:.9g}f" for v in feats) + " },")
    parts.append("};")
    parts.append("")

    rot = ", ".join(f"{v:.9g}f" for v in expected_rotation)
    parts.append(f"const float TV_EXPECTED_ROTATION_HZ[{n}] = {{ {rot} }};")
    parts.append("")

    susp = ", ".join("true" if s else "false" for s in expected_suspect)
    parts.append(f"const bool TV_EXPECTED_SUSPECT[{n}] = {{ {susp} }};")
    parts.append("")

    parts.append("const float TV_BASELINE_FEATURES[60][5] = {")
    for row in baseline_matrix:
        parts.append("  { " + ", ".join(f"{v:.9g}f" for v in row) + " },")
    parts.append("};")
    parts.append("")

    means = ", ".join(f"{v:.9g}f" for v in expected_means)
    stds = ", ".join(f"{v:.9g}f" for v in expected_stds)
    parts.append(f"const float TV_EXPECTED_MEANS[5] = {{ {means} }};")
    parts.append(f"const float TV_EXPECTED_STDS[5] = {{ {stds} }};")
    parts.append("")

    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(parts))

    print(f"생성 완료: {out_path}")
    print(f"테스트 윈도우 {n}개, baseline 60윈도우")
    print("기대 판별 결과 (is_suspect):")
    for (label, _), s in zip(TEST_CASES, expected_suspect):
        print(f"  {label:<24} -> {'의심' if s else '정상'}")
    print("\n다음 단계: .ino의 VALIDATION_MODE를 1로 바꿔 컴파일·업로드하면")
    print("ESP32가 이 정답값들과 자기 계산 결과를 비교해 PASS/FAIL을 출력한다.")


if __name__ == "__main__":
    main()
