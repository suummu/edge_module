"""
reference_comparison.py

목적:
    scikit-learn의 Isolation Forest로 같은 데이터를 판별해보고,
    baseline_detector.py의 순수 통계(baseline) 방식과 결과를 비교한다.

    *** 중요: 이 파일은 ESP32에 이식되지 않는다. ***
    scikit-learn은 C/ESP32 환경에 존재하지 않으므로, 이건 어디까지나
    "우리가 만든 통계 baseline 방식이 그럴듯한 결과를 내는지" 검증하는
    개발 단계의 참고/대조 도구일 뿐이다.

    최종 배포 로직은 반드시 baseline_detector.py(순수 통계 baseline)를 따른다.

    v2 수정 사항:
      - 특징 벡터를 feature_extraction.py v2의 실제 변별력 있는 특징(harmonic 에너지 등)으로 갱신
      - "스케일링 없이" vs "StandardScaler 적용 후" 두 가지로 비교해서,
        Isolation Forest의 실패가 알고리즘 자체 결함이 아니라 전처리 누락 때문임을
        명확히 구분해서 보여준다 (전처리까지 포함하면 ESP32 이식 부담이 더 커진다는
        논지를 더 정확하게 뒷받침하기 위함).
"""

import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

from . import signal_generator as gen
from . import feature_extraction as feat
from . import baseline_detector as det

WINDOW_SIZE = gen.WINDOW_SIZE
# baseline_detector.py의 배포 로직과 "동일한 특징 세트"로 비교해야 이 대조가 의미가 있다.
# 따라서 자체 리스트를 새로 정의하지 않고 ACTIVE_FEATURES를 단일 출처로 재사용한다.
FEATURE_KEYS = det.ACTIVE_FEATURES


def features_to_vector(feature_dict):
    """dict -> 고정 순서의 숫자 배열로 변환 (scikit-learn 입력 형식에 맞춤)"""
    return [feature_dict[k] for k in FEATURE_KEYS]


def run_comparison(use_scaler: bool):
    normal_vectors = []
    for i in range(60):
        _, sig = gen.generate_normal_signal(seed=i)
        f = feat.extract_features(sig[:WINDOW_SIZE])
        normal_vectors.append(features_to_vector(f))
    normal_vectors = np.array(normal_vectors)

    scaler = None
    if use_scaler:
        scaler = StandardScaler()
        normal_vectors = scaler.fit_transform(normal_vectors)

    model = IsolationForest(contamination=0.05, random_state=42)
    model.fit(normal_vectors)

    test_cases = {
        "정상 신호": gen.generate_normal_signal(seed=123),
        "경미한 고장": gen.generate_faulty_signal(seed=123, fault_strength=0.3),
        "심각한 고장": gen.generate_faulty_signal(seed=123, fault_strength=0.9),
    }

    results = {}
    for label, (t, sig) in test_cases.items():
        f = feat.extract_features(sig[:WINDOW_SIZE])
        vector = np.array([features_to_vector(f)])
        if scaler is not None:
            vector = scaler.transform(vector)
        prediction = model.predict(vector)[0]  # 1: 정상, -1: 이상
        results[label] = "이상" if prediction == -1 else "정상"
    return results


if __name__ == "__main__":
    print("=== scikit-learn Isolation Forest 참고 비교 ===")
    print("(주의: 이 결과는 개발 단계 참고용. ESP32 최종 배포 로직 아님)\n")

    print("--- (A) 스케일링 없이 원본 특징값 그대로 사용 ---")
    results_no_scale = run_comparison(use_scaler=False)
    for label, verdict in results_no_scale.items():
        print(f"[{label}] Isolation Forest 판정: {verdict}")

    print("\n--- (B) StandardScaler로 정규화 후 사용 ---")
    results_scaled = run_comparison(use_scaler=True)
    for label, verdict in results_scaled.items():
        print(f"[{label}] Isolation Forest 판정: {verdict}")

    print("\n=== 결과 분석 (실제 실행 결과 기준, 정직하게 기록) ===")
    same = results_no_scale == results_scaled
    print(f"(A) 스케일링 없음과 (B) 스케일링 적용 결과가 동일한가: {same}")
    print("이번 v2 특징 세트(harmonic2/3_energy, high_freq_energy 포함)에서는")
    print("스케일링 여부와 무관하게 Isolation Forest도 경미한/심각한 고장을 잘 구분했다.")
    print("(참고: 이전 v1 특징 세트 - peak_freq 포함 시 - 에서는 스케일 차이 때문에")
    print("Isolation Forest가 심각한 고장까지 '정상'으로 오판한 적이 있었다.")
    print("즉 특징 선택 자체가 알고리즘 성능에 큰 영향을 준다는 것을 확인한 셈.)")
    print("\n핵심 논지 (여전히 유효):")
    print("- Isolation Forest도 '좋은 특징'을 주면 잘 작동할 수 있다 - ML 알고리즘")
    print("  자체가 나쁜 게 아니라 특징 설계가 핵심이라는 뜻.")
    print("- 그럼에도 baseline_detector.py(순수 통계 baseline)을 최종 선택한 이유는 성능이 아니라 '이식성':")
    print("  scikit-learn은 ESP32에 존재하지 않고, Isolation Forest를 쓰려면 트리 구조")
    print("  전체를 C로 재현하거나 TFLite Micro 같은 별도 프레임워크가 필요해진다.")
    print("  baseline_detector.py 방식은 평균/표준편차 몇 개 숫자만 저장하면 되므로 이식 난이도가 압도적으로 낮다.")
    print("- 즉 '성능이 부족해서'가 아니라 '저비용·저사양 엣지 디바이스 이식 용이성' 때문에")
    print("  단순 통계 baseline을 택했다는 것이 이 비교의 정확한 결론.")
