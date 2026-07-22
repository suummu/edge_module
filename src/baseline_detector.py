"""
baseline_detector.py

목적:
    feature_extraction.py에서 뽑은 특징값들을 가지고 "정상/이상"을 실제로 판별하는 로직.

    엣지알리미 논리정리에서 확정한 2중 판단 구조를 그대로 구현:
      1) 상대적 판정 (학습 기반): 정상 데이터의 평균/표준편차로 baseline을 만들고,
         새 데이터가 거기서 얼마나 벗어났는지로 판별
      2) 절대적 판정 (규칙 기반): 데이터시트 상 절대 넘으면 안 되는 고정값
         (지금은 실제 하드웨어가 없으므로 임의 값으로 자리만 잡아둠 -> TODO 표시)

    v2에서 추가한 것 (첫 버전에서 빠져 있던, 문서에 이미 정리했던 핵심 원칙들):
      3) 연속성 필터링: 단발성 노이즈로 인한 오탐을 줄이기 위해, 연속 N개 윈도우
         중 일정 비율 이상 이상 판정이 나와야 "확정 이상"으로 격상
      4) 위험 정상화(Normalization of Deviance) 방지: baseline을 무비판적으로
         계속 갱신하면, 서서히 나빠지는 걸 "새로운 정상"으로 착각하게 된다.
         이를 막기 위해 원본 baseline을 별도 보관하고, 원본 대비 이탈폭이
         특정 한도를 넘으면 경고를 발생시킨다.

    주의: 지금은 scikit-learn 없이, 순수 통계식(평균/표준편차)만으로 구현한다.
    이유: ESP32에는 scikit-learn이 없어서 결국 이 순수 통계 방식으로 이식해야 하기 때문에,
    파이썬 검증 단계부터 "실제로 이식 가능한 형태"로 설계하는 것이 낫다.
    (Isolation Forest 등은 별도로 reference_comparison.py에서 "참고 비교용"으로만 다룬다.)
"""

import numpy as np
from collections import deque

from . import signal_generator as gen
from . import feature_extraction as feat

WINDOW_SIZE = gen.WINDOW_SIZE

# ---- 절대 임계값 (TODO: 실제 센서·모터 데이터시트 확보 후 갱신) ----
# 지금은 하드웨어가 없어 임의로 "정상 RMS의 3배를 넘으면 무조건 위험"으로 임시 설정.
ABSOLUTE_RMS_LIMIT = None  # 나중에 데이터시트 기반 값으로 채울 자리. None이면 절대판정 비활성.

# ---- 위험 정상화 방지 파라미터 ----
# 원본 baseline 평균 대비, 지금 들어오는 값들의 평균이 이 배수 이상 벌어지면
# "서서히 나빠지고 있다"는 경고를 별도로 띄운다 (재보정과 무관하게 항상 확인).
DRIFT_WARNING_RATIO = 1.5

# ---- 판별에 실제 사용할 특징 ----
# peak_freq는 feature_extraction.py에서 확인했듯 표준편차가 0에 가까워 판별력이 없으므로 제외.
# peak_magnitude도 정상/고장 차이가 거의 없어 제외. rotation_hz_estimate는 하모닉 밴드
# 위치를 정하는 기준축일 뿐 판별용 특징이 아니므로 제외.
# kurtosis는 제외: 베어링 결함류 지표인데 본 프로젝트는 베어링 정밀진단을 범위 밖으로
# 뒀고, 시뮬레이션 신호(사인파 합성)가 애초에 임펄스성이 아니라 판별력을 기대하기 어려움.
# 실측 데이터로 판별력이 확인되면 재검토 대상.
# harmonic1/2/3_energy는 각각 불평형(1x)/정렬불량(2x)/이완(2x,3x,4x 일부) 고장에
# 직접 대응하는 지표로 구성.
ACTIVE_FEATURES = ["rms", "harmonic1_energy", "harmonic2_energy", "harmonic3_energy", "high_freq_energy"]


class BaselineDetector:
    """
    정상 데이터로 baseline(평균/표준편차)을 학습하고,
    새로운 특징값이 정상 범위를 벗어나는지 판별하는 클래스.

    사용 흐름:
        detector = BaselineDetector()
        detector.fit(정상_특징값_리스트)          # baseline 학습 (실제로는 수일치 데이터 필요)
        result = detector.judge(새_특징값)         # 판별
    """

    def __init__(self, n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                 features_to_check=None):
        """
        n_sigma: 평균에서 몇 표준편차 벗어나면 "1차 의심"으로 볼지
        confirm_window: 연속 몇 개 윈도우를 보고 "확정 이상" 여부를 판단할지
        confirm_ratio: confirm_window 중 몇 비율 이상이 의심 판정이어야 확정할지
        features_to_check: 판별에 실제로 쓸 특징 이름 리스트.
            None이면 fit()에 들어온 모든 키를 사용.
            (peak_freq처럼 해상도 문제로 변별력이 없는 특징은 여기서 제외 가능)
        """
        self.n_sigma = n_sigma
        self.means = {}
        self.stds = {}
        self.fitted = False
        self.features_to_check = features_to_check

        # 원본 baseline (위험 정상화 방지를 위해 최초 학습값을 절대 덮어쓰지 않고 보관)
        self.original_means = {}

        # 연속성 필터링을 위한 최근 판정 이력
        self.confirm_window = confirm_window
        self.confirm_ratio = confirm_ratio
        self._recent_suspicions = deque(maxlen=confirm_window)

    def fit(self, feature_dicts):
        """
        feature_dicts: extract_features()가 반환하는 dict의 리스트.
        여러 개의 "정상 상태" 윈도우에서 뽑은 특징값들이 들어와야 함.

        실제 배포 시에는 "수일간 수집한 정상 데이터"가 여기 들어가야 한다.
        (엣지알리미 논리정리에서 강조한 "자가 보정은 며칠이 필요하다"는 제약이 여기 해당)
        """
        keys = feature_dicts[0].keys()
        if self.features_to_check is None:
            self.features_to_check = list(keys)

        for key in keys:
            values = np.array([f[key] for f in feature_dicts])
            self.means[key] = float(np.mean(values))
            self.stds[key] = float(np.std(values))

        # 최초 학습 시에만 원본을 기록 (재보정 시에는 갱신하지 않음)
        if not self.fitted:
            self.original_means = dict(self.means)

        self.fitted = True

    def _check_drift(self, feature_dict):
        """
        위험 정상화(Normalization of Deviance) 감시.

        현재 관측값이 '원본 baseline' 대비 얼마나 벌어져 있는지 확인한다.
        판정 자체(is_anomaly)와는 별개로, "장기적으로 서서히 나빠지고 있는지"를
        경고하는 용도. std가 아주 작아 나눗셈이 불안정한 경우는 절댓값 비교로 대체.
        """
        drift_warnings = []
        for key in self.features_to_check:
            orig = self.original_means.get(key, 0)
            current = feature_dict.get(key, 0)
            if abs(orig) < 1e-9:
                continue
            ratio = current / orig if orig != 0 else float("inf")
            if ratio >= DRIFT_WARNING_RATIO:
                drift_warnings.append(
                    f"{key} 원본 baseline 대비 {ratio:.2f}배 (원본={orig:.4f}, 현재={current:.4f})"
                )
        return drift_warnings

    def judge(self, feature_dict):
        """
        하나의 새 특징값 세트를 받아 정상/이상을 판별.

        반환값:
            dict {
                "is_suspect": bool,       # 이번 한 윈도우만으로 본 1차 의심 여부
                "is_anomaly": bool,       # 연속성 필터까지 반영한 확정 이상 여부
                "reasons": [...],         # 1차 의심 사유
                "drift_warnings": [...],  # 위험 정상화(서서히 나빠짐) 경고
                "details": {...},
            }
        """
        if not self.fitted:
            raise RuntimeError("먼저 fit()으로 baseline을 학습해야 합니다.")

        reasons = []
        details = {}

        # --- 1) 상대적 판정 (학습 기반, 3-sigma 규칙) ---
        # features_to_check에 있는 특징만 판별에 사용 (peak_freq 등 변별력 없는 값 제외 가능)
        for key in self.features_to_check:
            if key not in feature_dict:
                continue
            value = feature_dict[key]
            mean = self.means[key]
            std = self.stds[key]
            lower = mean - self.n_sigma * std
            upper = mean + self.n_sigma * std
            details[key] = (value, (round(lower, 4), round(upper, 4)))
            if value < lower or value > upper:
                reasons.append(f"{key} 정상범위 이탈 (관측값={value:.4f}, 정상범위={lower:.4f}~{upper:.4f})")

        # --- 2) 절대적 판정 (규칙 기반, 데이터시트 고정값) ---
        if ABSOLUTE_RMS_LIMIT is not None and feature_dict.get("rms", 0) > ABSOLUTE_RMS_LIMIT:
            reasons.append(f"[절대판정] RMS가 안전 한계({ABSOLUTE_RMS_LIMIT})를 초과함")

        is_suspect = len(reasons) > 0

        # --- 3) 연속성 필터링 ---
        # 단발성 노이즈로 인한 오탐을 줄이기 위해, 최근 N개 윈도우 중
        # confirm_ratio 이상이 의심 판정일 때만 "확정 이상"으로 격상한다.
        self._recent_suspicions.append(is_suspect)
        suspicion_rate = sum(self._recent_suspicions) / len(self._recent_suspicions)
        is_confirmed_anomaly = (
            len(self._recent_suspicions) == self.confirm_window
            and suspicion_rate >= self.confirm_ratio
        )

        # --- 4) 위험 정상화 감시 ---
        drift_warnings = self._check_drift(feature_dict)

        return {
            "is_suspect": is_suspect,
            "is_anomaly": is_confirmed_anomaly,
            "reasons": reasons,
            "drift_warnings": drift_warnings,
            "details": details,
            "suspicion_rate_in_window": round(suspicion_rate, 2),
        }

    def reset_confirmation_state(self):
        """새로운 설비/세션을 시작할 때 연속성 필터 이력을 초기화."""
        self._recent_suspicions.clear()


def build_normal_baseline(n_windows=60):
    """
    정상 신호에서 여러 윈도우를 뽑아 baseline 학습용 특징값 리스트를 만든다.
    실제로는 이 부분이 "수일간 실제 센서로 수집한 정상 데이터"로 대체되어야 한다.

    v2: n_windows 기본값을 20 -> 60으로 늘림. 표준편차 추정이 너무 적은 샘플에
    좌우되지 않도록 하기 위함 (여전히 실제로는 훨씬 더 많은 실측 데이터가 필요함).
    """
    feature_list = []
    for i in range(n_windows):
        _, sig = gen.generate_normal_signal(seed=i)  # seed를 바꿔가며 약간씩 다른 정상 신호 생성
        features = feat.extract_features(sig[:WINDOW_SIZE])
        feature_list.append(features)
    return feature_list


def evaluate_detection_rate(detector, n_trials=100, fault_strength=0.5):
    """
    정량적 성능 확인: 정상/고장 신호를 각각 n_trials번 만들어 판정시켜서
    오탐율(정상인데 이상으로 판정)과 미탐율(고장인데 정상으로 판정)을 계산.

    주의: 이 수치는 어디까지나 '가짜 신호' 기준이라 실제 하드�웨어 성능을
    대변하지 않는다. 하지만 로직 자체의 내적 일관성을 확인하는 용도로는 유효하다.
    confirm_window 로직 때문에 단발 판정은 is_suspect로, 확정 판정은 is_anomaly로 본다.
    """
    false_positive = 0
    for i in range(n_trials):
        detector.reset_confirmation_state()
        _, sig = gen.generate_normal_signal(seed=1000 + i)
        result = detector.judge(feat.extract_features(sig[:WINDOW_SIZE]))
        if result["is_suspect"]:
            false_positive += 1

    false_negative = 0
    for i in range(n_trials):
        detector.reset_confirmation_state()
        _, sig = gen.generate_faulty_signal(seed=2000 + i, fault_strength=fault_strength)
        result = detector.judge(feat.extract_features(sig[:WINDOW_SIZE]))
        if not result["is_suspect"]:
            false_negative += 1

    return {
        "false_positive_rate": false_positive / n_trials,
        "false_negative_rate": false_negative / n_trials,
        "n_trials": n_trials,
        "fault_strength_tested": fault_strength,
    }


def evaluate_confirmed_false_positive_rate(n_windows=200, seed=42):
    """
    Scenario A: continuity filter를 통과한 뒤의 "진짜" 오탐율.

    evaluate_detection_rate()가 재는 오탐율은 is_suspect(연속성 필터 통과 전
    1차 판정) 기준이고, 매 시행마다 reset_confirmation_state()를 호출해서
    confirm_window가 애초에 쌓일 기회가 없다. 실제 경보는 is_anomaly(확정
    이상)로 울리므로, 이 함수는 정상 신호만 연속으로(세션 시작 시 딱 한 번만
    리셋한 채) 흘려보내 is_anomaly가 실제로 몇 번 뜨는지를 잰다.
    """
    normal_features = build_normal_baseline(n_windows=60)
    detector = BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                                 features_to_check=ACTIVE_FEATURES)
    detector.fit(normal_features)
    detector.reset_confirmation_state()

    rng = np.random.default_rng(seed)
    rot_tracker = feat.RotationTracker()
    confirmed_count = 0
    for _ in range(n_windows):
        window_seed = int(rng.integers(0, 1_000_000))
        _, sig = gen.generate_normal_signal(seed=window_seed)
        features = feat.extract_features(sig[:WINDOW_SIZE], rotation=rot_tracker)
        result = detector.judge(features)
        if result["is_anomaly"]:
            confirmed_count += 1

    return {
        "confirmed_false_positive_rate": confirmed_count / n_windows,
        "n_windows": n_windows,
    }


def measure_detection_latency(n_trials=20, fault_strength=0.5, normal_windows_before=10,
                               max_windows_after_fault=30, seed=100):
    """
    Scenario B: 오탐 억제(continuity filter)의 대가로 탐지가 얼마나 늦어지는지 측정.

    정상 -> 고장으로 전환되는 연속 시퀀스(트라이얼 내부는 리셋 없는 하나의 스트림)를
    만들어, 고장이 시작된 시점부터 is_anomaly가 처음 뜨는 시점까지 몇 윈도우가
    걸리는지 잰다. confirm_window=5, confirm_ratio=0.6이므로 이론적 최소 지연은
    3윈도우(5개 중 3개 의심)다. max_windows_after_fault까지 안 뜨면 미탐지로 기록.
    """
    normal_features = build_normal_baseline(n_windows=60)
    rng = np.random.default_rng(seed)

    latencies = []
    missed_trials = 0
    for _ in range(n_trials):
        detector = BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                                     features_to_check=ACTIVE_FEATURES)
        detector.fit(normal_features)
        detector.reset_confirmation_state()
        rot_tracker = feat.RotationTracker()

        for _ in range(normal_windows_before):
            window_seed = int(rng.integers(0, 1_000_000))
            _, sig = gen.generate_normal_signal(seed=window_seed)
            features = feat.extract_features(sig[:WINDOW_SIZE], rotation=rot_tracker)
            detector.judge(features)

        detected_at = None
        for i in range(max_windows_after_fault):
            window_seed = int(rng.integers(0, 1_000_000))
            _, sig = gen.generate_faulty_signal(seed=window_seed, fault_strength=fault_strength)
            features = feat.extract_features(sig[:WINDOW_SIZE], rotation=rot_tracker)
            result = detector.judge(features)
            if result["is_anomaly"]:
                detected_at = i + 1  # 고장 시작 후 몇 번째 윈도우에서 확정됐는지 (1-based)
                break

        if detected_at is None:
            missed_trials += 1
        else:
            latencies.append(detected_at)

    return {
        "n_trials": n_trials,
        "fault_strength_tested": fault_strength,
        "detected_trials": len(latencies),
        "missed_trials": missed_trials,
        "latencies_windows": latencies,
        "mean_latency_windows": float(np.mean(latencies)) if latencies else None,
    }


if __name__ == "__main__":
    print("=== Baseline 학습 (정상 신호 60개 윈도우 사용) ===")
    normal_features = build_normal_baseline(n_windows=60)
    detector = BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                                 features_to_check=ACTIVE_FEATURES)
    detector.fit(normal_features)

    for key in ACTIVE_FEATURES:
        print(f"{key:<18} 평균={detector.means[key]:.4f}  표준편차={detector.stds[key]:.4f}")

    print("\n=== 판별 테스트 (단발 윈도우) ===")

    def run_case(label, t_sig):
        detector.reset_confirmation_state()
        t, sig = t_sig
        result = detector.judge(feat.extract_features(sig[:WINDOW_SIZE]))
        print(f"\n[{label}] 1차 의심: {result['is_suspect']}")
        for r in result["reasons"]:
            print(f"  - {r}")
        if result["drift_warnings"]:
            print("  [주의] 위험 정상화 경고:")
            for w in result["drift_warnings"]:
                print(f"    - {w}")

    run_case("정상 신호", gen.generate_normal_signal(seed=99))
    run_case("경미한 고장", gen.generate_faulty_signal(seed=99, fault_strength=0.3))
    run_case("심각한 고장", gen.generate_faulty_signal(seed=99, fault_strength=0.9))

    print("\n=== 연속성 필터링 테스트 (확정 이상으로 격상되는 과정) ===")
    detector.reset_confirmation_state()
    rot_tracker = feat.RotationTracker()  # 연속 스트림이므로 윈도우 간 회전수 추정을 체이닝
    print("동일한 심각한 고장 신호를 연속 5개 윈도우에 걸쳐 넣어봄:")
    for i in range(5):
        _, sig = gen.generate_faulty_signal(seed=500 + i, fault_strength=0.9)
        features = feat.extract_features(sig[:WINDOW_SIZE], rotation=rot_tracker)
        result = detector.judge(features)
        print(f"  윈도우 {i+1}: 1차의심={result['is_suspect']}  "
              f"확정이상={result['is_anomaly']}  "
              f"(최근{detector.confirm_window}개 중 의심비율={result['suspicion_rate_in_window']})")

    print("\n=== 위험 정상화(서서히 나빠짐) 시나리오 테스트 ===")
    detector.reset_confirmation_state()
    rot_tracker = feat.RotationTracker()  # 연속 스트림이므로 윈도우 간 회전수 추정을 체이닝
    signals, strengths = gen.generate_progressive_wear_sequence(n_windows=10, final_fault_strength=0.9, seed=7)
    print("50 윈도우 전체 대신, 대표로 10단계만 보여줌 (fault_strength 0 -> 0.9로 서서히 증가):")
    for i, (sig, strength) in enumerate(zip(signals, strengths)):
        features = feat.extract_features(sig[:WINDOW_SIZE], rotation=rot_tracker)
        result = detector.judge(features)
        drift_flag = " <- 위험 정상화 경고!" if result["drift_warnings"] else ""
        print(f"  단계{i+1:2d} (진행도={strength:.2f}): 1차의심={result['is_suspect']}{drift_flag}")

    print("\n=== 정량적 성능 평가 (가짜 신호 기준, 참고용) ===")
    detector.reset_confirmation_state()
    metrics = evaluate_detection_rate(detector, n_trials=100, fault_strength=0.5)
    print(f"시행 횟수: {metrics['n_trials']}회, 테스트한 고장 강도: {metrics['fault_strength_tested']}")
    print(f"오탐율 (정상을 이상으로 잘못 판정): {metrics['false_positive_rate']*100:.1f}%")
    print(f"미탐율 (고장을 정상으로 잘못 판정): {metrics['false_negative_rate']*100:.1f}%")
    print("(주의: 가짜 신호 기준 수치이며 실제 하드웨어 성능을 보장하지 않음)")
    print("\n[오탐 원인 메모] 3-sigma 규칙은 baseline 학습 샘플 수(60개)가 유한하기 때문에")
    print("모집단 표준편차를 완벽히 추정하지 못한다. 그래서 순수 통계적으로도 일정 비율의")
    print("오탐(약 0.3%가 이론치지만 표본 수가 적어 더 높게 나타남)은 항상 발생할 수 있다.")
    print("실제 배포 시 오탐율을 낮추려면: (a) baseline 학습 샘플을 늘리거나,")
    print("(b) n_sigma를 3.5~4로 높이거나, (c) 연속성 필터(confirm_window)로 단발 오탐을 거르면 된다.")
    print("지금 confirm_window=5, confirm_ratio=0.6이 바로 이 (c) 역할을 하고 있음.")

    print("\n=== Scenario A: 확정(is_anomaly) 기준 진짜 오탐율 ===")
    print("(위 오탐율은 is_suspect 기준 - continuity filter 통과 전 1차 판정이다.")
    print(" 실제 경보는 is_anomaly로 울리므로, 정상 신호를 리셋 없이 연속으로 흘려서 다시 잰다.)")
    scenario_a = evaluate_confirmed_false_positive_rate(n_windows=200)
    print(f"윈도우 수: {scenario_a['n_windows']}개 (연속 스트림, 세션당 1회만 리셋)")
    print(f"확정 오탐율 (is_anomaly 기준): {scenario_a['confirmed_false_positive_rate']*100:.1f}%")

    print("\n=== Scenario B: 탐지 지연(latency) - Scenario A의 대가 ===")
    scenario_b = measure_detection_latency(n_trials=20, fault_strength=0.5)
    print(f"시행 횟수: {scenario_b['n_trials']}회, 테스트한 고장 강도: {scenario_b['fault_strength_tested']}")
    print(f"탐지 성공: {scenario_b['detected_trials']}회, 미탐지: {scenario_b['missed_trials']}회")
    if scenario_b["mean_latency_windows"] is not None:
        print(f"평균 탐지 지연: 고장 시작 후 {scenario_b['mean_latency_windows']:.1f}윈도우 "
              f"(개별: {scenario_b['latencies_windows']})")
    else:
        print("탐지 지연: 측정된 성공 사례 없음 (전부 미탐지)")
    print("\n[정직한 트레이드오프] Scenario A의 낮아진 오탐율만 내세우면 안 된다 -")
    print("continuity filter가 단발 오탐을 걸러주는 대신, 그만큼 확정 판정이 늦게 뜬다.")
    print("confirm_window=5, confirm_ratio=0.6 기준 이론적 최소 지연은 3윈도우다.")

    print("\n다음 단계: reference_comparison.py 에서 scikit-learn 방식과 비교 참고")
    print("이후 실제 하드웨어 데이터 확보되면 signal_generator.py만 실제 데이터 리더로 교체")
