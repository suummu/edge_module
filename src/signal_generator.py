"""
signal_generator.py

목적:
    실제 ESP32 + MPU-6050 하드웨어가 아직 없거나, 데이터 수집이 안 된 상태에서도
    FFT -> 특징추출 -> 판별 로직을 먼저 설계·검증하기 위한 "가짜 진동 신호 생성기".

    나중에 실제 센서 데이터가 들어오면, 이 파일에서 만드는 배열(signal)을
    "실제 CSV/시리얼에서 읽은 배열"로 그대로 교체하면 된다.
    즉 이 파일 하나만 나중에 real_signal_reader.py 같은 걸로 갈아끼우면
    다른 모듈은 수정할 필요가 없도록 설계했다.

전제 (임의로 잡은 값, 실제 하드웨어 스펙 확정되면 조정):
    - 쿨링팬 회전속도: 3000 RPM = 50 Hz (초당 50회전)
    - 샘플링 레이트: 1000 Hz (나이퀴스트 기준 500Hz까지 관찰 가능, 50Hz의 10배수까지 여유 확보)
    - FFT 윈도우 크기: 256 포인트 (2의 거듭제곱 - ESP32 FFT 라이브러리 제약 고려)
"""

import numpy as np

# ---- 공통 파라미터 (전체 파이프라인이 공유) ----
SAMPLE_RATE = 1000        # Hz, 초당 샘플 수
WINDOW_SIZE = 256         # FFT 포인트 수 (2의 거듭제곱)
FAN_ROTATION_HZ = 50      # 정상 회전 주파수 (3000 RPM 기준)


def generate_normal_signal(duration_sec=1.0, noise_level=0.05, seed=None,
                            rpm_jitter_hz=0.8, amp_jitter=0.03):
    """
    정상 상태 진동 신호를 흉내낸다.

    실제로는 회전 주파수(기본파) + 약한 배음(harmonics) + 랜덤 노이즈로 구성된다.
    이게 "정상일 때 baseline 학습" 단계에서 쓰일 데이터를 대신한다.

    v2 수정 사항 (첫 버전의 문제점 보완):
      - 첫 버전은 seed를 바꿔도 회전 주파수가 항상 정확히 50.0Hz로 고정되어 있어서,
        peak_freq의 표준편차가 0으로 나오는 비현실적인 상황이 발생했다.
        (실제 모터는 부하·전압 변동으로 회전속도가 미세하게 흔들린다.)
      - rpm_jitter_hz: 회전 주파수 자체에 소량의 랜덤 흔들림을 추가해 현실성을 높임
      - amp_jitter: 진폭에도 소량의 개체차/순간 변동을 추가
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration_sec, 1 / SAMPLE_RATE)

    # 회전 주파수 자체의 미세한 흔들림 (부하 변동, 전압 변동 등을 흉내)
    actual_rotation_hz = FAN_ROTATION_HZ + rng.normal(0, rpm_jitter_hz)
    base_amp = 1.0 + rng.normal(0, amp_jitter)

    # 기본 회전 주파수 성분
    signal = base_amp * np.sin(2 * np.pi * actual_rotation_hz * t)

    # 약한 2차 배음 - 정상 상태에서도 약간은 존재 (진폭도 소량 변동)
    harmonic_amp = 0.15 + rng.normal(0, amp_jitter * 0.5)
    signal += harmonic_amp * np.sin(2 * np.pi * (2 * actual_rotation_hz) * t)

    # 랜덤 노이즈 (센서 자체 노이즈, 미세한 환경 진동 등)
    signal += rng.normal(0, noise_level, size=t.shape)

    return t, signal


def generate_faulty_signal(duration_sec=1.0, noise_level=0.05, fault_strength=0.8, seed=None):
    """
    인위적 고장 상태(예: 팬 날개 불균형, 베어링 마모)를 흉내낸 신호.

    실제 고장의 물리적 특징:
      - 특정 배음(harmonic)의 진폭이 비정상적으로 커짐
      - 고주파 대역에 추가적인 피크가 생김 (베어링 결함 주파수 등)

    fault_strength: 0에 가까울수록 경미한 고장, 1에 가까울수록 심각한 고장
    """
    t, signal = generate_normal_signal(duration_sec, noise_level, seed)

    # 3차 배음(약 150Hz 부근)이 비정상적으로 커짐 - 예: 날개 불균형을 흉내
    signal += fault_strength * np.sin(2 * np.pi * (3 * FAN_ROTATION_HZ) * t)

    # 베어링 결함을 흉내낸 고주파 성분 (약 320Hz 부근에 임의로 배치)
    signal += (fault_strength * 0.6) * np.sin(2 * np.pi * 320 * t)

    return t, signal


def generate_progressive_wear_sequence(n_windows=50, final_fault_strength=0.9, seed=None):
    """
    서서히 진행되는 마모를 흉내낸 시계열 시퀀스.

    엣지알리미 논리정리에서 강조한 "위험 정상화(Normalization of Deviance)" 문제
    -- 매우 서서히 나빠지면 어제와 오늘의 차이가 미미해서 AI가 계속 "정상"으로
    착각하는 현상 -- 을 재현하고, 이걸 탐지하는 로직(05번 파일)을 검증하기 위한 데이터.

    fault_strength를 0 -> final_fault_strength까지 선형으로 서서히 증가시킨다.
    """
    rng = np.random.default_rng(seed)
    signals = []
    strengths = np.linspace(0, final_fault_strength, n_windows)
    for i, strength in enumerate(strengths):
        window_seed = None if seed is None else int(rng.integers(0, 1_000_000))
        if strength < 0.01:
            _, sig = generate_normal_signal(seed=window_seed)
        else:
            _, sig = generate_faulty_signal(fault_strength=float(strength), seed=window_seed)
        signals.append(sig)
    return signals, strengths


if __name__ == "__main__":
    # 간단 확인용: 정상/고장 신호를 만들어서 기본 통계만 출력
    t_n, sig_n = generate_normal_signal(seed=1)
    t_f, sig_f = generate_faulty_signal(seed=1)

    print("=== 가짜 신호 생성 확인 ===")
    print(f"샘플링 레이트: {SAMPLE_RATE} Hz, 신호 길이: {len(sig_n)} 샘플")
    print(f"정상 신호  - RMS: {np.sqrt(np.mean(sig_n**2)):.4f}")
    print(f"고장 신호  - RMS: {np.sqrt(np.mean(sig_f**2)):.4f}")

    # v2 수정 확인: seed를 바꾸면 회전 주파수 자체가 미세하게 달라지는지 확인
    print("\n=== 회전주파수 지터 확인 (seed별로 달라야 정상) ===")
    for s in range(5):
        rng = np.random.default_rng(s)
        jittered = FAN_ROTATION_HZ + rng.normal(0, 0.8)
        print(f"seed={s}: 실제 회전주파수 근사값 = {jittered:.3f} Hz")

    print("\n다음 단계: feature_extraction.py 에서 이 신호로 FFT/특징추출 검증")
