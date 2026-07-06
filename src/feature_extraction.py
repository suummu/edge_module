"""
feature_extraction.py

목적:
    signal_generator.py에서 만든 (또는 나중에 실제 센서에서 읽은) 시간 영역 신호를 받아서:
    1) FFT로 주파수 영역으로 변환
    2) 핵심 특징값(RMS, 첨도, 배음 에너지 등) 추출

    이 파일의 함수들은 입력이 "가짜 신호"든 "실제 센서 신호"든 상관없이 동작한다.
    즉 signal_generator.py만 실제 데이터 리더로 교체되면, 이 파일은 그대로 재사용 가능.

주의(실제 ESP32 이식 시 고려할 점, 지금은 파이썬으로만 검증):
    - np.fft, scipy.stats 같은 라이브러리는 C/ESP32에 없음
    - RMS, 첨도는 결국 for 루프 기반 직접 계산으로 재구현해야 함
    - 여기서는 "어떤 값이 왜 필요한지"를 파이썬으로 먼저 확정하는 것이 목적
"""

import numpy as np
from scipy.stats import kurtosis
from scipy.fft import fft, fftfreq

from . import signal_generator as gen

SAMPLE_RATE = gen.SAMPLE_RATE
WINDOW_SIZE = gen.WINDOW_SIZE


def compute_fft(signal, sample_rate=SAMPLE_RATE):
    """
    시간 영역 신호 -> 주파수 영역 변환.

    반환값:
        freqs: 각 주파수 성분의 실제 Hz 값 (양의 주파수만)
        magnitudes: 각 주파수 성분의 크기(진폭)
    """
    n = len(signal)
    fft_result = fft(signal)
    freqs = fftfreq(n, d=1 / sample_rate)

    # 실수 신호의 FFT는 대칭이므로 양의 주파수 절반만 사용
    half = n // 2
    freqs = freqs[:half]
    magnitudes = np.abs(fft_result[:half]) * 2 / n  # 정규화

    return freqs, magnitudes


def frequency_band_energy(freqs, magnitudes, center_hz, band_width_hz=5.0):
    """
    특정 주파수 대역(center_hz ± band_width_hz)의 에너지 합을 구한다.

    왜 필요한가:
        FFT 주파수 해상도(bin 간격)는 sample_rate / window_size로 정해진다.
        지금 설정(1000Hz / 256포인트)은 해상도가 약 3.9Hz라, "정확히 몇 Hz에서
        피크가 떴는가"라는 단일 값(peak_freq)은 회전속도가 조금만 흔들려도
        같은 bin에 계속 걸리거나 옆 bin으로 튀는 등 불안정할 수 있다.
        따라서 실제 판별에는 "이 대역 전체에 에너지가 얼마나 실렸는가"를
        같이 보는 것이 더 안정적이다. (엣지알리미 논리정리의
        "절대 위치가 아닌 상대적 위치/배수 판별" 원칙과 같은 맥락)
    """
    idx = (freqs >= center_hz - band_width_hz) & (freqs <= center_hz + band_width_hz)
    if not np.any(idx):
        return 0.0
    return float(np.sum(magnitudes[idx]))


def extract_features(signal, sample_rate=SAMPLE_RATE, rotation_hz_estimate=None):
    """
    하나의 신호 구간(윈도우)에서 핵심 특징값을 뽑아낸다.

    v2 수정 사항 (첫 버전의 문제점 보완):
      - 첫 버전은 peak_freq(FFT에서 가장 큰 단일 bin의 위치)만 특징으로 썼는데,
        FFT 해상도 한계 때문에 이 값의 변동성이 거의 0으로 나와 판별에 기여를
        못했다. 대신 2차/3차 배음 "대역 에너지"를 특징으로 추가해서, 고장 시
        3차 배음이 커지는 패턴을 더 안정적으로 잡아낸다.
      - rotation_hz_estimate: 현재 추정 회전 주파수. None이면 기본 FAN_ROTATION_HZ 사용.
        (실제 배포 시에는 회전계 센서나 1차 피크로 실시간 추정해서 넣어야 함 -> TODO)

    반환하는 특징:
        rms: 신호 유효값 (에너지 크기, 고장 시 대체로 증가)
        kurtosis_val: 첨도 (신호가 얼마나 뾰족한지, 베어링 결함 시 증가하는 경향)
        peak_freq: FFT에서 가장 큰 피크가 나타난 주파수 (참고용, 단독 판별엔 부적합)
        peak_magnitude: 그 피크의 크기
        harmonic2_energy: 2차 배음(회전주파수 x2) 대역 에너지
        harmonic3_energy: 3차 배음(회전주파수 x3) 대역 에너지 - 불균형 고장의 주 지표
        high_freq_energy: 250~400Hz 고주파 대역 에너지 - 베어링 결함류 지표
    """
    rms = float(np.sqrt(np.mean(signal ** 2)))
    kurtosis_val = float(kurtosis(signal))  # 정규분포 기준 0 (scipy는 excess kurtosis 반환)

    freqs, magnitudes = compute_fft(signal, sample_rate)

    # DC 성분(0Hz)은 회전 신호와 무관하므로 제외하고 피크 탐색
    valid_idx = freqs > 1.0
    peak_idx = np.argmax(magnitudes[valid_idx])
    peak_freq = float(freqs[valid_idx][peak_idx])
    peak_magnitude = float(magnitudes[valid_idx][peak_idx])

    base_hz = rotation_hz_estimate if rotation_hz_estimate is not None else gen.FAN_ROTATION_HZ
    harmonic2_energy = frequency_band_energy(freqs, magnitudes, base_hz * 2)
    harmonic3_energy = frequency_band_energy(freqs, magnitudes, base_hz * 3)
    high_freq_energy = float(np.sum(magnitudes[(freqs >= 250) & (freqs <= 400)]))

    return {
        "rms": rms,
        "kurtosis": kurtosis_val,
        "peak_freq": peak_freq,
        "peak_magnitude": peak_magnitude,
        "harmonic2_energy": harmonic2_energy,
        "harmonic3_energy": harmonic3_energy,
        "high_freq_energy": high_freq_energy,
    }


if __name__ == "__main__":
    # 01번 신호 생성기로 정상/고장 신호를 만들어 특징값 비교
    t_n, sig_n = gen.generate_normal_signal(seed=1)
    t_f, sig_f = gen.generate_faulty_signal(seed=1)

    # WINDOW_SIZE(256)만큼만 잘라서 사용 (실제 ESP32에서도 윈도우 단위로 처리)
    features_normal = extract_features(sig_n[:WINDOW_SIZE])
    features_faulty = extract_features(sig_f[:WINDOW_SIZE])

    print("=== 특징 추출 비교 (정상 vs 고장) ===")
    print(f"{'특징':<15}{'정상':>12}{'고장':>12}{'변화':>12}")
    for key in features_normal:
        n_val = features_normal[key]
        f_val = features_faulty[key]
        change = f_val - n_val
        print(f"{key:<15}{n_val:>12.4f}{f_val:>12.4f}{change:>+12.4f}")

    print("\n다음 단계: baseline_detector.py 에서 이 특징값으로 판별 로직 검증")
