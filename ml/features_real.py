"""
features_real.py

목적:
    src/feature_extraction.py의 특징추출 로직을 "실데이터"에 적용하기 위한 어댑터.

    기존 extract_features()는 시뮬레이션 전제(1kHz, 회전수 탐색범위 30~70Hz,
    high_freq 대역 250~400Hz)가 함수 내부에 하드코딩돼 있다. 실데이터는
    샘플레이트(12kHz/50kHz)와 회전수(12~61Hz)가 제각각이므로, 같은 '특징 정의'를
    유지하되 기준축(회전주파수)과 대역만 파라미터로 받도록 확장한다.

    특징 정의 자체(RMS, 1x/2x/3x 하모닉 대역 에너지, 고주파 에너지)는 기존과
    동일한 함수(compute_fft, frequency_band_energy, estimate_rotation_hz)를
    재사용한다 - 검증된 로직을 복제하지 않기 위함.

kurtosis 재검토:
    시뮬레이션 단계에서는 "임펄스성 신호가 아니라 판별력 기대 어려움"으로
    ACTIVE_FEATURES에서 제외했었다(재검토 조건: 실측 데이터로 판별력 확인).
    CWRU는 베어링 결함 = 임펄스성 신호이므로, 여기서는 특징 벡터에 포함시켜
    실데이터 기준 판별력을 실제로 측정한다. (포함 여부 결론은 학습 결과가 말해줌)

스케일 정규화:
    시뮬레이션과 달리 실데이터는 센서 감도/운전조건에 따라 절대 진폭이 제각각이다.
    하모닉 에너지를 그대로 쓰면 "진폭이 큰 파일"만 학습하게 되므로,
    하모닉/고주파 에너지는 전체 스펙트럼 에너지에 대한 '비율' 버전도 함께 만든다.
    (ESP32에서도 나눗셈 한 번이면 되므로 이식성 훼손 없음)
"""

import numpy as np
from scipy.stats import kurtosis

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.feature_extraction import (  # noqa: E402
    compute_fft, frequency_band_energy, estimate_rotation_hz,
)

# 분류/탐지에 사용할 특징 이름 (벡터 순서 고정 - C 포팅 시에도 이 순서를 따름)
FEATURE_NAMES = [
    "rms",
    "kurtosis",
    "harmonic1_ratio",
    "harmonic2_ratio",
    "harmonic3_ratio",
    "high_freq_ratio",
]


def extract_features_real(signal, sample_rate, nominal_rotation_hz,
                          high_freq_band=None):
    """
    실데이터 윈도우 1개 -> 특징 dict.

    nominal_rotation_hz: 명목 회전주파수 (CWRU=RPM/60, MaFaulDa=파일명).
        실제 회전수는 이 값에서 약간 어긋나므로, 기존 estimate_rotation_hz()로
        명목값 ±20% 범위 안에서 실제 피크를 다시 찾는다 - "회전수는 모든 분석의
        기준축" 원칙을 실데이터에서도 유지.
    high_freq_band: (low, high) Hz. None이면 회전 하모닉 대역(~10x)을 벗어난
        구간부터 나이퀴스트의 80%까지를 고주파 대역으로 잡는다.
    """
    signal = signal - np.mean(signal)  # DC(센서 오프셋) 제거
    rms = float(np.sqrt(np.mean(signal ** 2)))
    kurt = float(kurtosis(signal))

    freqs, mags = compute_fft(signal, sample_rate)

    search = (nominal_rotation_hz * 0.8, nominal_rotation_hz * 1.2)
    rot_hz = estimate_rotation_hz(freqs, mags, search_range_hz=search)
    if rot_hz is None:
        rot_hz = nominal_rotation_hz

    # 대역폭: FFT 해상도의 2배 이상, 최소 5Hz (기존 설계와 동일한 취지)
    resolution = sample_rate / len(signal)
    band = max(5.0, 2.0 * resolution)

    h1 = frequency_band_energy(freqs, mags, rot_hz * 1, band)
    h2 = frequency_band_energy(freqs, mags, rot_hz * 2, band)
    h3 = frequency_band_energy(freqs, mags, rot_hz * 3, band)

    if high_freq_band is None:
        high_freq_band = (rot_hz * 10, sample_rate / 2 * 0.8)
    hf = float(np.sum(mags[(freqs >= high_freq_band[0]) & (freqs <= high_freq_band[1])]))

    total = float(np.sum(mags[freqs > 1.0])) + 1e-12  # DC 제외 전체 스펙트럼 에너지

    return {
        "rms": rms,
        "kurtosis": kurt,
        "rotation_hz_estimate": rot_hz,
        "harmonic1_energy": h1,
        "harmonic2_energy": h2,
        "harmonic3_energy": h3,
        "high_freq_energy": hf,
        "harmonic1_ratio": h1 / total,
        "harmonic2_ratio": h2 / total,
        "harmonic3_ratio": h3 / total,
        "high_freq_ratio": hf / total,
    }


def windows_to_matrix(windows, feature_names=FEATURE_NAMES):
    """Window 리스트 -> (X, y, groups, 전체 특징 dict 리스트)."""
    X, y, groups, dicts = [], [], [], []
    for w in windows:
        f = extract_features_real(w.signal, w.sample_rate, w.rotation_hz)
        X.append([f[name] for name in feature_names])
        y.append(w.label)
        groups.append(w.group)
        dicts.append(f)
    return np.array(X), np.array(y), np.array(groups), dicts
