"""
feature_extraction_v2.py — 재설계 파이프라인 (v2) 파이썬 정답 구현

C 코어(edge_module_c/core)와 "같은 입력 → 같은 출력"이 되도록
연산 순서·bin 인덱싱까지 C와 동일하게 맞춘 float64 기준 구현.
(C 는 float32 라 미세 오차만 존재해야 하며, 이를 validate_v2_c 로 대조)

v1 대비 변경 (각각이 해결하는 문제):
  1) Hanning 창 (C 코어에 기존재)  → 스펙트럼 누설
  2) 1024pt FFT (C 코어에 기존재)  → bin 0.977Hz, 대역 에너지 분산 감소
  3) 포물선 피크 보간 (C 기존재)    → 회전수 sub-bin 정밀도
  4) 비율 특징 (신규)              → 검증(ml/)·실장(C) 통일 + 진폭 불변성
  5) log(x+1e-6) 공간 3σ (신규)   → 에너지 분포 오른쪽 꼬리 오탐 해결
  6) HF 상한 255Hz (신규)         → 센서 DLPF(260Hz) 감쇠 구간 배제
"""

import numpy as np

SAMPLE_RATE = 1000.0
FFT_SIZE = 1024
NUM_BINS = FFT_SIZE // 2
BIN_HZ = SAMPLE_RATE / FFT_SIZE            # 0.9765625 Hz

HARMONIC_BW_HZ = 5.0 + 2.0 * BIN_HZ        # 물리 허용오차 5Hz + Hann 주엽 2bin
HF_CUTOFF_RATIO = 4.5
SENSOR_BW_HZ = 260.0                       # MPU-6050 DLPF=0 가속도 대역
LOG_EPS = 1e-6

FEATURE_NAMES = ["rms", "harmonic1_ratio", "harmonic2_ratio",
                 "harmonic3_ratio", "high_freq_ratio"]

# C: w = 0.5*(1 - cos(2*pi*i/(n-1)))  — 대칭(symmetric) Hann
_HANN = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(FFT_SIZE) / (FFT_SIZE - 1)))
_HANN_COHERENT_GAIN = 0.5


def fft_spectrum(signal):
    """C em_fft_spectrum 과 동일: 평균 제거 → RMS → Hann → FFT → 진폭 스펙트럼.

    반환: (rms, mag[NUM_BINS])
    """
    sig = np.asarray(signal, dtype=np.float64)
    assert len(sig) == FFT_SIZE
    sig = sig - sig.mean()
    rms = float(np.sqrt(np.mean(sig ** 2)))
    spec = np.fft.rfft(sig * _HANN)[:NUM_BINS]
    mag = np.abs(spec) * (2.0 / FFT_SIZE) / _HANN_COHERENT_GAIN
    mag[0] *= 0.5
    return rms, mag


def band_energy(mag, lo_hz, hi_hz):
    """C band_energy 와 동일한 bin 인덱싱 (floor/ceil, 양끝 포함)."""
    lo = int(np.floor(lo_hz / BIN_HZ))
    hi = int(np.ceil(hi_hz / BIN_HZ))
    lo = max(lo, 1)
    hi = min(hi, NUM_BINS - 1)
    if hi < lo:
        return 0.0
    return float(np.sum(mag[lo:hi + 1] ** 2))


def rotation_measure(mag, rated_hz, lo_ratio=0.6, hi_ratio=1.4):
    """C em_rotation_measure 와 동일: 탐색범위 최대 피크 + 포물선 보간 + SNR.

    반환: (rotation_hz, snr)
    """
    lo = int(np.floor(rated_hz * lo_ratio / BIN_HZ))
    hi = int(np.ceil(rated_hz * hi_ratio / BIN_HZ))
    lo = max(lo, 1)
    hi = min(hi, NUM_BINS - 2)
    if hi <= lo + 4:
        return rated_hz, 0.0

    pk = lo + int(np.argmax(mag[lo:hi + 1]))

    mask = np.ones(hi - lo + 1, dtype=bool)
    for k in range(pk - 2, pk + 3):
        if lo <= k <= hi:
            mask[k - lo] = False
    floor_bins = mag[lo:hi + 1][mask]
    floor_avg = float(np.mean(floor_bins)) if len(floor_bins) else 0.0
    snr = float(mag[pk] / floor_avg) if floor_avg > 1e-20 else 0.0

    y0, y1, y2 = mag[pk - 1], mag[pk], mag[pk + 1]
    denom = y0 - 2.0 * y1 + y2
    delta = 0.0
    if abs(denom) > 1e-12:
        delta = float(np.clip(0.5 * (y0 - y2) / denom, -0.5, 0.5))
    return (pk + delta) * BIN_HZ, snr


def extract_features(mag, rotation_hz, rms, rated_hz):
    """C em_extract_features 와 동일: rms + 비율 특징 4종."""
    f1 = rotation_hz if rotation_hz > 0.0 else rated_hz
    bw = HARMONIC_BW_HZ

    e_total = float(np.sum(mag[1:] ** 2))
    if e_total < 1e-20:
        e_total = 1e-20

    hf_lo = rated_hz * HF_CUTOFF_RATIO
    hf_hi = min(SAMPLE_RATE * 0.5, SENSOR_BW_HZ - 5.0)

    return np.array([
        rms,
        band_energy(mag, 1 * f1 - bw, 1 * f1 + bw) / e_total,
        band_energy(mag, 2 * f1 - bw, 2 * f1 + bw) / e_total,
        band_energy(mag, 3 * f1 - bw, 3 * f1 + bw) / e_total,
        band_energy(mag, hf_lo, hf_hi) / e_total,
    ])


def to_z_space(x):
    """C em_to_z_space 와 동일: 3σ 판정 전 log(x + 1e-6) 변환."""
    return np.log(np.maximum(np.asarray(x, dtype=np.float64), 0.0) + LOG_EPS)


def process_window(signal, rated_hz=50.0):
    """윈도우 1개 전체 처리 (트래커 없는 단발). 반환 dict."""
    rms, mag = fft_spectrum(signal)
    rot, snr = rotation_measure(mag, rated_hz)
    feats = extract_features(mag, rot, rms, rated_hz)
    return {"rms": rms, "rotation_hz": rot, "snr": snr,
            "features": feats, "z_space": to_z_space(feats)}
