"""v2 파이프라인 수치 검증 — 해석해(수학적 정답)와 직접 대조.

구조: C 코어와 동일한 1024pt Hann FFT 기준 (src/feature_extraction_v2.py).
실행: repo 루트에서 `python tests/validate_v2.py`
"""
import sys, pathlib
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from src.feature_extraction_v2 import (
    fft_spectrum, rotation_measure, band_energy, process_window, to_z_space,
    FFT_SIZE, SAMPLE_RATE, BIN_HZ, HARMONIC_BW_HZ)

rng = np.random.default_rng(42)
t = np.arange(FFT_SIZE) / SAMPLE_RATE
results = []

def check(name, value, expect, tol, unit=""):
    ok = abs(value - expect) <= tol
    results.append(ok)
    print(f"  {'PASS' if ok else 'FAIL'} {name}: 측정 {value:.6g}{unit} / 정답 {expect:.6g}{unit} (허용 ±{tol:g})")

# ── 검증 1: 창함수 진폭 복원 (누설 0 주파수 = 정확히 52주기) ─────────────
print("[1] Hanning 창 진폭 보정 — 순음 A=0.10g @ 50.78125Hz (bin 정중앙) + 중력 DC")
f_exact = 52 * BIN_HZ
sig = 0.10 * np.sin(2 * np.pi * f_exact * t) + 0.98
rms, mag = fft_spectrum(sig)
check("피크 진폭 복원", mag[52], 0.10, 0.002, "g")
check("RMS (= A/√2)", rms, 0.10 / np.sqrt(2), 1e-4, "g")
check("DC 제거 후 DC bin", mag[0], 0.0, 1e-6, "g")

# ── 검증 2: 누설 최악 조건 — bin 경계 정중앙에서 대역 에너지 포획률 ─────
print("[2] 누설 최악 조건 — 순음 @ bin 51.5 경계 정중앙 (50.29Hz)")
f_worst = 51.5 * BIN_HZ
sig = 0.10 * np.sin(2 * np.pi * f_worst * t)
_, mag = fft_spectrum(sig)
e_total = float(np.sum(mag[1:] ** 2))
cap_hann = band_energy(mag, f_worst - HARMONIC_BW_HZ, f_worst + HARMONIC_BW_HZ) / e_total
seg = sig - sig.mean()
mag_rect = np.abs(np.fft.rfft(seg)[:FFT_SIZE // 2]) * 2 / FFT_SIZE
lo, hi = int((f_worst - HARMONIC_BW_HZ) / BIN_HZ), int(np.ceil((f_worst + HARMONIC_BW_HZ) / BIN_HZ))
cap_rect = float(np.sum(mag_rect[lo:hi+1] ** 2) / np.sum(mag_rect[1:] ** 2))
print(f"  1×대역(±{HARMONIC_BW_HZ:.1f}Hz) 에너지 포획률: 창 없음 {cap_rect*100:.1f}% → Hanning {cap_hann*100:.1f}%")
results.append(cap_hann > 0.99)
print(f"  {'PASS' if cap_hann > 0.99 else 'FAIL'} Hanning 포획률 > 99%")

# ── 검증 3: 포물선 보간 정확도 — 임의 주파수 20점 스윕 ────────────────────
print("[3] 회전수 추정 정확도 — 40~65Hz 무작위 20개 주파수 (노이즈 포함)")
errs_interp, errs_bin = [], []
for f_true in rng.uniform(40, 65, 20):
    sig = 0.05 * np.sin(2 * np.pi * f_true * t) + 0.01 * rng.standard_normal(FFT_SIZE)
    _, mag = fft_spectrum(sig)
    est, _ = rotation_measure(mag, 50.0)
    errs_interp.append(abs(est - f_true))
    lo, hi = int(30 / BIN_HZ), int(70 / BIN_HZ)
    k = lo + int(np.argmax(mag[lo:hi+1]))
    errs_bin.append(abs(k * BIN_HZ - f_true))
print(f"  보간 없음: 평균오차 {np.mean(errs_bin):.3f}Hz / 최대 {np.max(errs_bin):.3f}Hz")
print(f"  보간 적용: 평균오차 {np.mean(errs_interp):.3f}Hz / 최대 {np.max(errs_interp):.3f}Hz")
results.append(np.max(errs_interp) < 0.15)
print(f"  {'PASS' if np.max(errs_interp) < 0.15 else 'FAIL'} 보간 최대오차 < 0.15Hz")

# ── 검증 4: 비율 특징의 진폭 불변성 (회전수↑ → 전체 진동↑ 시나리오) ─────
print("[4] 비율 특징 불변성 — 같은 신호를 진폭 2.5배로 (속도 변화 모사)")
base = (0.05 * np.sin(2*np.pi*50*t) + 0.01 * np.sin(2*np.pi*100*t)
        + 0.005 * rng.standard_normal(FFT_SIZE))
fA = process_window(base)["features"]
fB = process_window(base * 2.5)["features"]
dev_ratio = abs(fB[1] / fA[1] - 1)
print(f"  h1_ratio 변화율: {dev_ratio*100:.2f}%  (절대 특징이었다면 +525%)")
print(f"  rms 변화율: {(fB[0]/fA[0]-1)*100:.1f}% (rms는 절대량 유지 — 가동/정지·절대한계용)")
results.append(dev_ratio < 0.01)
print(f"  {'PASS' if dev_ratio < 0.01 else 'FAIL'} 비율 특징 변화 < 1%")

# ── 검증 5: log + 3σ 오탐률 — 오른쪽 꼬리 분포에서 ────────────────────────
print("[5] 3σ 상단 오탐률 — 정상 대역에너지 분포(χ², 오른쪽 긴 꼬리) 100만 표본")
normal_energy = 0.02 * rng.chisquare(4, 1_000_000) / 4
lin_fp = np.mean(normal_energy > normal_energy.mean() + 3*normal_energy.std())
z = to_z_space(normal_energy)
log_fp = np.mean(z > z.mean() + 3*z.std())
print(f"  선형 3σ 오탐률: {lin_fp*100:.3f}%   log 변환 후 3σ: {log_fp*100:.3f}%")
results.append(log_fp < lin_fp)
print(f"  {'PASS' if log_fp < lin_fp else 'FAIL'} log 변환이 오탐률 감소")

# ── 검증 6: 판별력 — 정상 vs 불평형(1× 성분 3배) 분리도 ─────────────────
print("[6] 판별력 — 정상 vs 불평형, log 공간 h1_ratio 분리도 (각 60윈도)")
zs_n, zs_f = [], []
for i in range(60):
    r = np.random.default_rng(i)
    ph = r.uniform(0, 6.28)
    nrm = (0.03*np.sin(2*np.pi*50*t + ph) + 0.008*np.sin(2*np.pi*100*t)
           + 0.009 * r.standard_normal(FFT_SIZE))
    flt = (0.09*np.sin(2*np.pi*50*t + ph) + 0.008*np.sin(2*np.pi*100*t)
           + 0.009 * r.standard_normal(FFT_SIZE))
    zs_n.append(float(to_z_space(process_window(nrm)["features"][1])))
    zs_f.append(float(to_z_space(process_window(flt)["features"][1])))
zs_n, zs_f = np.array(zs_n), np.array(zs_f)
sep = (zs_f.mean() - zs_n.mean()) / zs_n.std()
print(f"  불평형 이동량: 정상 σ의 {sep:.1f}배 (3σ 임계 대비 여유 {sep/3:.1f}×)")
results.append(sep > 3)
print(f"  {'PASS' if sep > 3 else 'FAIL'} 분리도 > 3σ")

print(f"\n===== 결과: PASS {sum(results)} / {len(results)} =====")
sys.exit(0 if all(results) else 1)
