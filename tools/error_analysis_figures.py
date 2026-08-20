"""
error_analysis_figures.py

목적:
    ESP32 이식 오차 분석("최상/최악 시나리오")을 실제 수치로 그려서 확인한다.
    전부 repo의 실제 코드(signal_generator / feature_extraction / baseline_detector)와
    C 구현을 그대로 옮긴 float32 파이프라인으로 계산한 값이다 — 개념도가 아님.

    생성 그림 (analysis/ 폴더):
      figE1_porting_error.png   — 파이썬(float64) vs C(float32) FFT 스펙트럼 + bin별 상대오차
                                   → "이식 수치 오차는 무시 가능" 주장의 근거
      figE2_resolution_error.png — 회전주파수 추정 오차가 배음 차수(1x/2x/3x)별로
                                   대역 에너지를 얼마나 놓치게 하는가
                                   → "해상도 오차가 3차 배음에서 증폭" 주장의 근거
      figE3_sensitivity.png     — 노이즈 수준(σ 팽창)별 고장 강도-탐지율 곡선
                                   → "최악 시나리오에서 민감도 경계 2~4배 상승" 주장의 근거

    사용법 (repo 루트에서):  python -m tools.error_analysis_figures
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

from src import signal_generator as gen
from src import feature_extraction as feat
from src.baseline_detector import BaselineDetector, ACTIVE_FEATURES

# ---- 한글 폰트 (Windows) ----
for name in ("Malgun Gothic", "NanumGothic"):
    if any(f.name == name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break
plt.rcParams["axes.unicode_minus"] = False

# ---- 팔레트 (검증된 기본 팔레트, 라이트 모드) ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE_AXIS = "#c3c2b7"
S1_BLUE = "#2a78d6"
S2_AQUA = "#1baf7a"
S3_YELLOW = "#eda100"
CRITICAL = "#d03b3b"   # 상태색 — '허용 한계선' 표시 전용

N = gen.WINDOW_SIZE
SR = float(gen.SAMPLE_RATE)
FREQS = np.arange(N // 2) * (SR / N)

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "analysis")
os.makedirs(OUT_DIR, exist_ok=True)


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE_AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.title.set_color(INK)
    ax.grid(True, color=GRID, linewidth=0.7, alpha=0.9)
    ax.set_axisbelow(True)


# ==================== C(float32) FFT — edgealimi.c 를 한 줄씩 그대로 옮긴 것 ====================
def c_fft_magnitudes_f32(signal):
    f32 = np.float32
    log2n = int(math.log2(N))
    re = np.zeros(N, dtype=f32)
    im = np.zeros(N, dtype=f32)
    for i in range(N):
        j = int(bin(i)[2:].zfill(log2n)[::-1], 2)   # bit_reverse
        re[j] = f32(signal[i])
    for stage in range(1, log2n + 1):
        m = 1 << stage
        half = m >> 1
        w_step_re = f32(math.cos(-2 * math.pi / m))
        w_step_im = f32(math.sin(-2 * math.pi / m))
        for block in range(0, N, m):
            w_re, w_im = f32(1), f32(0)
            for k in range(half):
                top, bot = block + k, block + k + half
                t_re = f32(w_re * re[bot] - w_im * im[bot])
                t_im = f32(w_re * im[bot] + w_im * re[bot])
                re[bot] = f32(re[top] - t_re)
                im[bot] = f32(im[top] - t_im)
                re[top] = f32(re[top] + t_re)
                im[top] = f32(im[top] + t_im)
                next_w_re = f32(w_re * w_step_re - w_im * w_step_im)
                w_im = f32(w_re * w_step_im + w_im * w_step_re)
                w_re = next_w_re
    norm = f32(2.0 / N)
    return np.sqrt(re[: N // 2] ** 2 + im[: N // 2] ** 2) * norm


# ==================== 그림 E1: 이식 수치 오차 ====================
def fig_porting_error():
    _, sig = gen.generate_faulty_signal(seed=99, fault_strength=0.5)
    sig = sig[:N].astype(np.float32)

    py_mag = np.abs(np.fft.fft(sig.astype(np.float64)))[: N // 2] * 2 / N
    c_mag = c_fft_magnitudes_f32(sig)
    rel_err = np.abs(c_mag - py_mag) / np.maximum(np.abs(py_mag), 1e-6) * 100

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.4, 6.4), height_ratios=[3, 2])
    fig.patch.set_facecolor(SURFACE)

    ax1.plot(FREQS, py_mag, color=S1_BLUE, linewidth=2, label="파이썬 (float64, numpy)")
    ax1.plot(FREQS, c_mag, color=S3_YELLOW, linewidth=2, linestyle=(0, (4, 3)),
             label="C 이식 (float32, 직접 구현 FFT)")
    ax1.set_title("이식 수치 오차 ① — 같은 신호(고장 강도 0.5)의 FFT 스펙트럼이 겹치는가", fontsize=11, loc="left")
    ax1.set_xlabel("주파수 (Hz)")
    ax1.set_ylabel("진폭 (정규화)")
    ax1.legend(frameon=False, fontsize=9, labelcolor=INK2)
    ax1.annotate("두 곡선이 완전히 겹침\n(점선이 실선 위에 정확히 포개짐)",
                 xy=(50, py_mag[FREQS.searchsorted(50)]), xytext=(150, py_mag.max() * 0.75),
                 fontsize=9, color=INK2,
                 arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8))
    style_axes(ax1)

    ax2.semilogy(FREQS, np.maximum(rel_err, 1e-7), color=S1_BLUE, linewidth=2)
    # Malgun Gothic에 수학용 마이너스 글리프가 없어 지수 표기가 깨짐 → 평문 표기로 대체
    from matplotlib.ticker import FuncFormatter, LogLocator
    ax2.yaxis.set_major_locator(LogLocator(base=10, numticks=6))
    ax2.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0e}".replace("e-0", "e-").replace("e+0", "e")))
    ax2.axhline(2.0, color=CRITICAL, linewidth=1.2, linestyle=(0, (4, 3)))
    ax2.text(FREQS[-1], 2.0, " 허용 한계 2%", va="bottom", ha="right", fontsize=9, color=CRITICAL)
    worst = rel_err.max()
    ax2.text(FREQS[-1], worst, f" 실측 최악 {worst:.4f}%", va="bottom", ha="right",
             fontsize=9, color=INK2)
    ax2.set_title("이식 수치 오차 ② — bin별 상대오차 (로그 스케일)", fontsize=11, loc="left")
    ax2.set_xlabel("주파수 (Hz)")
    ax2.set_ylabel("상대오차 (%)")
    ax2.set_ylim(1e-7, 30)
    style_axes(ax2)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "figE1_porting_error.png")
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    print(f"저장: {path}  (실측 최악 상대오차 {worst:.5f}%)")


# ==================== 그림 E2: 해상도 오차의 배음 증폭 ====================
def fig_resolution_error():
    # 통제된 신호: 1x/2x/3x 배음이 전부 들어있는 고장 신호 (지터 없이 순수 50Hz)
    t = np.arange(N) / SR
    rng = np.random.default_rng(7)
    est_errors = np.linspace(-3.0, 3.0, 61)   # 회전주파수 추정 오차 (Hz)
    n_avg = 30                                # 노이즈 평균화

    captured = {1: np.zeros_like(est_errors), 2: np.zeros_like(est_errors), 3: np.zeros_like(est_errors)}
    for _ in range(n_avg):
        sig = (np.sin(2 * np.pi * 50 * t)
               + 0.4 * np.sin(2 * np.pi * 100 * t)
               + 0.4 * np.sin(2 * np.pi * 150 * t)
               + rng.normal(0, 0.05, N))
        mag = np.abs(np.fft.fft(sig))[: N // 2] * 2 / N
        for k in (1, 2, 3):
            true_e = feat.frequency_band_energy(FREQS, mag, 50.0 * k)
            for i, e in enumerate(est_errors):
                got = feat.frequency_band_energy(FREQS, mag, (50.0 + e) * k)
                captured[k][i] += (got / true_e) / n_avg

    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    fig.patch.set_facecolor(SURFACE)

    series = [(1, S1_BLUE, "1× (harmonic1 — 불평형)"),
              (2, S2_AQUA, "2× (harmonic2 — 정렬불량)"),
              (3, S3_YELLOW, "3× (harmonic3 — 이완)")]
    for k, color, label in series:
        ax.plot(est_errors, captured[k] * 100, color=color, linewidth=2, label=label)
        ax.text(est_errors[-1] + 0.05, captured[k][-1] * 100, f"{k}×",
                color=INK2, fontsize=9, va="center")

    # FFT 해상도 한계(±반 bin = ±1.95Hz) 영역 표시
    half_bin = (SR / N) / 2
    ax.axvspan(-half_bin, half_bin, color=GRID, alpha=0.5, zorder=0)
    ax.text(0, 8, f"FFT 해상도 한계\n±{half_bin:.2f}Hz (반 bin)",
            ha="center", fontsize=9, color=INK2)

    # 해상도 한계 내(±반 bin)에서 각 배음의 최악 포착률
    in_limit = np.abs(est_errors) <= half_bin + 1e-9
    worst3 = captured[3][in_limit].min() * 100
    idx_worst = np.where(in_limit)[0][np.argmin(captured[3][in_limit])]
    ax.annotate(f"해상도 한계 내 최악: 3× 대역 {100 - worst3:.0f}% 손실",
                xy=(est_errors[idx_worst], worst3), xytext=(-2.9, 40), fontsize=9, color=INK2,
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8))

    ax.set_title("해상도 오차의 배음 증폭 — 회전수 추정이 틀리면 상위 배음 대역이 먼저 무너진다",
                 fontsize=11, loc="left")
    ax.set_xlabel("회전주파수 추정 오차 (Hz)")
    ax.set_ylabel("대역 에너지 포착률 (%)")
    ymax = max(captured[k].max() for k in (1, 2, 3)) * 100
    ax.set_ylim(0, ymax + 8)   # 클리핑 방지 (100% 초과 = 어긋난 대역이 인접 누설 bin을 주워담는 효과)
    ax.text(0.02, 0.97, "포착률 100% 초과 구간은 어긋난 대역이 인접 누설 bin을 포함하는 효과",
            transform=ax.transAxes, fontsize=8, color=MUTED, va="top")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="lower left")
    style_axes(ax)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "figE2_resolution_error.png")
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    print(f"저장: {path}")


# ==================== 그림 E3: σ 팽창에 따른 민감도 열화 ====================
def fig_sensitivity():
    strengths = np.array([0.01, 0.02, 0.03, 0.05, 0.08, 0.12, 0.2, 0.3])
    scenarios = [
        (0.05, S1_BLUE, "최상: 노이즈 0.05 (시뮬레이션 기준)"),
        (0.10, S2_AQUA, "중간: 노이즈 2배 (현실적 설치)"),
        (0.20, S3_YELLOW, "최악: 노이즈 4배 (부착 불량·외부 진동)"),
    ]
    n_trials = 60

    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    fig.patch.set_facecolor(SURFACE)

    for noise, color, label in scenarios:
        # 각 시나리오의 노이즈 수준으로 baseline을 학습 → σ가 그만큼 팽창
        feats = []
        for i in range(60):
            _, sig = gen.generate_normal_signal(noise_level=noise, seed=i)
            feats.append(feat.extract_features(sig[:N]))
        det = BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                               features_to_check=ACTIVE_FEATURES)
        det.fit(feats)

        rates = []
        for s in strengths:
            hit = 0
            for i in range(n_trials):
                det.reset_confirmation_state()
                _, sig = gen.generate_faulty_signal(noise_level=noise, seed=3000 + i,
                                                    fault_strength=float(s))
                if det.judge(feat.extract_features(sig[:N]))["is_suspect"]:
                    hit += 1
            rates.append(hit / n_trials * 100)
        ax.plot(strengths, rates, color=color, linewidth=2, marker="o", markersize=5,
                label=label)

    ax.axhline(98, color=BASELINE_AXIS, linewidth=1, linestyle=(0, (4, 3)))
    ax.text(strengths[0], 98, "실용 기준 98%", va="bottom", ha="left", fontsize=9, color=MUTED)
    ax.set_xscale("log")
    ax.set_xticks(strengths)
    ax.set_xticklabels([f"{s:g}" for s in strengths])
    ax.set_title("σ 팽창에 따른 민감도 열화 — 노이즈가 커질수록 탐지 가능한 최소 고장 강도가 밀려난다",
                 fontsize=11, loc="left")
    ax.set_xlabel("고장 강도 (fault_strength)")
    ax.set_ylabel("탐지율 (%, 1차 의심 기준)")
    ax.set_ylim(-4, 112)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc="lower right")
    style_axes(ax)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "figE3_sensitivity.png")
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    print(f"저장: {path}")


if __name__ == "__main__":
    fig_porting_error()
    fig_resolution_error()
    fig_sensitivity()
    print("\n3개 그림 생성 완료 — analysis/ 폴더 확인")
