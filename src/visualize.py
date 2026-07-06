"""
visualize.py

목적:
    signal_generator / feature_extraction / baseline_detector / reference_comparison
    로직이 만들어내는 결과를 그래프로 눈에 보이게 확인한다.
    발표 자료(슬라이드)에 넣을 근거 그림을 여기서 먼저 뽑아볼 수 있다.

    생성하는 그림:
        fig1_fft_spectrum.png       정상 vs 고장 신호의 주파수 스펙트럼 비교
        fig2_feature_comparison.png  정상/경미/심각 고장의 특징값 막대 비교
        fig3_progressive_wear.png   서서히 나빠지는 마모 과정에서 특징값 추이
        fig4_detection_confusion.png 고장 강도별 탐지 성공률 막대그래프

주의: 전부 가짜 신호 기반 그래프이므로, 발표에서 쓸 때는 반드시
"시뮬레이션 신호 기준"이라고 명시해야 한다 (스펙 원칙: 없는 걸 있는 것처럼 꾸미지 않기).

주의(폰트): 한글 폰트 경로는 OS/환경마다 다르다. 아래 _KOREAN_FONT_CANDIDATES에
없는 환경에서는 한글이 네모(□)로 깨질 수 있으니, 실행 환경에 맞는 경로를 추가할 것.
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

from . import signal_generator as gen
from . import feature_extraction as feat
from . import baseline_detector as det

# 한글 깨짐 방지: 환경별로 흔히 설치되는 CJK 폰트 경로 후보를 순서대로 시도한다.
_KOREAN_FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",       # Linux (Noto CJK)
    "/usr/share/fonts/truetype/noto/NotoSansCJKkr-Regular.otf",     # Linux (배포판별 변형)
    "C:/Windows/Fonts/malgun.ttf",                                   # Windows (맑은 고딕)
    "/System/Library/Fonts/AppleSDGothicNeo.ttc",                    # macOS
]

_font_name = None
for _path in _KOREAN_FONT_CANDIDATES:
    if os.path.exists(_path):
        fm.fontManager.addfont(_path)
        _font_name = fm.FontProperties(fname=_path).get_name()
        break

if _font_name:
    plt.rcParams["font.family"] = _font_name
else:
    print("[경고] 한글 폰트를 찾지 못했습니다. 그래프의 한글이 깨질 수 있습니다. "
          "_KOREAN_FONT_CANDIDATES에 현재 환경의 한글 폰트 경로를 추가하세요.")
plt.rcParams["axes.unicode_minus"] = False

WINDOW_SIZE = gen.WINDOW_SIZE
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
os.makedirs(OUT_DIR, exist_ok=True)

# 크림/차콜 계열 색상 (프로젝트 슬라이드 디자인 토큰과 통일)
COLOR_INK = "#1A1A1A"
COLOR_MUTED = "#6B6358"
COLOR_ACCENT = "#C8553D"
COLOR_BG = "#FAF7F0"
COLOR_SURFACE = "#F5F1E8"


def fig1_fft_spectrum():
    _, sig_n = gen.generate_normal_signal(seed=1)
    _, sig_f = gen.generate_faulty_signal(seed=1, fault_strength=0.9)

    freqs_n, mag_n = feat.compute_fft(sig_n[:WINDOW_SIZE])
    freqs_f, mag_f = feat.compute_fft(sig_f[:WINDOW_SIZE])

    fig, ax = plt.subplots(figsize=(9, 5), facecolor=COLOR_BG)
    ax.set_facecolor(COLOR_BG)
    ax.plot(freqs_n, mag_n, color=COLOR_MUTED, linewidth=1.8, label="정상 신호")
    ax.plot(freqs_f, mag_f, color=COLOR_ACCENT, linewidth=1.8, label="고장 신호 (fault_strength=0.9)")
    ax.set_xlim(0, 400)
    ax.set_xlabel("주파수 (Hz)", color=COLOR_INK)
    ax.set_ylabel("진폭", color=COLOR_INK)
    ax.set_title("정상 vs 고장 신호의 주파수 스펙트럼 (시뮬레이션)", color=COLOR_INK, fontsize=14, fontweight="bold")
    ax.tick_params(colors=COLOR_INK)
    for spine in ax.spines.values():
        spine.set_color(COLOR_MUTED)
    ax.legend(frameon=False, labelcolor=COLOR_INK)
    ax.annotate("3차 배음 (150Hz)\n고장 시 급증", xy=(150, mag_f[np.argmin(np.abs(freqs_f-150))]),
                xytext=(190, mag_f.max()*0.7), color=COLOR_ACCENT, fontsize=9,
                arrowprops=dict(arrowstyle="->", color=COLOR_ACCENT))
    plt.tight_layout()
    plt.savefig(f"{OUT_DIR}/fig1_fft_spectrum.png", dpi=150, facecolor=COLOR_BG)
    plt.close()


def fig2_feature_comparison():
    keys = ["rms", "harmonic3_energy", "high_freq_energy"]
    labels_kr = {"rms": "RMS", "harmonic3_energy": "3차 배음 에너지", "high_freq_energy": "고주파 대역 에너지"}

    cases = {
        "정상": gen.generate_normal_signal(seed=42),
        "경미한 고장": gen.generate_faulty_signal(seed=42, fault_strength=0.3),
        "심각한 고장": gen.generate_faulty_signal(seed=42, fault_strength=0.9),
    }
    values = {label: feat.extract_features(sig[:WINDOW_SIZE]) for label, (t, sig) in cases.items()}

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5), facecolor=COLOR_BG)
    x_labels = list(cases.keys())
    colors = [COLOR_MUTED, "#B8935F", COLOR_ACCENT]

    for ax, key in zip(axes, keys):
        ax.set_facecolor(COLOR_BG)
        bar_values = [values[label][key] for label in x_labels]
        ax.bar(x_labels, bar_values, color=colors, width=0.55)
        ax.set_title(labels_kr[key], color=COLOR_INK, fontsize=12, fontweight="bold")
        ax.tick_params(colors=COLOR_INK)
        for spine in ax.spines.values():
            spine.set_color(COLOR_MUTED)
        for i, v in enumerate(bar_values):
            ax.text(i, v, f"{v:.3f}", ha="center", va="bottom", color=COLOR_INK, fontsize=9)

    fig.suptitle("정상 → 경미한 고장 → 심각한 고장 특징값 변화 (시뮬레이션)",
                 color=COLOR_INK, fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(f"{OUT_DIR}/fig2_feature_comparison.png", dpi=150, facecolor=COLOR_BG)
    plt.close()


def fig3_progressive_wear():
    signals, strengths = gen.generate_progressive_wear_sequence(n_windows=50, final_fault_strength=0.9, seed=7)
    rms_values = []
    h3_values = []
    for sig in signals:
        f = feat.extract_features(sig[:WINDOW_SIZE])
        rms_values.append(f["rms"])
        h3_values.append(f["harmonic3_energy"])

    fig, ax1 = plt.subplots(figsize=(9, 5), facecolor=COLOR_BG)
    ax1.set_facecolor(COLOR_BG)
    ax1.plot(strengths, rms_values, color=COLOR_MUTED, linewidth=2, label="RMS")
    ax1.set_xlabel("고장 진행도 (0=정상, 1=심각)", color=COLOR_INK)
    ax1.set_ylabel("RMS", color=COLOR_MUTED)
    ax1.tick_params(colors=COLOR_INK)

    ax2 = ax1.twinx()
    ax2.plot(strengths, h3_values, color=COLOR_ACCENT, linewidth=2, label="3차 배음 에너지")
    ax2.set_ylabel("3차 배음 에너지", color=COLOR_ACCENT)
    ax2.tick_params(colors=COLOR_ACCENT)

    ax1.set_title("서서히 진행되는 마모 시나리오 (위험 정상화 방지 검증용)",
                  color=COLOR_INK, fontsize=13, fontweight="bold")
    for spine in ax1.spines.values():
        spine.set_color(COLOR_MUTED)
    for spine in ax2.spines.values():
        spine.set_color(COLOR_MUTED)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False, labelcolor=COLOR_INK)

    plt.tight_layout()
    plt.savefig(f"{OUT_DIR}/fig3_progressive_wear.png", dpi=150, facecolor=COLOR_BG)
    plt.close()


def fig4_detection_confusion():
    ACTIVE_FEATURES = ["rms", "kurtosis", "harmonic2_energy", "harmonic3_energy", "high_freq_energy"]
    normal_features = det.build_normal_baseline(n_windows=60)
    detector = det.BaselineDetector(n_sigma=3.0, confirm_window=5, confirm_ratio=0.6,
                                     features_to_check=ACTIVE_FEATURES)
    detector.fit(normal_features)

    # 0.1부터 이미 100%로 나오면 판별 로직의 실제 민감도 한계가 안 보이므로,
    # 훨씬 더 미세한 강도(0.01~0.05)까지 포함해서 "어디서부터 못 잡기 시작하는가"를 확인한다.
    fault_strengths_to_test = [0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.9]
    detection_rates = []
    for fs in fault_strengths_to_test:
        detector.reset_confirmation_state()
        metrics = det.evaluate_detection_rate(detector, n_trials=50, fault_strength=fs)
        detection_rates.append(1 - metrics["false_negative_rate"])  # 탐지 성공률

    fig, ax = plt.subplots(figsize=(10, 5), facecolor=COLOR_BG)
    ax.set_facecolor(COLOR_BG)
    ax.bar([str(fs) for fs in fault_strengths_to_test], detection_rates, color=COLOR_ACCENT, width=0.55)
    ax.set_xlabel("고장 강도 (fault_strength, 클수록 심각)", color=COLOR_INK)
    ax.set_ylabel("탐지 성공률", color=COLOR_INK)
    ax.set_ylim(0, 1.15)
    ax.set_title("고장 강도별 탐지 성공률 (시뮬레이션, n=50회씩)", color=COLOR_INK, fontsize=13, fontweight="bold")
    ax.tick_params(colors=COLOR_INK)
    for spine in ax.spines.values():
        spine.set_color(COLOR_MUTED)
    for i, v in enumerate(detection_rates):
        ax.text(i, v + 0.03, f"{v*100:.0f}%", ha="center", color=COLOR_INK, fontsize=9)

    plt.tight_layout()
    plt.savefig(f"{OUT_DIR}/fig4_detection_confusion.png", dpi=150, facecolor=COLOR_BG)
    plt.close()
    return dict(zip(fault_strengths_to_test, detection_rates))


if __name__ == "__main__":
    print("그림 생성 중...")
    fig1_fft_spectrum()
    print("  fig1_fft_spectrum.png 완료")
    fig2_feature_comparison()
    print("  fig2_feature_comparison.png 완료")
    fig3_progressive_wear()
    print("  fig3_progressive_wear.png 완료")
    rates = fig4_detection_confusion()
    print("  fig4_detection_confusion.png 완료")
    print("\n고장 강도별 탐지 성공률 (시뮬레이션 기준):")
    for fs, rate in rates.items():
        print(f"  강도 {fs}: {rate*100:.0f}%")
    print("\n주의: 전부 시뮬레이션 신호 기준. 실제 하드웨어 데이터로 반드시 재검증 필요.")
