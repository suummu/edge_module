"""
make_figures.py

목적:
    학습 결과(ml/results/*_results.json)와 특징 데이터를 보고서용 그림으로 저장.
      F1. 고장 분류 혼동행렬 (결정트리 vs RandomForest)
      F2. RandomForest 특징 중요도
      F3. 클래스별 특징 분포 (박스플롯) - "어떤 특징이 어떤 고장을 가르는가"

사용법:
    python -m ml.make_figures cwru
    python -m ml.make_figures mafaulda
"""

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ml.features_real import FEATURE_NAMES, windows_to_matrix  # noqa: E402
from ml.train_models import RESULT_DIR  # noqa: E402

FIG_DIR = RESULT_DIR / "figs"

# 시각화 팔레트 (light surface 기준, 고정 순서 - 클래스에 색이 따라감)
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
CATEGORICAL = ["#2a78d6", "#eda100", "#e34948", "#4a3aa7", "#1baf7a", "#eb6834"]
SEQ_CMAP = LinearSegmentedColormap.from_list(
    "seq_blue", ["#f0efec", "#86b6ef", "#2a78d6", "#104281"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "text.color": TEXT_PRIMARY, "axes.labelcolor": TEXT_PRIMARY,
    "xtick.color": TEXT_SECONDARY, "ytick.color": TEXT_SECONDARY,
    "axes.edgecolor": "#d8d7d2", "axes.grid": False,
    "font.size": 10, "figure.dpi": 150,
    # 한글 라벨 지원 (Windows 기본 폰트), 마이너스 기호 깨짐 방지
    "font.family": "Malgun Gothic", "axes.unicode_minus": False,
})


def plot_confusion(results, dataset):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    for ax, model in zip(axes, ["decision_tree_d4", "random_forest"]):
        r = results["classification"][model]
        cm = np.array(r["confusion_matrix"], dtype=float)
        cm_norm = cm / cm.sum(axis=1, keepdims=True)
        labels = r["labels"]
        ax.imshow(cm_norm, cmap=SEQ_CMAP, vmin=0, vmax=1)
        ax.set_xticks(range(len(labels)), labels, rotation=30, ha="right")
        ax.set_yticks(range(len(labels)), labels)
        for i in range(len(labels)):
            for j in range(len(labels)):
                color = "white" if cm_norm[i, j] > 0.55 else TEXT_PRIMARY
                ax.text(j, i, f"{int(cm[i, j])}", ha="center", va="center",
                        color=color, fontsize=9)
        acc = r["accuracy"] * 100
        title = {"decision_tree_d4": f"Decision Tree d≤4 (ESP32 이식 후보) — {acc:.1f}%",
                 "random_forest": f"Random Forest (참고 상한) — {acc:.1f}%"}[model]
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("예측 라벨")
        ax.set_ylabel("실제 라벨")
    fig.suptitle(f"{dataset.upper()} 고장 분류 혼동행렬 (test는 학습에 안 쓴 파일만)", fontsize=11)
    fig.tight_layout()
    out = FIG_DIR / f"{dataset}_confusion.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_importance(results, dataset):
    imp = results["classification"]["feature_importance_rf"]
    names = list(imp)[::-1]
    vals = [imp[n] for n in names]
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.barh(names, vals, color="#2a78d6", height=0.55)
    for i, v in enumerate(vals):
        ax.text(v + 0.005, i, f"{v:.2f}", va="center", color=TEXT_SECONDARY, fontsize=9)
    ax.set_xlim(0, max(vals) * 1.18)
    ax.set_title(f"{dataset.upper()} 특징 중요도 (Random Forest)", fontsize=11)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    out = FIG_DIR / f"{dataset}_importance.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_distributions(windows, dataset):
    X, y, _, _ = windows_to_matrix(windows)
    labels = sorted(set(y))
    fig, axes = plt.subplots(2, 3, figsize=(11, 6))
    for k, (ax, fname) in enumerate(zip(axes.ravel(), FEATURE_NAMES)):
        data = [X[y == lbl, k] for lbl in labels]
        bp = ax.boxplot(data, tick_labels=labels, patch_artist=True,
                        medianprops={"color": TEXT_PRIMARY}, showfliers=False)
        for patch, color in zip(bp["boxes"], CATEGORICAL):
            patch.set_facecolor(color)
            patch.set_alpha(0.75)
            patch.set_edgecolor("none")
        ax.set_title(fname, fontsize=10)
        ax.tick_params(axis="x", rotation=25)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"{dataset.upper()} 클래스별 특징 분포 (윈도우 단위)", fontsize=12)
    fig.tight_layout()
    out = FIG_DIR / f"{dataset}_feature_dist.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def run(dataset):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    results = json.loads((RESULT_DIR / f"{dataset}_results.json").read_text(encoding="utf-8"))
    outs = [plot_confusion(results, dataset), plot_importance(results, dataset)]

    if dataset == "cwru":
        from ml.dataset import load_cwru_windows as loader
    else:
        from ml.dataset import load_mafaulda_windows as loader
    outs.append(plot_distributions(loader(), dataset))
    for o in outs:
        print(f"저장: {o}")


if __name__ == "__main__":
    for dataset in sys.argv[1:] or ["cwru", "mafaulda"]:
        try:
            run(dataset)
        except FileNotFoundError as e:
            print(f"{dataset}: 결과 없음 ({e})")
