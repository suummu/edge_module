"""파이썬 v2 정답(float64) → C 대조용 테스트 벡터 헤더 생성.

사용: repo 루트에서 `python tools/export_v2_vectors.py`
출력: edge_module_c/pc_test/v2_vectors.h
"""
import sys, pathlib
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from src.feature_extraction_v2 import (FFT_SIZE, SAMPLE_RATE, process_window,
                                       to_z_space, FEATURE_NAMES)

RATED_HZ = 50.0
t = np.arange(FFT_SIZE) / SAMPLE_RATE
rng = np.random.default_rng(7)


def make(label, f1, a1, a2, a3, hf_amp, noise, dc=0.98):
    sig = (a1 * np.sin(2 * np.pi * f1 * t)
           + a2 * np.sin(2 * np.pi * 2 * f1 * t + 0.7)
           + a3 * np.sin(2 * np.pi * 3 * f1 * t + 1.9)
           + hf_amp * np.sin(2 * np.pi * 240.0 * t + 0.3)
           + noise * rng.standard_normal(FFT_SIZE) + dc)
    return label, sig


CASES = [
    make("normal",        49.7, 0.030, 0.008, 0.004, 0.006, 0.009),
    make("normal_offbin", 50.5, 0.030, 0.008, 0.004, 0.006, 0.009),   # bin 경계 근처
    make("unbalance",     49.7, 0.090, 0.008, 0.004, 0.006, 0.009),   # 1x 3배
    make("misalign",      49.7, 0.030, 0.032, 0.004, 0.006, 0.009),   # 2x 4배
    make("looseness",     49.7, 0.030, 0.020, 0.016, 0.006, 0.009),   # 2x/3x 상승
    make("hf_anomaly",    49.7, 0.030, 0.008, 0.004, 0.024, 0.009),   # HF 4배
    make("speed_change",  57.5, 0.075, 0.020, 0.010, 0.006, 0.009),   # 속도↑, 진폭 2.5배
]

out = ["/* 자동 생성 — python tools/export_v2_vectors.py. 손으로 고치지 말 것 */",
       "#ifndef V2_VECTORS_H", "#define V2_VECTORS_H",
       f"#define V2_N_CASES {len(CASES)}",
       f"#define V2_WINDOW {FFT_SIZE}",
       f"static const float V2_RATED_HZ = {RATED_HZ}f;"]

labels, sigs, feats, rots, zs = [], [], [], [], []
for label, sig in CASES:
    r = process_window(sig, RATED_HZ)
    labels.append(label)
    sigs.append(sig)
    feats.append(r["features"])
    rots.append(r["rotation_hz"])
    zs.append(r["z_space"])
    print(f"{label:>14}: rot={r['rotation_hz']:.4f}Hz snr={r['snr']:.1f} "
          + " ".join(f"{n}={v:.6g}" for n, v in zip(FEATURE_NAMES, r["features"])))

def arr(v, per=8):
    s = [f"{x:.9e}f" for x in v]
    return ",\n    ".join(", ".join(s[i:i+per]) for i in range(0, len(s), per))

out.append("static const char *V2_LABELS[V2_N_CASES] = {"
           + ", ".join(f'"{l}"' for l in labels) + "};")
out.append("static const float V2_SIGNALS[V2_N_CASES][V2_WINDOW] = {")
for sig in sigs:
    out.append("  {\n    " + arr(sig) + "\n  },")
out.append("};")
out.append("static const double V2_EXPECT_FEATURES[V2_N_CASES][5] = {")
for f in feats:
    out.append("  {" + ", ".join(f"{x:.17g}" for x in f) + "},")
out.append("};")
out.append("static const double V2_EXPECT_ROT[V2_N_CASES] = {"
           + ", ".join(f"{x:.17g}" for x in rots) + "};")
out.append("static const double V2_EXPECT_Z[V2_N_CASES][5] = {")
for z in zs:
    out.append("  {" + ", ".join(f"{x:.17g}" for x in z) + "},")
out.append("};")
out.append("#endif")

dst = ROOT / "edge_module_c" / "pc_test" / "v2_vectors.h"
dst.write_text("\n".join(out))
print(f"\n→ {dst} ({dst.stat().st_size/1024:.0f} KB)")
