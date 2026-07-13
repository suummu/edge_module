"""
dataset.py

목적:
    다운로드된 CWRU(.mat) / MaFaulDa(.csv) 원시 신호를 읽어서
    "윈도우 단위 신호 + 메타데이터(라벨, 회전주파수, 샘플레이트)"로 잘라준다.

    기존 파이프라인의 signal_generator.py 자리를 실제 데이터 리더로 교체하는
    파일이다. feature_extraction의 함수들은 입력이 실신호여도 그대로 동작하므로
    (그렇게 설계했음), 이 파일은 '신호를 윈도우로 공급'하는 역할만 한다.

윈도우 크기 설계:
    ESP32 시뮬레이션 설정(1kHz/256pt)은 주파수 해상도 3.9Hz였다.
    실데이터도 비슷한 해상도와 '윈도우당 약 10회전' 조건을 맞추기 위해:
      - CWRU  12kHz -> 4096pt  (0.34s, 해상도 2.93Hz, 약 30Hz 회전 기준 10회전)
      - MaFaulDa 50kHz -> 16384pt (0.33s, 해상도 3.05Hz)
    hop = window/2 (50% 겹침)로 윈도우 수를 확보한다.

주의(데이터 누수 방지):
    같은 원본 파일에서 나온 윈도우들은 서로 강하게 상관되어 있으므로,
    train/test 분할은 반드시 "파일 단위"로 해야 한다. 이를 위해 모든 윈도우에
    group(파일 식별자)을 붙여 반환한다.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from .download_data import CWRU_DIR, CWRU_FILES, MAFAULDA_DIR, MAFAULDA_FOLDERS

CWRU_SAMPLE_RATE = 12_000
CWRU_WINDOW = 4096
MAFAULDA_SAMPLE_RATE = 50_000
MAFAULDA_WINDOW = 16384

# MaFaulDa CSV 컬럼: 0=타코미터, 1~3=언더행 베어링 가속도(축/반경/접선),
# 4~6=오버행 베어링 가속도, 7=마이크
# 불평형/정렬불량은 반경방향 진동에 가장 잘 드러나므로 언더행 반경방향(2)을 사용.
MAFAULDA_CHANNEL = 2


@dataclass
class Window:
    signal: np.ndarray       # 시간영역 신호 (윈도우 1개)
    label: str               # normal / imbalance / misalignment / inner_race / ball / outer_race
    rotation_hz: float       # 명목 회전주파수 (탐색범위 중심으로만 사용)
    sample_rate: int
    group: str               # 원본 파일 식별자 (파일 단위 train/test 분할용)
    severity: str = "-"      # 결함 심각도 표기 (예: "35g", "0.021in")


def _segment(signal, window, hop):
    for start in range(0, len(signal) - window + 1, hop):
        yield signal[start:start + window]


def load_cwru_windows(window=CWRU_WINDOW, hop=None):
    """CWRU .mat 파일들 -> Drive-End 가속도 채널 윈도우 리스트."""
    hop = hop or window // 2
    windows = []
    for num, (label, size_in, hp, rpm) in sorted(CWRU_FILES.items()):
        path = CWRU_DIR / f"{num}.mat"
        if not path.exists():
            continue
        mat = loadmat(path)
        # Drive-End 채널 키는 'X097_DE_time' 형태
        de_keys = [k for k in mat if k.endswith("_DE_time")]
        if not de_keys:
            continue
        signal = mat[de_keys[0]].ravel().astype(np.float64)
        for seg in _segment(signal, window, hop):
            windows.append(Window(
                signal=seg, label=label, rotation_hz=rpm / 60.0,
                sample_rate=CWRU_SAMPLE_RATE, group=f"cwru_{num}",
                severity=f"{size_in}in" if size_in else "-",
            ))
    return windows


def load_mafaulda_windows(window=MAFAULDA_WINDOW, hop=None, channel=MAFAULDA_CHANNEL):
    """MaFaulDa .csv 파일들 -> 지정 가속도 채널 윈도우 리스트."""
    hop = hop or window // 2
    windows = []
    for folder, label, severity, _n in MAFAULDA_FOLDERS:
        local_dir = MAFAULDA_DIR / folder
        if not local_dir.exists():
            continue
        for path in sorted(local_dir.glob("*.csv")):
            rotation_hz = float(path.name[:-4])
            data = np.loadtxt(path, delimiter=",", usecols=(channel,))
            for seg in _segment(data, window, hop):
                windows.append(Window(
                    signal=seg, label=label, rotation_hz=rotation_hz,
                    sample_rate=MAFAULDA_SAMPLE_RATE,
                    group=f"mafaulda_{folder}_{path.name}", severity=severity,
                ))
    return windows


if __name__ == "__main__":
    from collections import Counter
    for name, loader in [("CWRU", load_cwru_windows), ("MaFaulDa", load_mafaulda_windows)]:
        ws = loader()
        counts = Counter(w.label for w in ws)
        print(f"{name}: 윈도우 {len(ws)}개, 라벨 분포 {dict(counts)}")
