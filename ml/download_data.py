"""
download_data.py

목적:
    공개 데이터셋(CWRU, MaFaulDa)에서 학습에 필요한 서브셋만 선별 다운로드한다.

    - CWRU (Case Western Reserve University Bearing Data Center):
        12kHz Drive-End 가속도 데이터, .mat 파일 (파일당 약 4MB).
        정상 + 내륜(IR)/볼(B)/외륜(OR) 결함 x 결함크기(0.007/0.014/0.021인치)
        x 부하(0~3HP) 조합. 베어링 결함 = 고주파/충격성 신호 검증용.
    - MaFaulDa (Machinery Fault Database, UFRJ):
        50kHz, 250,000샘플(5초) x 8채널 CSV (파일당 약 17MB).
        정상 / 불평형(imbalance) / 수평·수직 정렬불량(misalignment).
        불평형(1x), 정렬불량(2x) = 본 프로젝트 핵심 고장유형과 직접 대응.
        전체 DB는 13GB이므로, 조건별로 회전속도를 고르게 분산시킨
        일부 파일만 받는다 (총 약 1GB 이내).

사용법:
    python -m ml.download_data cwru      # CWRU만
    python -m ml.download_data mafaulda  # MaFaulDa만
    python -m ml.download_data all
"""

import re
import sys
import urllib.request
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent.parent / "data"

# ---------------------------------------------------------------------------
# CWRU
# ---------------------------------------------------------------------------
CWRU_BASE = "https://engineering.case.edu/sites/default/files"
CWRU_DIR = DATA_DIR / "cwru"

# 파일번호 -> (라벨, 결함크기 inch, 부하 HP, 명목 RPM)
# 12kHz Drive-End 데이터 기준의 표준 파일번호 매핑.
CWRU_FILES = {
    # 정상 (Normal baseline, 48kHz가 아닌 baseline 채널 포함 .mat)
    97:  ("normal", 0.000, 0, 1797), 98:  ("normal", 0.000, 1, 1772),
    99:  ("normal", 0.000, 2, 1750), 100: ("normal", 0.000, 3, 1730),
    # 내륜 결함 (Inner Race)
    105: ("inner_race", 0.007, 0, 1797), 106: ("inner_race", 0.007, 1, 1772),
    107: ("inner_race", 0.007, 2, 1750), 108: ("inner_race", 0.007, 3, 1730),
    169: ("inner_race", 0.014, 0, 1797), 170: ("inner_race", 0.014, 1, 1772),
    171: ("inner_race", 0.014, 2, 1750), 172: ("inner_race", 0.014, 3, 1730),
    209: ("inner_race", 0.021, 0, 1797), 210: ("inner_race", 0.021, 1, 1772),
    211: ("inner_race", 0.021, 2, 1750), 212: ("inner_race", 0.021, 3, 1730),
    # 볼 결함 (Ball)
    118: ("ball", 0.007, 0, 1797), 119: ("ball", 0.007, 1, 1772),
    120: ("ball", 0.007, 2, 1750), 121: ("ball", 0.007, 3, 1730),
    185: ("ball", 0.014, 0, 1797), 186: ("ball", 0.014, 1, 1772),
    187: ("ball", 0.014, 2, 1750), 188: ("ball", 0.014, 3, 1730),
    222: ("ball", 0.021, 0, 1797), 223: ("ball", 0.021, 1, 1772),
    224: ("ball", 0.021, 2, 1750), 225: ("ball", 0.021, 3, 1730),
    # 외륜 결함 (Outer Race, 6시 방향 = 하중대 중심)
    130: ("outer_race", 0.007, 0, 1797), 131: ("outer_race", 0.007, 1, 1772),
    132: ("outer_race", 0.007, 2, 1750), 133: ("outer_race", 0.007, 3, 1730),
    197: ("outer_race", 0.014, 0, 1797), 198: ("outer_race", 0.014, 1, 1772),
    199: ("outer_race", 0.014, 2, 1750), 200: ("outer_race", 0.014, 3, 1730),
    234: ("outer_race", 0.021, 0, 1797), 235: ("outer_race", 0.021, 1, 1772),
    236: ("outer_race", 0.021, 2, 1750), 237: ("outer_race", 0.021, 3, 1730),
}

# ---------------------------------------------------------------------------
# MaFaulDa
# ---------------------------------------------------------------------------
MAFAULDA_BASE = "https://www02.smt.ufrj.br/~offshore/mfs/database/mafaulda"
MAFAULDA_DIR = DATA_DIR / "mafaulda"

# (서버 폴더 경로, 라벨, 심각도 표기, 받을 파일 수)
# 회전속도(파일명 = 근사 회전주파수 Hz)를 폴더 전체에서 고르게 뽑는다.
MAFAULDA_FOLDERS = [
    ("normal",                        "normal",       "-",      12),
    ("imbalance/6g",                  "imbalance",    "6g",     6),
    ("imbalance/35g",                 "imbalance",    "35g",    6),
    ("horizontal-misalignment/1.0mm", "misalignment", "h1.0mm", 6),
    ("horizontal-misalignment/2.0mm", "misalignment", "h2.0mm", 6),
    # vertical-misalignment/* 는 서버가 403 Forbidden으로 막아둠 (2026-07 확인).
    # 수평 정렬불량 12개 파일로 misalignment 클래스를 구성한다.
]


def _download(url, dest: Path):
    if dest.exists() and dest.stat().st_size > 0:
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    urllib.request.urlretrieve(url, tmp)
    tmp.rename(dest)
    return True


def download_cwru():
    print(f"[CWRU] {len(CWRU_FILES)}개 파일 -> {CWRU_DIR}")
    for num, (label, size, hp, rpm) in sorted(CWRU_FILES.items()):
        dest = CWRU_DIR / f"{num}.mat"
        new = _download(f"{CWRU_BASE}/{num}.mat", dest)
        print(f"  {num}.mat ({label}, {size}in, {hp}HP) {'다운로드' if new else '이미 있음'}")


def _list_remote_csvs(folder):
    html = urllib.request.urlopen(f"{MAFAULDA_BASE}/{folder}/", timeout=30).read().decode()
    return sorted(re.findall(r'href="([0-9][^"]*\.csv)"', html), key=float_prefix)


def float_prefix(name):
    return float(name[:-4])  # "12.288.csv" -> 12.288


def _evenly_spaced(items, k):
    if k >= len(items):
        return items
    step = (len(items) - 1) / (k - 1)
    return [items[round(i * step)] for i in range(k)]


def download_mafaulda():
    for folder, label, severity, n in MAFAULDA_FOLDERS:
        files = _evenly_spaced(_list_remote_csvs(folder), n)
        local_dir = MAFAULDA_DIR / folder
        print(f"[MaFaulDa] {folder} ({label}/{severity}) {len(files)}개 -> {local_dir}")
        for f in files:
            new = _download(f"{MAFAULDA_BASE}/{folder}/{f}", local_dir / f)
            print(f"  {f} {'다운로드' if new else '이미 있음'}")


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "all"
    if target in ("cwru", "all"):
        download_cwru()
    if target in ("mafaulda", "all"):
        download_mafaulda()
    print("완료")
