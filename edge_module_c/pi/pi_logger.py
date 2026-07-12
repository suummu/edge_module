#!/usr/bin/env python3
"""
pi_logger.py — 라즈베리파이 "도서관" 계층 (저장 전담, 해석 없음)

ESP32 의 GET /data 를 주기 폴링해 타임스탬프를 붙여 JSONL 로 축적한다.
일 단위 파일 회전. 표준 라이브러리만 사용 (Pi 에 pip 불필요).

사용:
    python3 pi_logger.py http://192.168.0.42 --interval 1.0 --outdir ./logs

디스플레이 계층(dashboard.html)은 라이브 값은 ESP32 에서,
이력/추세는 이 로그에서 읽는다는 것이 3계층 설계의 분담이다.
"""
import argparse
import json
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


def fetch(url: str, timeout: float = 2.0):
    with urllib.request.urlopen(url + "/data", timeout=timeout) as r:
        return json.loads(r.read().decode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("esp32_url", help="예: http://192.168.0.42")
    ap.add_argument("--interval", type=float, default=1.0, help="폴링 주기(초)")
    ap.add_argument("--outdir", default="./logs")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    misses = 0

    print(f"[pi_logger] {args.esp32_url} 폴링 시작 → {outdir}/", flush=True)
    while True:
        t0 = time.monotonic()
        try:
            rec = fetch(args.esp32_url)
            rec["ts"] = datetime.now(timezone.utc).isoformat()
            day_file = outdir / (datetime.now().strftime("%Y-%m-%d") + ".jsonl")
            with day_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            if misses:
                print(f"[pi_logger] 재연결 (누락 {misses}회)", flush=True)
                misses = 0
        except Exception as e:
            misses += 1
            # 통신 단절도 데이터: 언제 끊겼는지 남긴다
            if misses in (1, 10, 100):
                print(f"[pi_logger] 수집 실패 x{misses}: {e}", file=sys.stderr, flush=True)
        time.sleep(max(0.0, args.interval - (time.monotonic() - t0)))


if __name__ == "__main__":
    main()
