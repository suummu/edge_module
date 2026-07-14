#!/usr/bin/env python3
"""
pi_server.py — 라즈베리파이 "도서관" 계층 완성판

한 프로세스로 세 가지를 수행한다 (표준 라이브러리만 사용):
  1. 수집: ESP32 /data 를 주기 폴링 → 일 단위 JSONL 축적 (pi_logger 역할 흡수)
  2. 서빙: 감시 콘솔(dashboard.html)을 브라우저에 직접 제공
     → 현장 누구든 브라우저에 http://<pi-ip>:8080 만 치면 콘솔이 뜬다
  3. 제공: /data /learn /health 를 ESP32 로 프록시 (same-origin → CORS 소멸),
     /history 로 축적 이력을 반환 (디스플레이 계층이 "도서관"을 읽는 지점)

사용:
    python3 pi_server.py http://192.168.0.42 --port 8080 --outdir ./logs
    → 브라우저에서 http://<pi-ip>:8080 접속
"""
import argparse
import json
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

HIST_RING = 2000          # 메모리 이력 링 크기 (이력 API 용, 파일과 별개)

# PWA 정적 자산: 경로 → (파일명, Content-Type)
STATIC = {
    "/manifest.json":        ("manifest.json",        "application/manifest+json"),
    "/sw.js":                ("sw.js",                "text/javascript"),
    "/icon-192.png":         ("icon-192.png",         "image/png"),
    "/icon-512.png":         ("icon-512.png",         "image/png"),
    "/apple-touch-icon.png": ("apple-touch-icon.png", "image/png"),
}

state = {
    "esp32": "",
    "latest": {"mode": 0, "offline": 1},
    "ring": deque(maxlen=HIST_RING),
    "dashboard": b"",
    "static": {},
    "outdir": Path("./logs"),
    "lock": threading.Lock(),
}


def poller(interval: float):
    misses = 0
    while True:
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(state["esp32"] + "/data", timeout=2.0) as r:
                rec = json.loads(r.read().decode())
            rec["ts"] = datetime.now(timezone.utc).isoformat()
            with state["lock"]:
                state["latest"] = rec
                state["ring"].append(rec)
            day = state["outdir"] / (datetime.now().strftime("%Y-%m-%d") + ".jsonl")
            with day.open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            if misses:
                print(f"[pi_server] ESP32 재연결 (누락 {misses}회)", flush=True)
                misses = 0
        except Exception as e:
            misses += 1
            with state["lock"]:
                state["latest"] = {**state["latest"], "offline": 1}
            if misses in (1, 10, 100):
                print(f"[pi_server] 수집 실패 x{misses}: {e}", file=sys.stderr, flush=True)
        time.sleep(max(0.0, interval - (time.monotonic() - t0)))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):   # 기본 접근로그 억제
        pass

    def _send(self, code, body: bytes, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _proxy(self, path_qs: str, method="GET"):
        """ESP32 로 요청 전달. 장치 미응답도 JSON 으로 정직하게 알림."""
        try:
            req = urllib.request.Request(state["esp32"] + path_qs, method=method)
            with urllib.request.urlopen(req, timeout=3.0) as r:
                self._send(r.status, r.read())
        except Exception as e:
            self._send(502, json.dumps(
                {"ok": False, "reason": "esp32_unreachable", "detail": str(e)}
            ).encode())

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/" or u.path == "/index.html":
            self._send(200, state["dashboard"], "text/html; charset=utf-8")
        elif u.path in state["static"]:
            body, ctype = state["static"][u.path]
            self._send(200, body, ctype)
        elif u.path == "/data":
            with state["lock"]:
                body = json.dumps(state["latest"], ensure_ascii=False).encode()
            self._send(200, body)
        elif u.path == "/history":
            n = 180
            try:
                n = max(1, min(HIST_RING, int(parse_qs(u.query).get("n", ["180"])[0])))
            except ValueError:
                pass
            with state["lock"]:
                items = list(state["ring"])[-n:]
            self._send(200, json.dumps(items, ensure_ascii=False).encode())
        elif u.path == "/health":
            self._proxy("/health")
        else:
            self._send(404, b'{"error":"not_found"}')

    def do_POST(self):
        u = urlparse(self.path)
        if u.path in ("/learn", "/config", "/baseline/clear"):
            self._proxy(self.path, method="POST")
        else:
            self._send(404, b'{"error":"not_found"}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("esp32_url", help="예: http://192.168.0.42")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--interval", type=float, default=1.0, help="폴링 주기(초)")
    ap.add_argument("--outdir", default="./logs")
    ap.add_argument("--dashboard", default=None,
                    help="dashboard.html 경로 (기본: 이 파일 기준 ../viz/dashboard.html)")
    args = ap.parse_args()

    state["esp32"] = args.esp32_url.rstrip("/")
    state["outdir"] = Path(args.outdir)
    state["outdir"].mkdir(parents=True, exist_ok=True)

    dash = Path(args.dashboard) if args.dashboard else \
        Path(__file__).resolve().parent.parent / "viz" / "dashboard.html"
    state["dashboard"] = dash.read_bytes()
    for route, (fname, ctype) in STATIC.items():
        p = dash.parent / fname
        if p.exists():
            state["static"][route] = (p.read_bytes(), ctype)

    threading.Thread(target=poller, args=(args.interval,), daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[pi_server] http://0.0.0.0:{args.port} ← 브라우저로 접속 | "
          f"수집: {state['esp32']} → {state['outdir']}/", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
