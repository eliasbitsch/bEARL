"""Local Lichtblick (open-source Foxglove Studio fork) for the replays in results/replay, no Docker, no account.

    python tools/viewer.py                         # serve and open Varying-illu01 in the browser
    python tools/viewer.py --sequence Dynamic01 --no-browser

One HTTP server on :8781 serves the Lichtblick web build (downloaded once into tools/lichtblick-web, git-ignored)
with tools/replay_layout.json as default layout, and results/replay/*.mcap with Range requests. notebooks/story.py
embeds the same URL in an iframe (app.foxglove.dev forbids iframes; Foxglove desktop opens the .mcap directly).
"""
from __future__ import annotations

import argparse
import io
import json
import re
import socket
import tarfile
import threading
import urllib.request
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "tools" / "lichtblick-web"
LAYOUT = ROOT / "tools" / "replay_layout.json"
REPLAY = ROOT / "results" / "replay"
VERSION = "v1.31.0"
WEB_URL = f"https://github.com/lichtblick-suite/lichtblick/releases/download/{VERSION}/lichtblick-web.tar.gz"
PORT = 8781
BASE = f"http://localhost:{PORT}"


def ensure_web() -> None:
    if (WEB / "index.html").exists():
        return
    print(f"downloading Lichtblick {VERSION} web build (~40 MB) ...")
    with urllib.request.urlopen(WEB_URL) as r, tarfile.open(fileobj=io.BytesIO(r.read())) as tar:
        WEB.mkdir(parents=True, exist_ok=True)
        tar.extractall(WEB, members=[m for m in tar if not m.name.endswith(".map")], filter="data")


class Handler(SimpleHTTPRequestHandler):
    def log_message(self, *args) -> None:
        pass

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Expose-Headers", "Accept-Ranges, Content-Range, Content-Length")
        self.send_header("Accept-Ranges", "bytes")
        super().end_headers()

    def do_GET(self) -> None:
        path = self.path.split("?")[0]
        if path == "/bearl-viewer":
            return self._send(b"ok", "text/plain")
        if path in ("/", "/index.html"):
            # the web build reads its default layout from this placeholder (the Docker image patches it the same way)
            html = (WEB / "index.html").read_text(encoding="utf-8").replace(
                "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/", json.dumps(json.loads(LAYOUT.read_text())))
            return self._send(html.encode(), "text/html")
        if path.startswith("/replay/"):
            f = REPLAY / Path(path).name
            if not f.is_file():
                return self.send_error(404)
            return self._send_file(f)
        super().do_GET()

    def _send(self, body: bytes, ctype: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, f: Path) -> None:
        size = f.stat().st_size
        m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range", ""))
        a, b = 0, size - 1
        if m and (m[1] or m[2]):
            if m[1]:
                a, b = int(m[1]), min(int(m[2]), size - 1) if m[2] else size - 1
            else:
                a = max(size - int(m[2]), 0)
            if a > b or a >= size:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self.send_response(206)
            self.send_header("Content-Range", f"bytes {a}-{b}/{size}")
        else:
            self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(b - a + 1))
        self.end_headers()
        with open(f, "rb") as fh:
            fh.seek(a)
            left = b - a + 1
            while left > 0:
                chunk = fh.read(min(left, 1 << 20))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    return
                left -= len(chunk)


def serve() -> str:
    """Start the server in a daemon thread unless it already runs (e.g. from an earlier cell run). Returns the base URL."""
    with socket.socket() as s:
        if s.connect_ex(("127.0.0.1", PORT)) == 0:
            try:
                with urllib.request.urlopen(f"{BASE}/bearl-viewer", timeout=2) as r:
                    if r.read() == b"ok":
                        return BASE
            except OSError:
                pass
            raise RuntimeError(f"port {PORT} is used by another program; change PORT in tools/viewer.py")
    ensure_web()
    httpd = ThreadingHTTPServer(("127.0.0.1", PORT), partial(Handler, directory=str(WEB)))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return BASE


def url(seq: str) -> str:
    return f"{BASE}/?ds=remote-file&ds.url={quote(f'{BASE}/replay/{seq}.mcap', safe='')}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", default="Varying-illu01")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    if not (REPLAY / f"{a.sequence}.mcap").exists():
        raise SystemExit(f"no replay for {a.sequence}: python tools/build_replay.py --sequence {a.sequence}")
    serve()
    print(url(a.sequence), "  (Ctrl+C to stop)")
    if not a.no_browser:
        webbrowser.open(url(a.sequence))
    threading.Event().wait()


if __name__ == "__main__":
    main()
