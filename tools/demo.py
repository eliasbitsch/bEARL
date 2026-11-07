"""Live demo: replay one run in Lichtblick (Foxglove Studio fork) with the bEARL layout, in one command.

    python tools/demo.py                                    # VGGT-Long, Dynamic01, run_00
    python tools/demo.py --system vggt_long --sequence Varying-illu01
    python tools/demo.py --no-browser                       # only print the link

Builds the MCAP replay with pipeline.visualize if it is missing (trajectory, error line, camera, maps), starts
docker/demo (Lichtblick on :8081, file server on :8791) and opens the replay. On Windows Docker runs in WSL.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import urllib.request
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import RESULTS_DIR, load_system_config  # noqa: E402

LICHTBLICK, FILES = "http://localhost:8081", "http://localhost:8791"


def _wsl(p: Path) -> str:
    return f"/mnt/{p.drive[0].lower()}{p.as_posix()[2:]}"


def compose_up() -> None:
    compose = ROOT / "docker" / "demo" / "compose.yaml"
    cmd = ["docker", "compose", "-f", str(compose), "up", "-d"]
    if os.name == "nt":
        cmd = ["wsl.exe", "-d", os.environ.get("WSL_DISTRO", "Ubuntu-24.04"), "--",
               "docker", "compose", "-f", _wsl(compose), "up", "-d"]
    subprocess.run(cmd, check=True)
    for _ in range(30):                     # the containers need a moment after a cold start
        try:
            urllib.request.urlopen(LICHTBLICK, timeout=1)
            urllib.request.urlopen(FILES + "/results/", timeout=1)
            return
        except OSError:
            time.sleep(1)
    raise SystemExit("demo containers did not come up, see `docker compose -f docker/demo/compose.yaml logs`")


def replay(slug: str, seq: str, run: str) -> Path:
    run_dir = RESULTS_DIR / slug / seq / run
    viz = run_dir / "viz"
    mcaps = sorted(viz.glob("*.mcap"))
    if not mcaps:
        cmd = [sys.executable, "-m", "pipeline.visualize", "--sequence", seq,
               "--est", str(run_dir / "trajectory_tum.txt"), "--alignment", load_system_config(slug)["alignment"],
               "--out", str(viz.with_suffix(".mcap"))]
        if (run_dir / "map.ply").exists():
            cmd += ["--map", str(run_dir / "map.ply")]
        subprocess.run(cmd, check=True, cwd=ROOT)
        mcaps = sorted(viz.glob("*.mcap"))
    return mcaps[0]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--system", default="vggt_long")
    ap.add_argument("--sequence", default="Dynamic01")
    ap.add_argument("--run", default="run_00")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()

    mcap = replay(a.system, a.sequence, a.run)
    compose_up()
    url = f"{LICHTBLICK}/?ds=remote-file&ds.url={FILES}/results/{mcap.relative_to(RESULTS_DIR).as_posix()}"
    print(url)
    if not a.no_browser:
        webbrowser.open(url)


if __name__ == "__main__":
    main()
