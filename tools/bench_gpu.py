"""Run a command and record wall time and peak GPU memory (feasibility test for GPU SLAM systems).

    python tools/bench_gpu.py --name mast3r_slam --frames 613 --out results_bench/mast3r_slam -- <command ...>

Writes <out>/bench.json with: wall time, frames per second (if --frames is given), peak GPU memory
above the idle baseline, the GPU name and the exit code. Run it on the host (not inside the
container) so nvidia-smi sees the whole GPU. Use one GPU job at a time, e.g. with
`flock /tmp/bearl_gpu.lock python tools/bench_gpu.py ...`, otherwise the memory numbers mix.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import threading
import time
from pathlib import Path


def gpu_used_mib() -> float:
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    return float(out.split()[0])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", type=int, help="number of input frames, to report FPS")
    ap.add_argument("cmd", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    cmd = a.cmd[1:] if a.cmd and a.cmd[0] == "--" else a.cmd
    a.out.mkdir(parents=True, exist_ok=True)

    baseline = gpu_used_mib()
    peak = [baseline]
    stop = threading.Event()

    def poll():
        while not stop.wait(0.5):
            peak[0] = max(peak[0], gpu_used_mib())

    t = threading.Thread(target=poll, daemon=True); t.start()
    t0 = time.perf_counter()
    with open(a.out / "log.txt", "w") as log:
        rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
    wall = time.perf_counter() - t0
    stop.set(); t.join()

    gpu = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
                         capture_output=True, text=True).stdout.strip()
    res = {"name": a.name, "exit_code": rc, "wall_time_s": round(wall, 1),
           "fps": round(a.frames / wall, 2) if a.frames else None,
           "gpu_peak_mib_above_idle": round(peak[0] - baseline), "gpu": gpu, "command": cmd}
    (a.out / "bench.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
