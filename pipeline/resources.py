"""Background sampler for CPU, RAM and (optionally) GPU memory of a process tree or a Docker container.

Resource numbers are context only: every system may run on different hardware,
so they are reported together with the hardware and never used for ranking.

For a system that runs in Docker, the process tree only contains the `docker` CLI, not the SLAM system.
Such a runner names its container `cfg["container_name"]` (set by run_all.py when the system config has
`docker: true`); the sampler then reads the container with `docker stats` (about one sample per 2 s,
CPU in % of one core as for the process tree, RAM = container memory usage). GPU memory is the whole GPU in
both cases (nvidia-smi), so close other GPU programs during runs.
"""
from __future__ import annotations

import csv
import platform
import shutil
import subprocess
import threading
import time
from pathlib import Path

import psutil


def _gpu_memory_mib() -> float | None:
    if shutil.which("nvidia-smi") is None:
        return None
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2,
        ).stdout
        return sum(float(v) for v in out.split())
    except (subprocess.SubprocessError, ValueError):
        return None


_MEM_UNITS = {"B": 1 / 2**20, "KiB": 1 / 2**10, "MiB": 1.0, "GiB": 2**10, "TiB": 2**20,
              "kB": 1e3 / 2**20, "MB": 1e6 / 2**20, "GB": 1e9 / 2**20}


def _parse_mem_mib(s: str) -> float:
    """'426.3MiB' -> 426.3 (MiB)."""
    s = s.strip()
    for unit in sorted(_MEM_UNITS, key=len, reverse=True):
        if s.endswith(unit):
            return float(s[: -len(unit)]) * _MEM_UNITS[unit]
    raise ValueError(f"unknown memory unit in {s!r}")


def _docker_stats(container: str) -> tuple[float, float] | None:
    """(CPU % of one core, memory MiB) of a running container, None if it is not running (yet)."""
    try:
        out = subprocess.run(["docker", "stats", "--no-stream", "--format", "{{.CPUPerc}}|{{.MemUsage}}", container],
                             capture_output=True, text=True, timeout=10)
        if out.returncode != 0 or "|" not in out.stdout:
            return None
        cpu, mem = out.stdout.strip().splitlines()[0].split("|")
        return float(cpu.strip().rstrip("%")), _parse_mem_mib(mem.split("/")[0])
    except (subprocess.SubprocessError, ValueError, OSError):
        return None


def hardware_info() -> dict:
    info = {
        "platform": platform.platform(),
        "cpu": platform.processor() or platform.machine(),
        "cpu_cores_logical": psutil.cpu_count(logical=True),
        "ram_gib": round(psutil.virtual_memory().total / 2**30, 1),
        "gpu": None,
    }
    if shutil.which("nvidia-smi"):
        try:
            info["gpu"] = subprocess.run(
                ["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
                capture_output=True, text=True, timeout=2,
            ).stdout.strip()
        except subprocess.SubprocessError:
            pass
    return info


class ResourceSampler:
    """Samples the given process and all its children at a fixed rate.

    Usage:
        with ResourceSampler(out_csv) as sampler:
            run_slam()
        sampler.summary  # peak RAM, mean CPU, ...
    """

    def __init__(self, out_csv: Path, pid: int | None = None, rate_hz: float = 10.0,
                 gpu_every_n: int = 10, container: str | None = None):
        self.out_csv = Path(out_csv)
        self.root = psutil.Process(pid)
        self.container = container
        self.period = 1.0 / rate_hz
        self.gpu_every_n = gpu_every_n
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self.summary: dict = {}

    def _tree(self) -> list[psutil.Process]:
        try:
            return [self.root] + self.root.children(recursive=True)
        except psutil.NoSuchProcess:
            return []

    def _loop(self) -> None:
        rows = self._sample_container() if self.container else self._sample_tree()
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(self.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "cpu_percent_one_core", "ram_mib", "gpu_mem_mib_total"])
            w.writerows(rows)
        if rows:
            cpus = [r[1] for r in rows]
            gpus = [r[3] for r in rows if r[3] is not None]
            self.summary = {
                "cpu_mean_percent": sum(cpus) / len(cpus),
                "ram_peak_mib": max(r[2] for r in rows),
                "gpu_mem_peak_mib": max(gpus) if gpus else None,
                "scope": f"container {self.container}" if self.container else "process tree",
                "samples": len(rows),
            }

    def _sample_container(self) -> list[tuple]:
        """docker stats blocks about 1.7 s per call, which sets the rate; samples before the container
        starts and after it ends are skipped."""
        rows, t0 = [], time.perf_counter()
        while not self._stop.is_set():
            s = _docker_stats(self.container)
            if s is not None:
                rows.append((time.perf_counter() - t0, s[0], s[1], _gpu_memory_mib()))
            else:
                self._stop.wait(0.2)
        return rows

    def _sample_tree(self) -> list[tuple]:
        rows, i, t0 = [], 0, time.perf_counter()
        for p in self._tree():
            try:
                p.cpu_percent(None)  # prime the counters
            except psutil.Error:
                pass
        gpu = None
        while not self._stop.wait(self.period):
            cpu, rss = 0.0, 0
            for p in self._tree():
                try:
                    cpu += p.cpu_percent(None)
                    rss += p.memory_info().rss
                except psutil.Error:
                    continue
            if i % self.gpu_every_n == 0:
                gpu = _gpu_memory_mib()
            rows.append((time.perf_counter() - t0, cpu, rss / 2**20, gpu))
            i += 1
        return rows

    def __enter__(self) -> "ResourceSampler":
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join()
