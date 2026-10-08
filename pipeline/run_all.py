"""Run every registered SLAM system N times on every shared sequence.

    python -m pipeline.run_all                       # all systems in team.yaml
    python -m pipeline.run_all --system mast3r_slam  # only one system (e.g. on your own machine)
    python -m pipeline.run_all --system mast3r_slam --sequence Dark03 --run 3   # one job (cluster)

Finished runs are skipped, so the script can be restarted. Each run writes
results/<slug>/<sequence>/run_<k>/{trajectory_tum.txt, map.*, resources.csv, meta.json}.
"""
from __future__ import annotations

import argparse
import datetime as dt
import importlib
import json
import subprocess
import time
import traceback
from pathlib import Path

from pipeline.common import ROOT, load_sequences, load_system_config, load_systems, run_dir
from pipeline.resources import ResourceSampler, hardware_info


def _git_commit() -> str | None:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
    except (subprocess.SubprocessError, FileNotFoundError):
        return None


def run_one(system: dict, sequence: dict, run_idx: int, data_root: Path, force: bool = False) -> dict:
    out_dir = run_dir(system["slug"], sequence["name"], run_idx)
    meta_file = out_dir / "meta.json"
    if meta_file.exists() and not force:
        return json.loads(meta_file.read_text())
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = load_system_config(system["slug"])
    runner = importlib.import_module(f"pipeline.runners.{system['slug']}")
    meta = {
        "system": system["system"], "slug": system["slug"], "member": system.get("name") or system.get("owner"),
        "sequence": sequence["name"], "run": run_idx,
        "started": dt.datetime.now().isoformat(timespec="seconds"),
        "system_source": cfg.get("source"), "alignment": cfg.get("alignment"),
        "deviations_from_defaults": cfg.get("deviations_from_defaults"),
        "pipeline_commit": _git_commit(), "hardware": hardware_info(),
    }
    t0 = time.perf_counter()
    with ResourceSampler(out_dir / "resources.csv") as sampler:
        try:
            runner.run(sequence, out_dir, run_idx, cfg, data_root)
            meta["success"] = (out_dir / "trajectory_tum.txt").exists()
            meta["error"] = None if meta["success"] else "no trajectory_tum.txt written"
        except Exception as exc:  # a failed run is a result, not a crash of the pipeline
            meta["success"] = False
            meta["error"] = f"{type(exc).__name__}: {exc}"
            (out_dir / "error.log").write_text(traceback.format_exc())
    meta["wall_time_s"] = time.perf_counter() - t0
    meta["resources"] = sampler.summary
    meta_file.write_text(json.dumps(meta, indent=2))
    return meta


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--system", help="slug from team.yaml (default: all)")
    ap.add_argument("--sequence", help="sequence name (default: all)")
    ap.add_argument("--run", type=int, help="single repetition index (default: all N)")
    ap.add_argument("--force", action="store_true", help="re-run even if results exist")
    args = ap.parse_args()

    seq_cfg = load_sequences()
    data_root = ROOT / seq_cfg["data_root"]
    n_runs = seq_cfg["evaluation"]["runs_per_system"]
    systems = [s for s in load_systems() if args.system in (None, s["slug"])]
    sequences = [s for s in seq_cfg["sequences"] if args.sequence in (None, s["name"])]
    runs = [args.run] if args.run is not None else range(n_runs)
    if not systems:
        raise SystemExit("No matching system registered in team.yaml")

    for system in systems:
        for sequence in sequences:
            for k in runs:
                meta = run_one(system, sequence, k, data_root, args.force)
                status = "ok" if meta["success"] else f"FAILED ({meta['error']})"
                print(f"{system['slug']:>20} | {sequence['name']:>16} | run {k:02d} | {status}")


if __name__ == "__main__":
    main()
