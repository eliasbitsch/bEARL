"""Compute the standard trajectory metrics for every run in results/.

Metrics (no self-invented metrics - see docs/PLAN.md, Section 5):
  * ATE  - absolute trajectory error, translational RMSE etc.        (Sturm et al. 2012)
  * Alignment: Umeyama (1991); SE(3) for metric systems, Sim(3) for monocular systems
    (Zhang & Scaramuzza 2018). Every run is ALSO evaluated with Sim(3) as a transparency check.
  * Robustness: success flag and fraction of the GT duration covered by the estimate
    (ORB-SLAM3 protocol: report failures, never drop them).

    python -m pipeline.evaluate          ->  results/metrics.csv and results/summary.csv
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

from pipeline.common import RESULTS_DIR, ROOT, load_sequences, load_system_config, load_systems


def _ape(ref: PoseTrajectory3D, est: PoseTrajectory3D, correct_scale: bool) -> tuple[dict, float]:
    est = copy.deepcopy(est)
    _, _, scale = est.align(ref, correct_scale=correct_scale)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref, est))
    return ape.get_all_statistics(), float(scale)



def evaluate_run(gt: PoseTrajectory3D, est_file: Path, alignment: str, ev: dict) -> dict:
    est = file_interface.read_tum_trajectory_file(str(est_file))
    ref, est = sync.associate_trajectories(gt, est, max_diff=ev["t_max_diff"])
    sim3 = alignment == "sim3"

    ate, scale = _ape(ref, est, correct_scale=sim3)
    ate_sim3, scale_sim3 = _ape(ref, est, correct_scale=True)

    gt_duration = gt.timestamps[-1] - gt.timestamps[0]
    tracked = (ref.timestamps[-1] - ref.timestamps[0]) / gt_duration if ref.num_poses > 1 else 0.0
    return {
        "matched_poses": ref.num_poses,
        "tracked_fraction": float(np.clip(tracked, 0.0, 1.0)),
        "ate_rmse_m": ate["rmse"], "ate_mean_m": ate["mean"], "ate_median_m": ate["median"],
        "ate_std_m": ate["std"], "ate_max_m": ate["max"],
        "scale": scale,                                   # 1.0 for SE(3)
        "ate_rmse_sim3_m": ate_sim3["rmse"], "scale_sim3": scale_sim3,   # transparency check
    }


def collect() -> pd.DataFrame:
    seq_cfg = load_sequences()
    ev = seq_cfg["evaluation"]
    rows = []
    for system in load_systems():
        alignment = load_system_config(system["slug"]).get("alignment", "se3")
        for seq in seq_cfg["sequences"]:
            gt_path = ROOT / seq["ground_truth"]
            if not gt_path.exists():
                print(f"[skip] ground truth missing: {gt_path}")
                continue
            gt = file_interface.read_tum_trajectory_file(str(gt_path))
            for run_path in sorted((RESULTS_DIR / system["slug"] / seq["name"]).glob("run_*")):
                meta_file = run_path / "meta.json"
                meta = json.loads(meta_file.read_text()) if meta_file.exists() else {}
                row = {"system": system["system"], "slug": system["slug"], "sequence": seq["name"],
                       "difficulty": seq["difficulty"], "run": run_path.name, "alignment": alignment,
                       "success": bool(meta.get("success", False)),
                       "wall_time_s": meta.get("wall_time_s"),
                       **{k: v for k, v in (meta.get("resources") or {}).items()}}
                est_file = run_path / "trajectory_tum.txt"
                if row["success"] and est_file.exists():
                    try:
                        row.update(evaluate_run(gt, est_file, alignment, ev))
                    except Exception as exc:  # e.g. no overlapping timestamps
                        row["success"] = False
                        row["eval_error"] = f"{type(exc).__name__}: {exc}"
                rows.append(row)
    return pd.DataFrame(rows)


def summarise(df: pd.DataFrame) -> pd.DataFrame:
    """Median and IQR over runs, success k/N - per system and sequence."""
    def iqr(x):
        return x.quantile(0.75) - x.quantile(0.25)

    ok = df[df["success"]]
    agg = ok.groupby(["system", "sequence"]).agg(
        ate_rmse_median_m=("ate_rmse_m", "median"), ate_rmse_iqr_m=("ate_rmse_m", iqr),
        tracked_fraction_median=("tracked_fraction", "median"), scale_median=("scale", "median"),
    )
    counts = df.groupby(["system", "sequence"])["success"].agg(successes="sum", runs="count")
    return counts.join(agg).reset_index()


def main() -> None:
    df = collect()
    if df.empty:
        raise SystemExit("No runs found in results/ - run `python -m pipeline.run_all` first")
    df.to_csv(RESULTS_DIR / "metrics.csv", index=False)
    summary = summarise(df)
    summary.to_csv(RESULTS_DIR / "summary.csv", index=False)
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
