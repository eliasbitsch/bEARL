"""Map quality against the pseudo-ground-truth map (pipeline/reference_map.py).

    python -m pipeline.map_eval                         # every system with a map.ply in results/<slug>/<seq>/run_00/
    python -m pipeline.map_eval --system vggt_long --sequence Dynamic01

Metrics (Knapitsch et al. 2017, "Tanks and Temples"; Chamfer as in Fan et al. 2017), both clouds thinned to
2 cm voxels first so that point density does not dominate:
  accuracy      distances estimate -> reference (mean, median)          "how correct is what was mapped"
  completeness  distances reference -> estimate (mean, median)          "how much of the room was mapped"
  precision(t)  share of estimate points closer than t to the reference
  recall(t)     share of reference points closer than t to the estimate
  F-score(t)    2 P R / (P + R), t = 5 cm and 10 cm
  Chamfer       mean(est -> ref) + mean(ref -> est)
  (accuracy and completeness are the cloud-to-cloud (C2C) distances of CloudCompare; checked identical on
   Dynamic01 to 0.001 cm with tools/cloudcompare_c2c.py)

Alignment: the map starts in the frame given by the trajectory alignment (Umeyama, Sim(3) for monocular, SE(3)
for metric systems, as in pipeline.evaluate) and is then refined with ICP (with scale for Sim(3)), as in the Tanks
and Temples protocol (rejected and flagged if it moves the map implausibly far). This removes the sensor-to-mocap-body offset, which is not a property of the map; local
distortion cannot be removed by a global transform.

Evaluation regions (same metrics, two volumes, both reported):
  room     the whole reference map (a LiDAR also sees the ceiling: completeness favours it, honestly)
  common   floor .. floor + 2.5 m, the part a forward-looking camera can observe at all
Caveat: the reference is built from the MID-360, so systems using that LiDAR are favoured in accuracy.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from evo.tools import file_interface
from scipy.spatial import cKDTree

from pipeline.common import RESULTS_DIR, ROOT, load_sequences, load_system_config, load_systems
from pipeline.reference_map import write_ply
from pipeline.visualize import _read_ply_xyzrgb, align_full

VOXEL = 0.02
TAUS = (0.05, 0.10)
COMMON_HEIGHT = 2.5
ICP_MAX_ROT_DEG, ICP_MAX_SHIFT_M = 10.0, 0.5   # plausibility limits for the ICP refinement (scale: 0.8 .. 1.25)


def voxel_thin(p: np.ndarray, voxel: float = VOXEL) -> np.ndarray:
    _, first = np.unique(np.floor(p / voxel).astype(np.int64), axis=0, return_index=True)
    return p[first]


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    """Least-squares s, R, t with dst ~ s R src + t (Umeyama 1991)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    u, d, vt = np.linalg.svd(b.T @ a / len(src))
    sgn = np.eye(3)
    sgn[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ sgn @ vt
    s = (d * np.diag(sgn)).sum() / a.var(0).sum() if with_scale else 1.0
    return s, r, mu_d - s * r @ mu_s


def icp(src: np.ndarray, ref_tree: cKDTree, ref: np.ndarray, with_scale: bool,
        max_dists=(0.30, 0.20, 0.10, 0.05), iters: int = 15, sample: int = 200_000, seed: int = 0):
    """Trimmed point-to-point ICP with a shrinking correspondence radius. Returns the transformed cloud and
    the accumulated (s, R, t)."""
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(src), min(sample, len(src)), replace=False)
    s_tot, r_tot, t_tot = 1.0, np.eye(3), np.zeros(3)
    cur = src[idx]
    for md in max_dists:
        for _ in range(iters):
            d, j = ref_tree.query(cur, distance_upper_bound=md)
            ok = np.isfinite(d)
            if ok.sum() < 1000:
                break
            s, r, t = umeyama(cur[ok], ref[j[ok]], with_scale)
            cur = s * cur @ r.T + t
            s_tot, r_tot, t_tot = s * s_tot, r @ r_tot, s * r @ t_tot + t
    return s_tot * src @ r_tot.T + t_tot, (s_tot, r_tot, t_tot)


def scores(est: np.ndarray, ref: np.ndarray) -> dict:
    d_er, _ = cKDTree(ref).query(est)
    d_re, _ = cKDTree(est).query(ref)
    out = {"n_est": len(est), "n_ref": len(ref),
           "accuracy_mean_m": d_er.mean(), "accuracy_median_m": np.median(d_er),
           "completeness_mean_m": d_re.mean(), "completeness_median_m": np.median(d_re),
           "chamfer_m": d_er.mean() + d_re.mean()}
    for tau in TAUS:
        p, r = (d_er < tau).mean(), (d_re < tau).mean()
        k = f"{int(tau * 100)}cm"
        out[f"precision_{k}"], out[f"recall_{k}"] = p, r
        out[f"fscore_{k}"] = 2 * p * r / (p + r) if p + r > 0 else 0.0
    return out


def evaluate_map(slug: str, seq: str, run: str = "run_00", export: bool = False) -> list[dict]:
    cfg = load_sequences()
    run_dir = RESULTS_DIR / slug / seq / run
    sim3 = load_system_config(slug).get("alignment", "se3") == "sim3"
    gt = file_interface.read_tum_trajectory_file(str(ROOT / cfg["data_root"] / seq / "gt_tum.txt"))
    est_traj = file_interface.read_tum_trajectory_file(str(run_dir / "trajectory_tum.txt"))
    _, (s, r, t) = align_full(gt, est_traj, sim3, cfg["evaluation"]["t_max_diff"])

    ref = voxel_thin(_read_ply_xyzrgb(ROOT / cfg["data_root"] / seq / "reference_map.ply")[0])
    raw = s * _read_ply_xyzrgb(run_dir / "map.ply")[0] @ r.T + t        # thin only once, after the final transform
    est = voxel_thin(raw)
    tree = cKDTree(ref)
    raw_icp, (s_i, r_i, t_i) = icp(raw, tree, ref, with_scale=sim3)
    rot_i = float(np.degrees(np.arccos(np.clip((np.trace(r_i) - 1) / 2, -1, 1))))
    # ICP is a refinement: a large correction means it did not converge to the right solution (e.g. a collapsed
    # map shrinks onto a wall with free scale). Then the trajectory alignment is kept and the case is flagged.
    icp_ok = 0.8 < s_i < 1.25 and rot_i < ICP_MAX_ROT_DEG and np.linalg.norm(t_i) < ICP_MAX_SHIFT_M
    est_icp = voxel_thin(raw_icp) if icp_ok else est

    z = ref[:, 2]
    hist, edges = np.histogram(z, bins=200)
    floor = edges[np.argmax(hist[: len(hist) // 2])]           # lower dominant peak = floor
    rows = []
    for region, lo, hi in (("room", -np.inf, np.inf), ("common", floor - 0.05, floor + COMMON_HEIGHT)):
        for stage, e in (("trajectory_aligned", est), ("icp_refined", est_icp)):
            rr = ref[(ref[:, 2] > lo) & (ref[:, 2] < hi)]
            ee = e[(e[:, 2] > lo) & (e[:, 2] < hi)] if region == "common" else e
            rows.append({"slug": slug, "sequence": seq, "run": run, "region": region, "stage": stage,
                         "icp_accepted": bool(icp_ok), "icp_scale": s_i, "icp_rotation_deg": rot_i,
                         "icp_translation_m": float(np.linalg.norm(t_i)), "floor_z_m": float(floor),
                         **scores(ee, rr)})
            if export and stage == "icp_refined":      # the exact clouds compared, e.g. for tools/cloudcompare_c2c.py
                write_ply(run_dir / "map_eval" / f"{region}_est.ply", ee)
                write_ply(run_dir / "map_eval" / f"{region}_ref.ply", rr)
    (run_dir / "map_metrics.json").write_text(json.dumps(rows, indent=2, default=float))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--system")
    ap.add_argument("--sequence")
    ap.add_argument("--export", action="store_true",
                    help="also write the compared clouds to results/<slug>/<seq>/run_00/map_eval/ (CloudCompare check)")
    a = ap.parse_args()
    rows = []
    for system in load_systems():
        if a.system not in (None, system["slug"]):
            continue
        for seq in load_sequences()["sequences"]:
            if a.sequence not in (None, seq["name"]):
                continue
            if (RESULTS_DIR / system["slug"] / seq["name"] / "run_00" / "map.ply").exists():
                rows += evaluate_map(system["slug"], seq["name"], export=a.export)
                print(f"{system['slug']} {seq['name']}: done")
    df = pd.DataFrame(rows)
    csv = RESULTS_DIR / "map_metrics.csv"
    if csv.exists() and not df.empty:              # keep the rows of systems/sequences not evaluated in this call
        old = pd.read_csv(csv)
        done = set(zip(df["slug"], df["sequence"]))
        old = old[[(s, q) not in done for s, q in zip(old["slug"], old["sequence"])]]
        df = pd.concat([old, df], ignore_index=True).sort_values(["slug", "sequence", "region", "stage"])
    df.to_csv(csv, index=False)
    cols = ["slug", "sequence", "region", "stage", "accuracy_median_m", "completeness_median_m", "fscore_5cm",
            "fscore_10cm", "chamfer_m", "icp_accepted", "icp_scale"]
    print(df[cols].round(3).to_string(index=False))


if __name__ == "__main__":
    main()
