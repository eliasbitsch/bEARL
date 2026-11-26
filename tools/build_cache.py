"""Small, git-tracked derivatives of the dataset and the maps, so notebooks/story.py runs without data/ or a GPU.

Needs data/M3DGR (images, GT, reference maps) and the map.ply files of the runs. Run once after new results:
    python tools/build_cache.py
Writes results/cache/: light_<seq>.csv, gt_<seq>.csv, traj_<slug>_<seq>.csv, thumbs/<seq>/*.jpg, slices_<seq>.npz
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from evo.core import sync
from evo.tools import file_interface

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402
from pipeline.visualize import _read_ply_xyzrgb, align_full  # noqa: E402

CFG = load_sequences()
DATA = ROOT / CFG["data_root"]
SEQS = [s["name"] for s in CFG["sequences"]]
OUT = ROOT / "results" / "cache"
SYSTEMS = {"vggt_long": True, "mast3r_slam": True, "fast_lio2": False}   # slug -> Sim(3) alignment
BLACK = 20            # grey value below which a pixel counts as near-black (same as 00_story)
THUMB_STEP = 15       # every 15th frame = 2 Hz
SLICE_Z = (0.9, 1.3)  # wall slice above furniture, same as the paper's map overlay
VOXEL = 0.02


METRICS = pd.read_csv(ROOT / "results" / "metrics.csv")


def rep_run(slug: str, seq: str) -> str:
    """Representative run = ATE closest to the median (MASt3R-SLAM is not deterministic on Varying-illu01)."""
    m = METRICS[(METRICS.slug == slug) & (METRICS.sequence == seq)]
    return m.loc[(m.ate_rmse_m - m.ate_rmse_m.median()).abs().idxmin(), "run"] if len(m) else "run_00"


def light(seq: str) -> pd.DataFrame:
    rows = [l.split() for l in (DATA / seq / "rgb.txt").read_text().splitlines() if not l.startswith("#")]
    t0 = float(rows[0][0])
    thumbs = OUT / "thumbs" / seq
    thumbs.mkdir(parents=True, exist_ok=True)
    out = []
    for k, (t, f) in enumerate(rows):
        img = cv2.imread(str(DATA / seq / f))
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        out.append((k, float(t), float(t) - t0, g.mean(), (g < BLACK).mean()))
        if k % THUMB_STEP == 0:
            cv2.imwrite(str(thumbs / f"{k:05d}.jpg"), cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA),
                        [cv2.IMWRITE_JPEG_QUALITY, 70])
    return pd.DataFrame(out, columns=["frame", "t", "t_rel", "brightness", "black_fraction"])


def trajectories(seq: str, gt) -> None:
    t0 = gt.timestamps[0]
    keep = np.arange(0, gt.num_poses, 10)                   # 300 Hz -> 30 Hz is enough for plots
    pd.DataFrame({"t_rel": gt.timestamps[keep] - t0, **dict(zip("xyz", gt.positions_xyz[keep].T))}).round(4) \
        .to_csv(OUT / f"gt_{seq}.csv", index=False)
    for slug, sim3 in SYSTEMS.items():
        f = ROOT / "results" / slug / seq / rep_run(slug, seq) / "trajectory_tum.txt"
        if not f.exists():
            continue
        est, _ = align_full(gt, file_interface.read_tum_trajectory_file(str(f)), sim3, CFG["evaluation"]["t_max_diff"])
        r, e = sync.associate_trajectories(gt, est, max_diff=CFG["evaluation"]["t_max_diff"])
        d = {"t_rel": r.timestamps - t0, **{f"gt_{a}": r.positions_xyz[:, i] for i, a in enumerate("xyz")},
             **{f"est_{a}": e.positions_xyz[:, i] for i, a in enumerate("xyz")},
             "error_m": np.linalg.norm(r.positions_xyz - e.positions_xyz, axis=1)}
        pd.DataFrame(d).round(4).to_csv(OUT / f"traj_{slug}_{seq}.csv", index=False)


def comparisons(seq: str, gt) -> dict:
    """Same-timestamp comparison and a trivial baseline (review: MASt3R-SLAM is scored on keyframes only).

    vggt_on_mast3r_kf_m: VGGT-Long ATE on the MASt3R-SLAM keyframe timestamps (nearest 10 Hz frame within 60 ms),
    Sim(3)-aligned on that subset. const_baseline_m: ATE of a constant position, i.e. the RMS distance of the ground
    truth to its centroid (Sim(3) alignment of a constant estimate collapses to the centroid).
    """
    rmse = lambda e: float(np.sqrt(np.mean(np.square(e))))
    p = gt.positions_xyz
    out = {"sequence": seq, "const_baseline_m": rmse(np.linalg.norm(p - p.mean(0), axis=1))}
    m = file_interface.read_tum_trajectory_file(
        str(ROOT / "results" / "mast3r_slam" / seq / rep_run("mast3r_slam", seq) / "trajectory_tum.txt"))
    v = file_interface.read_tum_trajectory_file(
        str(ROOT / "results" / "vggt_long" / seq / rep_run("vggt_long", seq) / "trajectory_tum.txt"))
    vs, _ = sync.associate_trajectories(v, m, max_diff=0.06)
    vs, _ = align_full(gt, vs, True, CFG["evaluation"]["t_max_diff"])
    r, e = sync.associate_trajectories(gt, vs, max_diff=CFG["evaluation"]["t_max_diff"])
    out.update(vggt_on_mast3r_kf_m=rmse(np.linalg.norm(r.positions_xyz - e.positions_xyz, axis=1)),
               n_common=r.num_poses)
    return out


def voxel(p: np.ndarray) -> np.ndarray:
    _, i = np.unique(np.floor(p / VOXEL).astype(np.int64), axis=0, return_index=True)
    return p[i]


def slices(seq: str, gt) -> None:
    cut = lambda p: voxel(p[(p[:, 2] > SLICE_Z[0]) & (p[:, 2] < SLICE_Z[1])][:, :2].astype(np.float32))
    out = {"reference": cut(_read_ply_xyzrgb(DATA / seq / "reference_map.ply")[0])}
    for slug, sim3 in SYSTEMS.items():
        cands = [ROOT / "results" / slug / seq / rep_run(slug, seq)]
        cands += sorted((ROOT / "results" / slug / seq).glob("run_*"))   # deterministic runs: any run with a map
        run = next((c for c in cands if (c / "map.ply").exists()), None)
        if run is None:
            continue
        est = file_interface.read_tum_trajectory_file(str(run / "trajectory_tum.txt"))
        _, (s, R, t) = align_full(gt, est, sim3, CFG["evaluation"]["t_max_diff"])
        p = _read_ply_xyzrgb(run / "map.ply")[0][::2]
        out[slug] = cut((s * p) @ R.T + t)
    np.savez_compressed(OUT / f"slices_{seq}.npz", **out)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    comp = []
    for seq in SEQS:
        gt = file_interface.read_tum_trajectory_file(str(DATA / seq / "gt_tum.txt"))
        light(seq).round(4).to_csv(OUT / f"light_{seq}.csv", index=False)
        trajectories(seq, gt)
        slices(seq, gt)
        comp.append(comparisons(seq, gt))
        print(seq, "done")
    pd.DataFrame(comp).round(4).to_csv(OUT / "comparisons.csv", index=False)
    pd.DataFrame([(s, q, rep_run(s, q)) for s in SYSTEMS for q in SEQS], columns=["slug", "sequence", "run"]) \
        .to_csv(OUT / "runs.csv", index=False)


if __name__ == "__main__":
    main()
