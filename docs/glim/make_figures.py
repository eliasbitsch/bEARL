"""Figures for docs/glim/REPORT.md (Philip Stix). Run from the repo root after the pipeline runs:

    python docs/glim/make_figures.py [--sequence Varying-illu01] [--run 0]

Writes docs/glim/fig_*.png and prints the numbers used in the report. Uses paper/ieee.mplstyle.
The lever-arm fit is a DIAGNOSTIC (it uses the ground truth), never a result for the paper.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402


def associate(gt: np.ndarray, est: np.ndarray, max_dt: float = 0.01) -> tuple[np.ndarray, np.ndarray]:
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    idx = np.where(np.abs(gt[idx - 1, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0]), idx - 1, idx)
    ok = np.abs(gt[idx, 0] - est[:, 0]) < max_dt
    return gt[idx[ok]], est[ok]


def umeyama(src: np.ndarray, dst: np.ndarray, scale: bool = False):
    """dst ~ s * R @ src + t (Umeyama 1991)."""
    ms, md = src.mean(0), dst.mean(0)
    X, Y = src - ms, dst - md
    U, D, Vt = np.linalg.svd(Y.T @ X / len(src))
    S = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    s = (D * S.diagonal()).sum() / X.var(0).sum() if scale else 1.0
    return R, md - s * R @ ms, s


def fit_lever_arm(pe: np.ndarray, Re: np.ndarray, pg: np.ndarray, iters: int = 100) -> np.ndarray:
    """Constant offset l (in the estimate's body frame) with pg ~ R (pe + Re l) + t, alternating LSQ."""
    l = np.zeros(3)
    for _ in range(iters):
        R, t, _ = umeyama(pe + Re @ l, pg)
        A = np.einsum("ij,njk->nik", R, Re).reshape(-1, 3)
        l = np.linalg.lstsq(A, (pg - (pe @ R.T + t)).reshape(-1), rcond=None)[0]
    return l


def read_ply_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    body = raw[raw.index(b"end_header\n") + len(b"end_header\n"):]
    return np.frombuffer(body, dtype="<f4").reshape(-1, 3)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequence", default="Varying-illu01")
    ap.add_argument("--run", type=int, default=0)
    a = ap.parse_args()

    plt.style.use(str(ROOT / "paper" / "ieee.mplstyle"))
    plt.rcParams.update({"savefig.format": "png", "savefig.dpi": 200})
    seq_dir = ROOT / load_sequences()["data_root"] / a.sequence
    run_dir = ROOT / "results" / "glim" / a.sequence / f"run_{a.run:02d}"
    G, E = associate(np.loadtxt(seq_dir / "gt_tum.txt"), np.loadtxt(run_dir / "trajectory_tum.txt"))
    t = G[:, 0] - G[0, 0]
    pg, pe = G[:, 1:4], E[:, 1:4]
    Re = Rotation.from_quat(E[:, 4:8]).as_matrix()

    # SE(3) alignment of GLIM onto the GT, as in pipeline/evaluate.py
    R, tr, _ = umeyama(pe, pg)
    pe_al = pe @ R.T + tr
    err = np.linalg.norm(pg - pe_al, axis=1)
    _, _, s_sim3 = umeyama(pe, pg, scale=True)
    # diagnostic: same, after removing a fitted constant marker offset
    lever = fit_lever_arm(pe, Re, pg)
    q = pe + Re @ lever
    R2, t2, _ = umeyama(q, pg)
    err2 = np.linalg.norm(pg - (q @ R2.T + t2), axis=1)
    _, _, s_sim3_2 = umeyama(q, pg, scale=True)
    rmse = lambda e: float(np.sqrt((e ** 2).mean()))
    print(f"ATE RMSE SE(3): {rmse(err):.4f} m (Sim(3) scale {s_sim3:.4f})")
    print(f"lever arm (IMU frame): {np.round(lever, 3)} m, |l| = {np.linalg.norm(lever):.3f} m")
    print(f"ATE RMSE after lever-arm fit: {rmse(err2):.4f} m (Sim(3) scale {s_sim3_2:.4f})")
    print(f"path length GT {np.linalg.norm(np.diff(pg, axis=0), axis=1).sum():.1f} m, "
          f"GLIM {np.linalg.norm(np.diff(pe, axis=0), axis=1).sum():.1f} m")

    # 1) top-down trajectories
    fig, ax = plt.subplots(figsize=(3.5, 3.0))
    ax.plot(pg[:, 0], pg[:, 1], label="Ground truth (mocap)", marker="")
    ax.plot(pe_al[:, 0], pe_al[:, 1], label="GLIM (SE(3)-aligned)", marker="")
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_aspect("equal")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=8)
    fig.savefig(OUT / "fig_trajectory.png"), plt.close(fig)

    # 2) ATE over time, before / after the lever-arm diagnostic
    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    ax.plot(t, err, label=f"as evaluated (RMSE {rmse(err):.3f} m)", marker="")
    ax.plot(t, err2, label=f"marker offset removed (RMSE {rmse(err2):.3f} m)", marker="")
    ax.set_xlabel("time [s]"), ax.set_ylabel("position error [m]")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=8)
    fig.savefig(OUT / "fig_ate_over_time.png"), plt.close(fig)

    # 3) top-down map with trajectory (map frame of GLIM)
    pts = read_ply_xyz(run_dir / "map.ply")
    # horizontal slice between floor and head height (walls, furniture), ceiling and outliers removed
    zf = np.percentile(pts[:, 2], 2)
    keep = (pts[:, 2] > zf + 0.15) & (pts[:, 2] < zf + 2.0)
    lo, hi = np.percentile(pts[keep, :2], [0.5, 99.5], axis=0)
    keep &= np.all((pts[:, :2] >= lo) & (pts[:, :2] <= hi), axis=1)
    fig, ax = plt.subplots(figsize=(3.5, 3.0))
    sc = ax.scatter(pts[keep, 0], pts[keep, 1], c=pts[keep, 2] - zf, s=0.6, cmap="viridis", rasterized=True,
                    marker=".", linewidths=0)
    ax.plot(pe[:, 0], pe[:, 1], color="#D55E00", label="GLIM trajectory", marker="", linewidth=0.8)
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_aspect("equal")
    fig.colorbar(sc, ax=ax, label="height above floor [m]", shrink=0.8)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=8)
    fig.savefig(OUT / "fig_map_topdown.png"), plt.close(fig)
    print(f"map: {len(pts)} points, extent x {np.ptp(pts[:, 0]):.1f} m, y {np.ptp(pts[:, 1]):.1f} m, "
          f"z {np.ptp(pts[:, 2]):.1f} m")


if __name__ == "__main__":
    main()
