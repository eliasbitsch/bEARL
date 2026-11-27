"""GLIM ATE with the ground-truth frame corrected by the rig calibration, and the dark/lit split (Philip Stix).

    python docs/glim/gt_frame_check.py

Corrected: GLIM's IMU poses are moved to the motion-capture body frame with the calibration chain of
lever_arm_check.py (body_T_base from the reference map @ base_T_lidar @ GLIM's lidar_T_imu), nothing is fitted
on the ground truth: T_world_body_est = T_world_imu_est @ inv(body_T_imu). Optionally the LiDAR-mocap time offset
of the reference map (config/sequences.yaml: reference_map.time_offset_s, GT time = sensor time + offset).
Then SE(3) Umeyama alignment and ATE RMSE on poses matched within 10 ms, as in pipeline/evaluate.py.

Dark/lit: a pose is dark if the camera frame at that time has > 80 % near-black pixels
(results/cache/light_<seq>.csv from tools/build_cache.py on main), as in notebooks/story.py.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from lever_arm_check import predicted_body_T_imu  # noqa: E402
from make_figures import associate, umeyama  # noqa: E402
from pipeline.common import load_sequences  # noqa: E402

DARK = 0.8
rmse = lambda e: float(np.sqrt(np.mean(np.square(e)))) if len(e) else float("nan")


def errors(G0: np.ndarray, E: np.ndarray, body_T_imu: np.ndarray | None, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Per-pose position error after SE(3) alignment; returns (GT timestamps, errors)."""
    E = E.copy()
    E[:, 0] += dt
    if body_T_imu is not None:
        imu_T_body = np.linalg.inv(body_T_imu)
        E[:, 1:4] += Rotation.from_quat(E[:, 4:8]).apply(imu_T_body[:3, 3])
    G, E = associate(G0, E)
    R, t, _ = umeyama(E[:, 1:4], G[:, 1:4])
    return G[:, 0], np.linalg.norm(G[:, 1:4] - (E[:, 1:4] @ R.T + t), axis=1)


def main() -> None:
    cfg = load_sequences()
    b_T_i = predicted_body_T_imu()
    offsets = cfg["reference_map"]["time_offset_s"]
    print("ATE RMSE [m], median over runs: as evaluated | corrected frame | corrected frame + time offset")
    split = {}
    for seq in [s["name"] for s in cfg["sequences"]]:
        G0 = np.loadtxt(ROOT / cfg["data_root"] / seq / "gt_tum.txt")
        light_csv = ROOT / "results" / "cache" / f"light_{seq}.csv"
        light = np.genfromtxt(light_csv, delimiter=",", names=True) if light_csv.exists() else None
        res = {"raw": [], "frame": [], "frame_dt": []}
        for run in sorted((ROOT / "results" / "glim" / seq).glob("run_*")):
            E = np.loadtxt(run / "trajectory_tum.txt")
            for key, bti, dt in (("raw", None, 0.0), ("frame", b_T_i, 0.0), ("frame_dt", b_T_i, offsets[seq])):
                t, e = errors(G0, E, bti, dt)
                dk = (np.interp(t - G0[0, 0], light["t_rel"], light["black_fraction"]) > DARK
                      if light is not None else np.zeros(len(e), bool))
                res[key].append((rmse(e), rmse(e[~dk]), rmse(e[dk]), int(dk.sum()), len(e)))
        med = {k: np.median(np.array(v), axis=0) for k, v in res.items()}
        spread = {k: 1000 * np.ptp(np.array(v)[:, 0]) for k, v in res.items()}
        print(f"  {seq:<15} {med['raw'][0]:.4f} | {med['frame'][0]:.4f} | {med['frame_dt'][0]:.4f}   "
              f"(spread over runs {spread['raw']:.2f} / {spread['frame']:.2f} / {spread['frame_dt']:.2f} mm)")
        split[seq] = med
    print("\nDark/lit split (median over runs), ATE RMSE [m]: lit | dark  (dark poses / all)")
    for seq, med in split.items():
        if med["raw"][3] == 0:
            print(f"  {seq:<15} no dark poses")
            continue
        print(f"  {seq:<15} as evaluated {med['raw'][1]:.4f} | {med['raw'][2]:.4f}    "
              f"corrected frame + time offset {med['frame_dt'][1]:.4f} | {med['frame_dt'][2]:.4f}    "
              f"({int(med['raw'][3])} / {int(med['raw'][4])})")


if __name__ == "__main__":
    main()
