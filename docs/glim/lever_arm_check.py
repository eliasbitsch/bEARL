"""Cross-check: GLIM's fitted mocap-body offset vs. the rig calibration of the reference map (Philip Stix).

    python docs/glim/lever_arm_check.py

Two independent estimates of where GLIM's IMU frame sits in the motion-capture body frame:
  predicted  body_T_imu = body_T_base (config/sequences.yaml: reference_map.body_T_base, estimated by map
             consistency, no SLAM involved) @ base_T_lidar (calibration chain, pipeline/reference_map.py on main)
             @ lidar_T_imu (GLIM's T_lidar_imu, docker/glim/config/config_sensors.json)
  fitted     the constant offset fitted between GLIM's trajectory and the GT (make_figures.fit_lever_arm),
             converted to the IMU origin in the body frame with the predicted rotation
Height (body z) is unobservable for planar motion in both (body_T_base has z = 0, and a vertical offset is
absorbed by the alignment), so only body x and y are compared. Diagnostic only, never a result.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_figures import associate, fit_lever_arm  # noqa: E402
from pipeline.common import load_sequences  # noqa: E402

# M3DGR calibration.md, as in pipeline/reference_map.py (main): mid360 -> camera_imu, and wheel(base) -> camera_imu
R_CIMU_MID360 = np.array([[0.0981574, -0.995106, 0.0113748],
                          [0.514299, 0.0409386, -0.856633],
                          [0.851975, 0.0899349, 0.515801]])
T_CIMU_MID360 = np.array([-0.00349766, -0.417688, 0.198242])
R_CIMU_WHEEL = np.array([[4.2873564019253907e-02, -9.9906999607154057e-01, 4.5826256555663858e-03],
                         [2.3548883729155812e-02, -3.5750257528033291e-03, -9.9971629438855181e-01],
                         [9.9880293731215963e-01, 4.2969316267296165e-02, 2.3373709079293481e-02]])
# GLIM config_sensors.json T_lidar_imu (p_lidar = T_lidar_imu p_imu): translation only, identity rotation
T_LIDAR_IMU = np.array([0.011, 0.02329, -0.04412])


def se3(r: np.ndarray, t) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def predicted_body_T_imu() -> np.ndarray:
    rig = load_sequences()["reference_map"]["body_T_base"]
    body_base = se3(Rotation.from_euler("xyz", [rig["roll_deg"], rig["pitch_deg"], rig["yaw_deg"]], degrees=True)
                    .as_matrix(), [rig["x_m"], rig["y_m"], 0.0])
    r_base_cimu = R_CIMU_WHEEL.T          # reference_map.base_T_lidar: rotation of the chain only
    base_lidar = se3(r_base_cimu @ R_CIMU_MID360, r_base_cimu @ T_CIMU_MID360)
    return body_base @ base_lidar @ se3(np.eye(3), T_LIDAR_IMU)


def main() -> None:
    cfg = load_sequences()
    b_T_i = predicted_body_T_imu()
    pred = b_T_i[:3, 3]
    print(f"predicted IMU origin in body frame: x {pred[0]:+.3f}  y {pred[1]:+.3f}  (z {pred[2]:+.3f}, z not estimated)")
    print(f"{'sequence':<16}{'fitted x':>10}{'fitted y':>10}{'dx':>8}{'dy':>8}{'|dxy|':>8}  [m]")
    for seq in [s["name"] for s in cfg["sequences"]]:
        G0 = np.loadtxt(ROOT / cfg["data_root"] / seq / "gt_tum.txt")
        G, E = associate(G0, np.loadtxt(ROOT / "results" / "glim" / seq / "run_00" / "trajectory_tum.txt"))
        Re = Rotation.from_quat(E[:, 4:8]).as_matrix()
        lever = fit_lever_arm(E[:, 1:4], Re, G[:, 1:4])    # body origin expressed in the IMU frame
        fit = -b_T_i[:3, :3] @ lever                         # IMU origin expressed in the body frame
        d = fit[:2] - pred[:2]
        print(f"{seq:<16}{fit[0]:>+10.3f}{fit[1]:>+10.3f}{d[0]:>+8.3f}{d[1]:>+8.3f}{np.linalg.norm(d):>8.3f}")


if __name__ == "__main__":
    main()
