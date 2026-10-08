"""Runner for GLIM (Philip Stix).

Copied by tools/register_system.py. Every runner has the same interface, so the
pipeline does not need to know anything about the individual SLAM system.

Contract - after run() returns, `out_dir` must contain:
    trajectory_tum.txt   estimated trajectory, TUM format: "t x y z qx qy qz qw" (t in seconds,
                         same clock as the ground truth), pose of the robot/camera in the map frame
    map.<ext>            the map / reconstruction: .ply or .pcd (point cloud), .pgm + .yaml (grid)
Raise an exception if the system fails (lost tracking, crash). The failure is then recorded
as an unsuccessful run - never silently retry or drop failed runs.

You may run the system any way you like (pip package, Docker, ROS 1/2, CUDA). Run it
OFFLINE so that every frame is processed (accuracy must not depend on machine speed).
"""
from __future__ import annotations

import subprocess
from pathlib import Path


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    """Run GLIM once on one sequence.

    sequence:  entry from config/sequences.yaml (name, difficulty, ground_truth)
    out_dir:   results/<slug>/<sequence>/run_<k>/ (already created)
    run_idx:   repetition index (0..N-1); use it as a random seed if the system has one
    cfg:       config/systems/<slug>.yaml
    data_root: folder with the prepared data of all sequences
    """
    seq_dir = data_root / sequence["name"]

    # TODO: call your SLAM system here, e.g.
    # subprocess.run(["docker", "run", "--rm", "-v", f"{seq_dir}:/data", "-v", f"{out_dir}:/out",
    #                 "my-slam-image", "/data", "/out"], check=True)
    raise NotImplementedError("Implement the runner for GLIM")

    # TODO: convert the system's output to out_dir / "trajectory_tum.txt" and out_dir / "map.<ext>"
