"""Turn the raw output of a VGGT-Long diagnostic run (cluster/run_vggt_long_{lit_only,noloop}.sh) into a result run.

    python tools/import_diagnostic.py --slug vggt_long_lit_only --src <dir> --frames cluster/lit_frames_varying.txt
    python tools/import_diagnostic.py --slug vggt_long_noloop --src <dir>          # all 10 Hz frames

<dir> holds camera_poses.txt, log.txt and frame_confidence.csv as written by docker/vggt_long/run.sh. Writes
results/<slug>/Varying-illu01/run_00/{trajectory_tum.txt, vggt_long/...} and prints the ATE (pipeline.evaluate).
Afterwards: python tools/build_cache.py.
"""
import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402
from pipeline.evaluate import evaluate_run  # noqa: E402

SEQ = "Varying-illu01"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slug", required=True)
    ap.add_argument("--src", type=Path, required=True)
    ap.add_argument("--frames", type=Path, help="image list the run used; default: all of data/M3DGR/<seq>/rgb_step3")
    a = ap.parse_args()

    cfg = load_sequences()
    if a.frames:
        names = sorted(l.strip() for l in a.frames.read_text().splitlines() if l.strip())
    else:
        names = sorted(p.name for p in (ROOT / cfg["data_root"] / SEQ / "rgb_step3").iterdir())
    poses = np.loadtxt(a.src / "camera_poses.txt").reshape(-1, 4, 4)
    assert len(poses) == len(names), (len(poses), len(names))
    out = ROOT / "results" / a.slug / SEQ / "run_00"
    (out / "vggt_long").mkdir(parents=True, exist_ok=True)
    stamps = np.array([float(Path(n).stem) for n in names])
    q = Rotation.from_matrix(poses[:, :3, :3]).as_quat()   # x, y, z, w = TUM order
    np.savetxt(out / "trajectory_tum.txt", np.column_stack([stamps, poses[:, :3, 3], q]), fmt="%.6f")
    for f in ("log.txt", "frame_confidence.csv", "config_used.yaml"):
        if (a.src / f).exists():
            shutil.copy(a.src / f, out / "vggt_long" / f)

    gt = file_interface.read_tum_trajectory_file(str(ROOT / cfg["data_root"] / SEQ / "gt_tum.txt"))
    r = evaluate_run(gt, out / "trajectory_tum.txt", "sim3", cfg["evaluation"])
    print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == "__main__":
    main()
