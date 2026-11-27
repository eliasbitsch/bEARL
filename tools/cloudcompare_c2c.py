"""Cross-check map_eval's distances with CloudCompare's cloud-to-cloud (C2C) distance, headless.

    python -m pipeline.map_eval --system vggt_long --sequence Dynamic01 --export
    python tools/cloudcompare_c2c.py results/vggt_long/Dynamic01/run_00/map_eval common

Runs `CloudCompare -SILENT ... -C2C_DIST` in both directions (estimate -> reference = accuracy,
reference -> estimate = completeness) and prints mean and median next to the values in map_metrics.json.
CloudCompare executable: $CLOUDCOMPARE, else the default install paths (Windows) or `CloudCompare` on PATH.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def find_cloudcompare() -> str:
    for c in (os.environ.get("CLOUDCOMPARE"), r"C:\Program Files\CloudCompare\CloudCompare.exe",
              str(Path.home() / "tools" / "cloudcompare" / "app" / "CloudCompare.exe"), shutil.which("CloudCompare")):
        if c and Path(c).exists():
            return c
    raise SystemExit("CloudCompare not found - set CLOUDCOMPARE to the executable")


def c2c(cc: str, compared: Path, reference: Path) -> np.ndarray:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "c2c.asc"
        subprocess.run([cc, "-SILENT", "-NO_TIMESTAMP", "-AUTO_SAVE", "OFF", "-C_EXPORT_FMT", "ASC",
                        "-O", str(compared), "-O", str(reference), "-C2C_DIST", "-POP_CLOUDS",
                        "-SAVE_CLOUDS", "FILE", str(out)], check=True, capture_output=True)
        return np.loadtxt(out, usecols=3)          # x y z C2C


def main() -> None:
    folder, region = Path(sys.argv[1]), sys.argv[2] if len(sys.argv) > 2 else "common"
    cc = find_cloudcompare()
    est, ref = folder / f"{region}_est.ply", folder / f"{region}_ref.ply"
    ours = next(r for r in json.loads((folder.parent / "map_metrics.json").read_text())
                if r["region"] == region and r["stage"] == "icp_refined")
    for name, d, key in (("accuracy (est -> ref)", c2c(cc, est, ref), "accuracy"),
                         ("completeness (ref -> est)", c2c(cc, ref, est), "completeness")):
        print(f"{name}: CloudCompare mean {d.mean() * 100:.3f} cm, median {np.median(d) * 100:.3f} cm | "
              f"map_eval mean {ours[key + '_mean_m'] * 100:.3f} cm, median {ours[key + '_median_m'] * 100:.3f} cm")


if __name__ == "__main__":
    main()
