"""Register your SLAM system in one step.

    python tools/register_system.py --member "Philip Stix" --system "My SLAM" --slug my_slam \
        --sensors lidar3d imu --alignment se3 --paper mykey2024 --loop-closure true

What it does:
  * fills in your entry in team.yaml (comments are kept)
  * creates config/systems/<slug>.yaml   from config/systems/_template.yaml
  * creates pipeline/runners/<slug>.py   from pipeline/runners/_template.py
Afterwards, complete the TODOs in those two files and add your paper to paper/references.bib.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SENSORS = {"lidar3d", "imu", "wheel_odom", "rgb", "depth"}


def fill_team_entry(member: str, values: dict) -> None:
    path = ROOT / "team.yaml"
    text = path.read_text(encoding="utf-8")
    start = text.find(f"- name: {member}")
    if start < 0:
        raise SystemExit(f'"{member}" not found in team.yaml - check the spelling of your name')
    nxt = re.search(r"\n\s*- name: |\n\S", text[start + 1:])
    end = start + 1 + nxt.start() if nxt else len(text)
    block = text[start:end]
    for key, val in values.items():
        block = re.sub(rf"(\n\s*{key}:)[^\n]*", lambda m: f"{m.group(1)} {val}", block, count=1)
    path.write_text(text[:start] + block + text[end:], encoding="utf-8")


def from_template(src: Path, dst: Path, repl: dict) -> None:
    if dst.exists():
        print(f"[keep] {dst.relative_to(ROOT)} already exists")
        return
    text = src.read_text(encoding="utf-8")
    for k, v in repl.items():
        text = text.replace(k, v)
    dst.write_text(text, encoding="utf-8")
    print(f"[new]  {dst.relative_to(ROOT)}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--member", required=True, help='your name exactly as in team.yaml')
    ap.add_argument("--system", required=True, help='official name, e.g. "My SLAM"')
    ap.add_argument("--slug", required=True, help="lowercase id, e.g. my_slam")
    ap.add_argument("--sensors", nargs="+", default=[], help=f"subset of {sorted(SENSORS)}")
    ap.add_argument("--alignment", choices=["se3", "sim3"], default="se3",
                    help="se3 = metric system, sim3 = monocular (scale-ambiguous)")
    ap.add_argument("--paper", default="null", help="BibTeX key of the system's paper")
    ap.add_argument("--loop-closure", choices=["true", "false"], default="null")
    ap.add_argument("--hardware", default="null", help='e.g. "Ryzen 7 5800H, 16 GB, no GPU"')
    a = ap.parse_args()

    if not re.fullmatch(r"[a-z][a-z0-9_]*", a.slug):
        raise SystemExit("slug must be lowercase letters, digits and underscores")
    if bad := set(a.sensors) - SENSORS:
        raise SystemExit(f"unknown sensors {bad}; allowed: {sorted(SENSORS)}")

    fill_team_entry(a.member, {
        "slug": a.slug, "system": a.system, "paper": a.paper,
        "sensors": "[" + ", ".join(a.sensors) + "]", "alignment": a.alignment,
        "loop_closure": a.loop_closure, "hardware": a.hardware,
    })
    print("[edit] team.yaml")
    repl = {"__SLUG__": a.slug, "__SYSTEM__": a.system, "__MEMBER__": a.member}
    from_template(ROOT / "config/systems/_template.yaml", ROOT / f"config/systems/{a.slug}.yaml", repl)
    from_template(ROOT / "pipeline/runners/_template.py", ROOT / f"pipeline/runners/{a.slug}.py", repl)
    print("Next: complete the TODOs in the two new files, then run "
          f"`python -m pipeline.run_all --system {a.slug} --sequence <name> --run 0`")


if __name__ == "__main__":
    main()
