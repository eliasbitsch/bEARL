import marimo

__generated_with = "0.26.0"
app = marimo.App(width="medium", app_title="bEARL - compare all systems")


@app.cell
def _():
    import subprocess
    import sys
    from pathlib import Path

    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
    import yaml

    ROOT = Path(__file__).resolve().parents[1]
    RES = ROOT / "results"
    CACHE = RES / "cache"
    DATA = ROOT / "data" / "M3DGR"
    SEQS = ["Dynamic01", "Sha-turn01", "Varying-illu01"]
    return CACHE, DATA, Path, RES, ROOT, SEQS, mo, np, pd, plt, subprocess, sys, yaml


@app.cell
def _(mo):
    mo.md(r"""
    # Compare all SLAM systems

    One page for every member: what is still missing for your system, one click to evaluate it, and all systems
    side by side with the same metrics. Every system is scored the same way:

    | | metric | reference |
    |---|---|---|
    | trajectory | ATE RMSE after Umeyama alignment: **Sim(3)** (with scale) for monocular systems such as VGGT-Long and MASt3R-SLAM, which cannot know metric scale; **SE(3)** for metric systems (LiDAR, RGB-D, IMU) | Sturm 2012, Umeyama 1991, Zhang & Scaramuzza 2018 |
    | map vs. reference | cloud-to-cloud distance (accuracy, completeness; identical to CloudCompare C2C), F-score at 10 cm, after the same alignment + ICP | Knapitsch 2017 (Tanks and Temples) |
    | wall noise | mean plane variance (MPV) of 10 cm neighbourhoods, next to the value of the LiDAR reference itself | Razlaw 2015 |
    | detectability | correct rate (OpenLORIS) and what the system reports itself (log, lost state) | Shi 2020 |

    The alignment is read from `config/systems/<slug>.yaml` (`alignment: sim3` or `se3`), so a monocular map is
    scaled with the same Sim(3) as its trajectory before it is compared.
    """)
    return


@app.cell
def _(ROOT, yaml):
    def load_systems():
        """Every registered system: config/systems/*.yaml (diagnostic variants included, template excluded)."""
        out = {}
        for f in sorted((ROOT / "config" / "systems").glob("*.yaml")):
            if f.stem.startswith("_"):
                continue
            c = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
            out[c.get("slug", f.stem)] = {"system": c.get("system", f.stem), "member": c.get("member", ""),
                                         "alignment": c.get("alignment", "?")}
        return out

    systems = load_systems()
    MAIN = ["vggt_long", "mast3r_slam", "glim"]   # add the slugs of Jiayi and Viktoriia once registered
    return MAIN, systems


@app.cell
def _(RES, ROOT, SEQS, pd, systems):
    def status_table():
        metrics = pd.read_csv(RES / "metrics.csv") if (RES / "metrics.csv").exists() else pd.DataFrame(columns=["slug"])
        maps = pd.read_csv(RES / "map_metrics.csv") if (RES / "map_metrics.csv").exists() else pd.DataFrame(columns=["slug"])
        rows = []
        for slug, s in systems.items():
            r = {"slug": slug, "system": s["system"], "member": s["member"], "alignment": s["alignment"]}
            for seq in SEQS:
                runs = sorted((RES / slug / seq).glob("run_*/trajectory_tum.txt"))
                r[f"runs {seq[:3]}"] = len(runs)
            r["map.ply (run_00)"] = sum((RES / slug / q / "run_00" / "map.ply").exists() for q in SEQS)
            r["in metrics.csv"] = int((metrics.slug == slug).sum())
            r["in map_metrics.csv"] = int((maps.slug == slug).any()) if len(maps) else 0
            rows.append(r)
        return pd.DataFrame(rows).set_index("slug")

    status = status_table()
    members_tex = {p.stem: ("\\todo" not in p.read_text(encoding="utf-8")) for p in (ROOT / "paper" / "members").glob("*.tex")}
    return members_tex, status


@app.cell
def _(DATA, members_tex, mo, status):
    _missing_data = not (DATA / "Dynamic01" / "gt_tum.txt").exists()
    mo.vstack([
        mo.md(r"""
        ## 1. Status - what is missing?

        Target per system: 5 runs per sequence (`results/<slug>/<sequence>/run_NN/trajectory_tum.txt`, TUM format,
        dataset timestamps), `map.ply` in `run_00` of each sequence (binary PLY, in the system's own frame), a row in
        both metric files, and a filled `paper/members/<name>.tex`.
        """),
        status,
        mo.md("Paper text filled in (no `\\todo` left): " +
              ", ".join(f"`{k}` {'yes' if v else '**no**'}" for k, v in members_tex.items())),
        mo.callout(mo.md("`data/M3DGR` not found: the buttons below need the dataset (ground truth, reference maps). "
                         "The tables and plots work without it."), kind="warn") if _missing_data else mo.md(""),
    ])
    return


@app.cell
def _(mo, systems):
    eval_slug = mo.ui.dropdown(sorted(systems), value="glim" if "glim" in systems else None, label="System")
    btn_traj = mo.ui.run_button(label="1. Evaluate trajectories (all systems -> metrics.csv)")
    btn_map = mo.ui.run_button(label="2. Evaluate the map of this system -> map_metrics.csv")
    btn_cache = mo.ui.run_button(label="3. Rebuild the plot cache (results/cache)")
    mo.vstack([
        mo.md("## 2. Evaluate\nRun these after you copied new runs into `results/`. They call the shared pipeline, "
              "so every system is scored by the same code."),
        eval_slug, mo.hstack([btn_traj, btn_map, btn_cache], justify="start"),
    ])
    return btn_cache, btn_map, btn_traj, eval_slug


@app.cell
def _(ROOT, btn_cache, btn_map, btn_traj, eval_slug, mo, subprocess, sys):
    def run(cmd):
        p = subprocess.run([sys.executable, *cmd], cwd=ROOT, capture_output=True, text=True)
        return f"$ python {' '.join(cmd)}\n" + (p.stdout + p.stderr)[-4000:]

    _out = []
    if btn_traj.value:
        _out.append(run(["-m", "pipeline.evaluate"]))
    if btn_map.value and eval_slug.value:
        _out.append(run(["-m", "pipeline.map_eval", "--system", eval_slug.value]))
    if btn_cache.value:
        _out.append(run(["tools/build_cache.py"]))
    mo.md("```\n" + "\n\n".join(_out) + "\n```") if _out else mo.md("*Press a button to run a step; re-run the "
                                                                       "cells below afterwards (they read the CSVs).*")
    return


@app.cell
def _(MAIN, RES, SEQS, mo, pd, systems):
    show = mo.ui.multiselect(sorted(systems), value=[s for s in MAIN if s in systems], label="Systems to compare")

    def compare_table(slugs):
        m = pd.read_csv(RES / "metrics.csv")
        mm = pd.read_csv(RES / "map_metrics.csv")
        mm = mm[(mm.region == "common") & (mm.stage == "icp_refined") & (mm.run == "run_00")]
        rows = []
        for slug in slugs:
            for seq in SEQS:
                a = m[(m.slug == slug) & (m.sequence == seq)].ate_rmse_m
                b = mm[(mm.slug == slug) & (mm.sequence == seq)]
                rows.append({
                    "system": systems[slug]["system"], "sequence": seq, "align": systems[slug]["alignment"],
                    "runs": len(a), "ATE median [m]": a.median() if len(a) else None,
                    "ATE IQR [mm]": 1000 * (a.quantile(0.75) - a.quantile(0.25)) if len(a) else None,
                    "F-score @10cm": b.fscore_10cm.iloc[0] if len(b) else None,
                    "accuracy C2C [cm]": 100 * b.accuracy_median_m.iloc[0] if len(b) else None,
                    "completeness C2C [cm]": 100 * b.completeness_median_m.iloc[0] if len(b) else None,
                    "MPV [mm2] (ref)": (f"{1e6 * b.mpv_est_m2.iloc[0]:.0f} ({1e6 * b.mpv_ref_m2.iloc[0]:.0f})"
                                        if len(b) else None),
                })
        return pd.DataFrame(rows).set_index(["system", "sequence"])

    mo.md("## 3. Side by side")
    return compare_table, show


@app.cell
def _(compare_table, mo, show):
    mo.vstack([show, compare_table(show.value).round(3),
               mo.md("ATE: median over the runs. Maps: run 0, floor to 2.5 m above the floor, after alignment + ICP. "
                     "MPV in mm²; the LiDAR reference itself is the floor in brackets (lower = flatter walls).")])
    return


@app.cell
def _(CACHE, SEQS, mo, pd, plt, show, systems):
    def fig_traj(slugs):
        fig, axes = plt.subplots(1, 3, figsize=(10, 3), sharex=True, sharey=True)
        for ax, s in zip(axes, SEQS):
            g = pd.read_csv(CACHE / f"gt_{s}.csv")
            ax.plot(g.x, g.y, color="0.7", lw=1.5, label="ground truth")
            for slug in slugs:
                f = CACHE / f"traj_{slug}_{s}.csv"
                if f.exists():
                    d = pd.read_csv(f)
                    ax.plot(d.est_x, d.est_y, lw=0.8, label=systems[slug]["system"])
            ax.set_title(s)
            ax.set_aspect("equal")
        axes[0].legend(fontsize=7, loc="lower left")
        fig.tight_layout()
        return fig

    mo.vstack([mo.md("Trajectories after alignment (from `results/cache`; press button 3 after adding a system "
                     "to `SYSTEMS` in `tools/build_cache.py`)."), fig_traj(show.value)])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. Checklist for a new system

    1. `python tools/register_system.py --member "..." --system "..." --slug ... --alignment se3|sim3 ...`
       (fills `team.yaml`, creates `config/systems/<slug>.yaml` and `pipeline/runners/<slug>.py`).
    2. Run with the authors' defaults; put each run in `results/<slug>/<sequence>/run_NN/`:
       `trajectory_tum.txt` (timestamp x y z qx qy qz qw, dataset time), `map.ply` (run_00 is enough),
       the system's own log (`<slug>.log`; needed for RQ3: does it report being lost?).
    3. Press buttons 1 and 2 above, add the slug to `SYSTEMS` in `tools/build_cache.py`, press 3.
    4. Write 1-2 sentences per section in `paper/members/<name>.tex` (state of the art, methods, results per RQ).
    5. Commit on your own branch and open a pull request; never edit `paper/main.tex` directly.
    """)
    return


if __name__ == "__main__":
    app.run()
