import marimo

__generated_with = "0.26.0"
app = marimo.App(width="medium", app_title="bEARL - silent vs. visible failure")


@app.cell
def _():
    import re
    import sys
    from pathlib import Path

    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
    from cycler import cycler

    ROOT = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(ROOT))
    RES = ROOT / "results"
    CACHE = RES / "cache"
    FIG = ROOT / "paper" / "figures"
    plt.style.use(ROOT / "paper" / "ieee.mplstyle")
    # the paper style cycles markers; dense time series become blobs with them
    plt.rcParams["axes.prop_cycle"] = cycler(color=["#0072B2", "#D55E00", "#009E73", "#000000"]) + cycler(
        linestyle=["-", "--", "-.", ":"])
    SEQS = ["Dynamic01", "Sha-turn01", "Varying-illu01"]
    CORE = {"vggt_long": "VGGT-Long", "mast3r_slam": "MASt3R-SLAM"}
    STYLE = {"vggt_long": dict(color="#D55E00", ls="-"), "mast3r_slam": dict(color="#0072B2", ls="--"),
             "gt": dict(color="0.65", ls="-")}
    DARK = 0.8   # frame counts as dark when more than 80 % of its pixels are near-black
    return CACHE, CORE, DARK, FIG, Path, RES, SEQS, STYLE, mo, np, pd, plt, re


@app.cell
def _(mo):
    mo.md(r"""
    # When the light goes out: silent vs. visible failure of foundation-model SLAM

    Two monocular SLAM systems built on 3D foundation models run on the same M3DGR ground-robot sequences:
    **VGGT-Long** (chunks of VGGT predictions joined by Sim(3), loop closure) and **MASt3R-SLAM** (MASt3R on image
    pairs, keyframe graph, relocalisation). Two sequences are well lit, in `Varying-illu01` the light is switched on
    and off.

    - **RQ1 - accuracy:** how accurate are trajectory and map in normal light?
    - **RQ2 - failure mode:** what happens when the light switches on and off?
    - **RQ3 - detectability:** does the run show that it failed, or does it look complete while it is wrong?

    This notebook only reads files in the repository (`results/`, `results/cache/`); it runs no SLAM system and
    needs no GPU and no dataset. SLAM runs: `pipeline/run_all.py` (ADA cluster), metrics: `pipeline/evaluate.py`,
    `pipeline/map_eval.py`, small derivatives of images and maps: `tools/build_cache.py`.
    """)
    return


@app.cell
def _(CACHE, RES, SEQS, pd):
    metrics = pd.read_csv(RES / "metrics.csv")
    maps = pd.read_csv(RES / "map_metrics.csv")
    runs = pd.read_csv(CACHE / "runs.csv")
    light = {s: pd.read_csv(CACHE / f"light_{s}.csv") for s in SEQS}
    gt = {s: pd.read_csv(CACHE / f"gt_{s}.csv") for s in SEQS}
    traj = {(slug, s): pd.read_csv(CACHE / f"traj_{slug}_{s}.csv")
            for slug in ("vggt_long", "mast3r_slam", "fast_lio2") for s in SEQS
            if (CACHE / f"traj_{slug}_{s}.csv").exists()}
    comp = pd.read_csv(CACHE / "comparisons.csv").set_index("sequence")
    return comp, gt, light, maps, metrics, runs, traj


@app.cell
def _(CACHE, SEQS, np):
    slices = {s: dict(np.load(CACHE / f"slices_{s}.npz")) for s in SEQS}
    return (slices,)


@app.cell
def _(DARK, RES, SEQS, light, np, pd, re, runs):
    def frame_time(seq, frame):
        """Full-rate frame index (30 Hz) -> seconds since the first image."""
        l = light[seq]
        return np.interp(frame, l.frame, l.t_rel)

    def vggt_links(seq):
        """Scale of each sequential chunk link from VGGT-Long's log; link a joins chunks a and a+1.

        Chunks are 60 frames with 30 overlap on the 10 Hz input, so link a sits on 10 Hz frames 30(a+1)...30(a+1)+29.
        """
        log = (RES / "vggt_long" / seq / "run_00" / "vggt_long" / "log.txt").read_text(errors="ignore").splitlines()
        rows, cur = [], None
        for line in log:
            if m := re.match(r"Aligning (\d+) and (\d+)", line):
                cur = int(m.group(1))
            elif cur is not None and (m := re.match(r"Estimated Scale: ([0-9.eE+-]+)", line)):
                rows.append((cur, float(m.group(1))))
                cur = None
        d = pd.DataFrame(rows, columns=["link", "scale"])
        d["t_rel"] = frame_time(seq, 3 * (30 * (d.link + 1) + 15))
        d["cumulative_scale"] = d.scale.cumprod()
        return d

    def vggt_messages(seq):
        """Failure-like messages in VGGT-Long's log, without the xFormers start-up notices."""
        log = (RES / "vggt_long" / seq / "run_00" / "vggt_long" / "log.txt").read_text(errors="ignore")
        return sum(1 for l in log.splitlines() if re.search(r"warn|fail|lost|error", l, re.I) and "xFormers" not in l)

    def mast3r_relocs(seq):
        """Failed relocalisations per block of about 30 frames (MASt3R-SLAM prints its FPS every 30 frames)."""
        run = runs[(runs.slug == "mast3r_slam") & (runs.sequence == seq)].run.iloc[0]
        log = (RES / "mast3r_slam" / seq / run / "mast3r_slam" / "log.txt").read_text(errors="ignore").splitlines()
        block, fails = 0, {}
        for line in log:
            if line.startswith("FPS:"):
                block += 1
            elif line.startswith("Failed to relocalize"):
                fails[block] = fails.get(block, 0) + 1
        d = pd.DataFrame({"block": range(block + 1)})
        d["failed"] = d.block.map(fails).fillna(0).astype(int)
        d["t_rel"] = frame_time(seq, 30 * d.block + 15)
        return d

    def vggt_conf(seq):
        """Mean VGGT depth confidence per frame (mean over the chunks that contain it); None until the run exists."""
        f = RES / "vggt_long" / seq / "run_00" / "frame_confidence.csv"
        if not f.exists():
            return None
        c = pd.read_csv(f)
        c["t"] = c.image.str.replace(".jpg", "", regex=False).astype(float)
        c = c.groupby("t", as_index=False).depth_conf_mean.mean()
        l = light[seq]
        c["t_rel"] = c.t - l.t.iloc[0]
        c["dark"] = np.interp(c.t_rel, l.t_rel, l.black_fraction) > DARK
        return c

    links = {s: vggt_links(s) for s in SEQS}
    relocs = {s: mast3r_relocs(s) for s in SEQS}
    conf = {s: vggt_conf(s) for s in SEQS}
    vggt_msgs = {s: vggt_messages(s) for s in SEQS}
    # share of the run the system itself reports as lost: MASt3R-SLAM relocalising (blocks of ~1 s), VGGT-Long never
    lost_pct = {("mast3r_slam", s): 100 * (relocs[s].failed > 0).sum() / light[s].t_rel.iloc[-1] for s in SEQS}
    lost_pct.update({("vggt_long", s): 0.0 for s in SEQS})
    return conf, links, lost_pct, relocs, vggt_msgs


@app.cell
def _(DARK, SEQS, gt, light, mo, np, pd):
    seq_table = pd.DataFrame([{
        "sequence": s,
        "duration [s]": round(gt[s].t_rel.iloc[-1], 0),
        "path [m]": round(float(np.linalg.norm(np.diff(gt[s][["x", "y"]].values, axis=0), axis=1).sum()), 0),
        "mean brightness (0-255)": round(light[s].brightness.mean(), 0),
        "dark frames [%]": round(100 * (light[s].black_fraction > DARK).mean(), 0),
    } for s in SEQS]).set_index("sequence")
    mo.vstack([mo.md("## 1. Data\nThree M3DGR indoor sequences in one 6.5 m x 3 m room, motion-capture ground truth."),
               seq_table])
    return


@app.cell
def _(DARK, SEQS, light, plt):
    def fig_light():
        fig, axes = plt.subplots(3, 1, figsize=(3.5, 2.6), sharex=True)
        for ax, s in zip(axes, SEQS):
            l = light[s]
            ax.plot(l.t_rel, l.brightness, color="0.2", lw=0.6)
            ax.fill_between(l.t_rel, 0, 1, where=l.black_fraction > DARK, color="0.85",
                            transform=ax.get_xaxis_transform(), lw=0)
            ax.set_ylim(0, 255)
            ax.set_ylabel(s, fontsize=7)
        axes[-1].set_xlabel("time [s]")
        fig.supylabel("image brightness", fontsize=8)
        return fig

    fig_light()
    return


@app.cell
def _(mo):
    mo.md(r"""## 2. RQ1 - accuracy in normal light""")
    return


@app.cell
def _(CORE, SEQS, lost_pct, maps, metrics, pd):
    def results_table():
        """Long format; maps come from run_00 (only run with a stored map for every system)."""
        rows = []
        for slug, name in CORE.items():
            for s in SEQS:
                m = metrics[(metrics.slug == slug) & (metrics.sequence == s)]
                mm = maps[(maps.slug == slug) & (maps.sequence == s) & (maps.stage == "icp_refined")
                          & (maps.region == "common")]
                q = m.ate_rmse_m.quantile([0.25, 0.5, 0.75])
                rows.append({"system": name, "sequence": s,
                             "ATE RMSE [m]": q[0.5], "IQR [m]": q[0.75] - q[0.25],
                             "success": f"{int(m.success.sum())}/{len(m)}",
                             "time span [%]": 100 * m.tracked_fraction.median(),
                             "reported lost [%]": lost_pct[(slug, s)],
                             "map accuracy [m]": mm.accuracy_median_m.median(),
                             "F-score @10 cm": mm.fscore_10cm.median(),
                             "ICP accepted": bool(mm.icp_accepted.all())})
        return pd.DataFrame(rows).set_index(["system", "sequence"])

    table = results_table()
    table.round(3)
    return (table,)


@app.cell
def _(CORE, SEQS, STYLE, gt, plt, traj):
    def fig_trajectories():
        fig, axes = plt.subplots(1, 3, figsize=(7.16, 1.6), sharex=True, sharey=True)
        for ax, s in zip(axes, SEQS):
            ax.plot(gt[s].x, gt[s].y, lw=1.2, label="ground truth", **STYLE["gt"])
            for slug, name in CORE.items():
                d = traj[(slug, s)]
                ax.plot(d.est_x, d.est_y, lw=0.7, label=name, **STYLE[slug])
            ax.set_aspect("equal")
            ax.set_title(s, fontsize=8)
            ax.tick_params(labelsize=6)
        axes[0].set_xlim(-4, 4)
        axes[0].set_ylim(-2.5, 2.5)
        axes[0].set_ylabel("y [m]")
        axes[1].set_xlabel("x [m]")
        h, l = axes[0].get_legend_handles_labels()
        fig.legend(h, l, loc="upper center", ncol=3, fontsize=6, frameon=False)
        fig.tight_layout(rect=(0, 0, 1, 0.88), w_pad=0.3)
        return fig

    fig_trajectories()
    return (fig_trajectories,)


@app.cell
def _(CORE, plt, slices):
    # one map per system is required; slots of systems without results yet stay as labelled placeholders
    MAP_SLOTS = [*CORE.items(), ("glim", "GLIM"), ("orb_slam3", "ORB-SLAM3"), ("tbd", "5th system")]

    def fig_maps(seq="Dynamic01"):
        sl = slices[seq]
        fig, axes = plt.subplots(1, len(MAP_SLOTS), figsize=(7.16, 1.15), sharex=True, sharey=True)
        for ax, (slug, name) in zip(axes, MAP_SLOTS):
            ax.scatter(*sl["reference"].T, s=0.005, c="0.75", rasterized=True)
            if slug in sl:
                ax.scatter(*sl[slug].T, s=0.005, c="k", alpha=0.6, rasterized=True)
            else:
                ax.text(0.5, 0.5, "pending", transform=ax.transAxes, ha="center", va="center", fontsize=7)
            ax.set_aspect("equal")
            ax.set_title(name, fontsize=7)
            ax.tick_params(labelsize=5)
        axes[0].set_xlim(-6, 8.5)
        axes[0].set_ylim(-4.5, 3.5)
        fig.tight_layout(w_pad=0.2)
        return fig

    fig_maps()
    return (fig_maps,)


@app.cell
def _(mo, table):
    _v, _m = table.loc["VGGT-Long"], table.loc["MASt3R-SLAM"]
    mo.md(f"""
    **RQ1.** In normal light both systems track the whole run. VGGT-Long is more accurate
    (ATE {_v.loc['Dynamic01', 'ATE RMSE [m]']:.2f} / {_v.loc['Sha-turn01', 'ATE RMSE [m]']:.2f} m vs.
    {_m.loc['Dynamic01', 'ATE RMSE [m]']:.2f} / {_m.loc['Sha-turn01', 'ATE RMSE [m]']:.2f} m), but a lower ATE does
    not mean a better map: on Sha-turn01 MASt3R-SLAM's map scores higher (F-score
    {_m.loc['Sha-turn01', 'F-score @10 cm']:.2f} vs. {_v.loc['Sha-turn01', 'F-score @10 cm']:.2f}).
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. RQ2 - what happens when the light goes out?

    Move the slider through `Varying-illu01`. Grey bands are dark phases (more than 80 % near-black pixels).
    """)
    return


@app.cell
def _(light, mo):
    t_slider = mo.ui.slider(0.0, float(light["Varying-illu01"].t_rel.iloc[-1]), step=0.5, value=60.0,
                            label="time [s]", full_width=True)
    t_slider
    return (t_slider,)


@app.cell
def _(CORE, DARK, STYLE, conf, light, links, np, plt, relocs, traj):
    def fig_timeline(seq="Varying-illu01", cursor=None):
        l = light[seq]
        c = conf[seq]
        n = 3 if c is not None else 2
        fig, axes = plt.subplots(n, 1, figsize=(7.16, 0.72 * n + 0.35), sharex=True)
        axes = [None, *axes]   # dark phases are shown as grey bands, no separate brightness panel
        for ax in axes[1:]:
            ax.fill_between(l.t_rel, 0, 1, where=l.black_fraction > DARK, color="0.88",
                            transform=ax.get_xaxis_transform(), lw=0)
            if cursor is not None:
                ax.axvline(cursor, color="k", lw=0.6)
        v = traj[("vggt_long", seq)]
        axes[1].plot(v.t_rel, v.error_m, lw=0.8, label="VGGT-Long (every frame)", **STYLE["vggt_long"])
        m = traj[("mast3r_slam", seq)]   # keyframes only: markers, so gaps stay visible
        axes[1].plot(m.t_rel, m.error_m, "o", ms=1.8, color=STYLE["mast3r_slam"]["color"],
                     label="MASt3R-SLAM (keyframes)")
        axes[1].set_ylabel("position\nerror [m]", fontsize=7)
        k = links[seq]
        axes[2].step(k.t_rel, np.log10(k.cumulative_scale), where="mid", lw=0.9, ls="-.",
                     color=STYLE["vggt_long"]["color"], label="VGGT-Long: chained chunk scale")
        axes[2].set_ylabel("log10 chunk\nscale", fontsize=7)
        r = relocs[seq]
        ax2 = axes[2].twinx()
        ax2.bar(r.t_rel, r.failed, width=0.9, color=STYLE["mast3r_slam"]["color"], alpha=0.45,
                label="MASt3R-SLAM: failed relocalisations")
        ax2.set_ylabel("failed\nrelocs / s", fontsize=7)
        ax2.grid(False)
        if c is not None:
            axes[3].plot(c.t_rel, c.depth_conf_mean, lw=0.7, **STYLE["vggt_long"])
            axes[3].axhline(1.0, color="0.4", lw=0.5, ls=":")
            axes[3].set_ylabel("VGGT depth\nconfidence", fontsize=7)
        h = [a.get_legend_handles_labels() for a in (axes[1], axes[2], ax2)]
        fig.legend(sum((x[0] for x in h), []), sum((x[1] for x in h), []), loc="upper center", ncol=4,
                   fontsize=6, frameon=False, bbox_to_anchor=(0.5, 1.0))
        axes[-1].set_xlabel("time [s]", fontsize=7)
        fig.tight_layout(rect=(0, 0, 1, 0.93), h_pad=0.3)
        return fig

    return (fig_timeline,)


@app.cell
def _(CACHE, CORE, STYLE, fig_timeline, gt, light, mo, plt, slices, t_slider, traj):
    def map_at(seq, t):
        fig, ax = plt.subplots(figsize=(3.0, 1.9))
        ax.scatter(*slices[seq]["reference"].T, s=0.02, c="0.8", rasterized=True)
        g = gt[seq]
        ax.plot(g.x, g.y, lw=0.5, **STYLE["gt"])
        i = (g.t_rel - t).abs().idxmin()
        ax.plot(g.x[i], g.y[i], "o", color="0.3", ms=5, label="true position")
        for slug, name in CORE.items():
            d = traj[(slug, seq)]
            j = (d.t_rel - t).abs().idxmin()
            ax.plot(d.est_x[j], d.est_y[j], "x", ms=6, mew=1.5, color=STYLE[slug]["color"], label=name)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=6, loc="upper right")
        return fig

    _seq = "Varying-illu01"
    _l = light[_seq]
    _frame = int(_l.frame[(_l.t_rel - t_slider.value).abs().idxmin()]) // 15 * 15
    mo.vstack([
        mo.hstack([mo.image(str(CACHE / "thumbs" / _seq / f"{_frame:05d}.jpg"), width=240),
                   map_at(_seq, t_slider.value)], justify="start"),
        fig_timeline(_seq, cursor=t_slider.value),
    ])
    return


@app.cell
def _(DARK, SEQS, comp, conf, light, np, pd, traj):
    def rmse(e):
        return float(np.sqrt(np.mean(np.square(e))))

    def dark_lit_rmse():
        """ATE RMSE restricted to dark / lit frames (same statistic as the ATE, review M1)."""
        rows = []
        for (slug, s), d in traj.items():
            if slug == "fast_lio2":
                continue
            l = light[s]
            dk = np.interp(d.t_rel, l.t_rel, l.black_fraction) > DARK
            rows.append({"slug": slug, "sequence": s, "rmse_dark_m": rmse(d.error_m[dk]) if dk.any() else np.nan,
                         "rmse_lit_m": rmse(d.error_m[~dk]), "last_pose_s": d.t_rel.iloc[-1]})
        return pd.DataFrame(rows).set_index(["slug", "sequence"])

    darklit = dark_lit_rmse()
    conf_lit = {s: conf[s][~conf[s].dark].depth_conf_mean.median() for s in SEQS if conf[s] is not None}
    checks = comp.join(pd.Series(conf_lit, name="conf_lit_median"))
    return checks, darklit


@app.cell
def _(checks, darklit, mo):
    mo.vstack([mo.md(r"""
    **Fairness checks.** MASt3R-SLAM is scored on its keyframes only; VGGT-Long on every frame. `vggt_on_mast3r_kf_m`
    scores VGGT-Long on the same timestamps. `const_baseline_m` is the ATE of a robot that never moves (the RMS
    distance of the ground truth to its centroid). `conf_lit_median`: VGGT confidence in lit frames.
    """), checks.round(3), darklit.round(3)])
    return


@app.cell
def _(checks, conf, darklit, links, lost_pct, mo, relocs, table, vggt_msgs):
    _s = "Varying-illu01"
    _v, _m = table.loc[("VGGT-Long", _s)], table.loc[("MASt3R-SLAM", _s)]
    _c = conf[_s]
    _conf_txt = ("" if _c is None else
                 f" VGGT's own depth confidence drops from a median of {_c[~_c.dark].depth_conf_mean.median():.1f} in lit"
                 f" frames to {_c[_c.dark].depth_conf_mean.median():.2f} in dark frames (95th percentile"
                 f" {_c[_c.dark].depth_conf_mean.quantile(0.95):.2f}). The confidence is 1 + e^x, so 1.0 is the lowest"
                 " value the model can output: it flags the dark input. VGGT-Long only drops points below 10 % of the"
                 " lower of the two chunk medians, a relative threshold that a pair of dark chunks passes. In lit frames"
                 f" after the blackouts the confidence is back at {checks.loc[_s, 'conf_lit_median']:.1f}"
                 f" (Dynamic01: {checks.loc['Dynamic01', 'conf_lit_median']:.1f}) while the pose is wrong: nothing flags"
                 " the wrong trajectory.")
    mo.md(f"""
    **RQ2.** VGGT-Long outputs a pose for every frame and ends at an ATE of {_v['ATE RMSE [m]']:.2f} m, no better than a
    robot that never moves ({checks.loc[_s, 'const_baseline_m']:.2f} m). Dark chunks are still joined to their
    neighbours; the chained chunk scale ends at {links[_s].cumulative_scale.iloc[-1]:.3f} (lit sequences:
    {links['Dynamic01'].cumulative_scale.iloc[-1]:.2f}, {links['Sha-turn01'].cumulative_scale.iloc[-1]:.2f}), and the
    trajectory shrinks towards the room centre. The error is larger in lit frames
    ({darklit.loc[('vggt_long', _s), 'rmse_lit_m']:.2f} m) than in dark ones
    ({darklit.loc[('vggt_long', _s), 'rmse_dark_m']:.2f} m). Its log contains {vggt_msgs[_s]} failure messages.
    MASt3R-SLAM stops adding keyframes in the dark and relocalises without success during
    {lost_pct[('mast3r_slam', _s)]:.0f} % of the run ({int(relocs[_s].failed.sum())} attempts, from the second dark
    phase on; in the first one it neither adds keyframes nor relocalises). Its last keyframe is at
    {darklit.loc[('mast3r_slam', _s), 'last_pose_s']:.0f} s. On its keyframes its ATE is {_m['ATE RMSE [m]']:.2f} m,
    VGGT-Long's on the same timestamps {checks.loc[_s, 'vggt_on_mast3r_kf_m']:.2f} m.{_conf_txt}
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""## 4. RQ3 - does the run show that it failed?""")
    return


@app.cell
def _(CORE, SEQS, conf, lost_pct, metrics, pd, relocs, vggt_msgs):
    def detect_table():
        rows = []
        for slug, name in CORE.items():
            for s in SEQS:
                m = metrics[(metrics.slug == slug) & (metrics.sequence == s)]
                c = conf[s]
                rows.append({
                    "system": name, "sequence": s,
                    "ATE [m]": m.ate_rmse_m.median(),
                    "time span [%]": 100 * m.tracked_fraction.median(),
                    "reported lost [%]": lost_pct[(slug, s)],
                    "failure messages in log": vggt_msgs[s] if slug == "vggt_long" else int(relocs[s].failed.sum()),
                    "median depth conf. (dark frames)": (c[c.dark].depth_conf_mean.median()
                                                         if slug == "vggt_long" and c is not None and c.dark.any()
                                                         else None),
                })
        return pd.DataFrame(rows).set_index(["system", "sequence"])

    detect_table().round(3)
    return


@app.cell
def _(mo):
    mo.md(r"""
    **RQ3.** A run counts as visibly failed when the system itself reports it (log message, relocalisation state).
    VGGT-Long reports nothing on any sequence; MASt3R-SLAM reports being lost for 0 % of the lit sequences and for
    most of the dark time of `Varying-illu01`. The "time span" column (first to last matched pose) is not
    gap-aware and is shown for transparency only; it cannot show tracking loss inside a run. VGGT's confidence flags
    the dark input but not the wrong trajectory that follows. Related work: learned trackers trade explicit tracking
    loss for silent drift (Thomas et al. 2026); VGGT's pose errors are only partly predicted by its confidence
    (Srinivasan and Nordström 2026); 3D foundation models produce dense geometry even for noise (Paul et al. 2026).
    """)
    return


@app.cell
def _(metrics, mo, pd):
    _det = metrics[metrics.slug.isin(["vggt_long", "mast3r_slam"])].pivot_table(
        index=["slug", "sequence"], columns="run", values="ate_rmse_m").round(3)
    _side = metrics[metrics.slug.isin(["fast_lio2", "map_long", "map_long_sim3", "vggt_long_16gb"])].groupby(
        ["slug", "sequence"]).agg(ate_rmse_m=("ate_rmse_m", "median"), scale=("scale_sim3", "median")).round(3)
    mo.vstack([
        mo.md(r"""
        ## 5. Repeatability and side results

        **Repeatability.** Five runs per sequence agree within 0.1 mm except MASt3R-SLAM on `Varying-illu01`, which
        alternates between two outcomes while it relocalises. Repetitions therefore show determinism, not robustness.
        """),
        _det,
        mo.md(r"""
        **Side results (one sentence each in the paper).** FAST-LIO2 (LiDAR + IMU) reproduces the published M3DGR
        value on Dynamic01 (0.14 m vs. 0.13 m) and serves as a pipeline check; its error is mostly the unknown
        offset between the mocap body and the LiDAR; on `Varying-illu01` it stays at 0.14 m, so scene and ground truth
        are fine there. Map-Long (VGGT-Long with the "metric" MapAnything, `map_long`) gets the scale wrong by a factor
        of 4 to 14 indoors (`scale_sim3`). The 16 GB variant of VGGT-Long (chunks of 30) loses accuracy on the sharp
        turns of Sha-turn01 and fails on `Varying-illu01` just like the default, so the failure does not hinge on the
        chunk length.
        """),
        _side,
    ])
    return


@app.cell
def _(mo):
    export = mo.ui.run_button(label="Write paper figures and table")
    mo.vstack([mo.md("## 6. Export to `paper/`"), export])
    return (export,)


@app.cell
def _(FIG, SEQS, fig_maps, fig_timeline, fig_trajectories, plt, table):
    def write_paper():
        """Figures and the results table for paper/; also callable without the UI."""
        FIG.mkdir(parents=True, exist_ok=True)
        for name, make in (("trajectories", fig_trajectories), ("maps_dynamic01", fig_maps),
                           ("timeline_varying", fig_timeline)):
            f = make()
            f.savefig(FIG / f"{name}.pdf")
            plt.close(f)
        # one row per system, so the table stays short once all members' systems are in
        lines = [r"\setlength{\tabcolsep}{3.5pt}", r"\begin{tabular}{@{}lccccccc@{}}", r"\toprule",
                 r" & \multicolumn{3}{c}{ATE RMSE [m]} & \multicolumn{3}{c}{F-score @10\,cm} & Lost \\",
                 r"\cmidrule(lr){2-4}\cmidrule(lr){5-7}\cmidrule(l){8-8}",
                 r"System & D & S & V & D & S & V & V [\%] \\", r"\midrule"]
        for sys_ in table.index.get_level_values(0).unique():
            r = table.loc[sys_]
            f = [f"{r.loc[s, 'F-score @10 cm']:.2f}" + ("$^\\dagger$" if not r.loc[s, "ICP accepted"] else "")
                 for s in SEQS]
            lines.append(f"{sys_} & " + " & ".join(f"{r.loc[s, 'ATE RMSE [m]']:.2f}" for s in SEQS) + " & "
                         + " & ".join(f) + f" & {r.loc[SEQS[2], 'reported lost [%]']:.0f} \\\\")
        lines += [r"\bottomrule", r"\end{tabular}"]
        (FIG.parent / "tables" / "results.tex").write_text(
            "% Generated by notebooks/story.py - do not edit by hand.\n" + "\n".join(lines) + "\n")
        return ["trajectories.pdf", "maps_dynamic01.pdf", "timeline_varying.pdf", "tables/results.tex"]

    return (write_paper,)


@app.cell
def _(export, mo, write_paper):
    mo.stop(not export.value)
    mo.md("Written: " + ", ".join(f"`{f}`" for f in write_paper()))
    return


if __name__ == "__main__":
    app.run()
