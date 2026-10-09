# Project Plan: Comparing SLAM Systems on a Shared Ground-Robot Dataset

Course: *Einsatz Autonomer Robotersysteme Lab* (FH Technikum Wien), group of 4.
Status: 2026-10-08. Background research: [literature.md](literature.md), BibTeX: [references.bib](../paper/references.bib).

---

## 1. Requirements from the kick-off slides

| Requirement | What it means for us |
|---|---|
| Each member evaluates at least 1 SLAM system, **one shared dataset** for all | 4 core systems, all on the same sequences |
| Run the systems on a simulation or a dataset | Public real dataset with ground truth; Gazebo only as a private test sandbox |
| Metrics: ATE, RPE, map overlays (PSNR / IoU for semantic SLAM); RPE dropped after feedback from the lecturer | Only established metrics, no self-invented ones (Section 5) |
| ≥ 1 plot overlaying the trajectories of all systems, ≥ 1 map per system | Produced automatically by the pipeline |
| Figures: serif font, ≥ body text size, ≥ 300 DPI, readable in grayscale | One shared Matplotlib style file for all plots |
| Paper: exactly 3 pages, IEEE template, IMRAD, English | Page budget in Section 8 |
| Code submission: repo with README, requirements/Dockerfile, results, slides | Repo layout in Section 6 |
| Grading: 50 % implementation, 50 % paper, both ≥ 60 % | |
| **"Fancy visualisation is the minimum, the hard part is critical reflection in the context of theory"** | Plan the discussion from day one: *why* does system X behave like this? |
| Goal for lab session 1: dataset plays back in ROS (playback, RViz) + overview of candidate SLAM pipelines and metrics | Phase 1 in Section 7 |

Our own constraints: wheeled ground robot (no drone), keep it simple, not everyone has a strong laptop (no Isaac Sim), **ground truth is mandatory**, **standard metrics only**.
The lecturer allows any setup per system: ROS 1, ROS 2 or no ROS, CPU or NVIDIA GPU.

---

## 2. Dataset

### Selection criteria (in priority order)
1. **Ground truth independent of the tested sensors** (motion capture / laser tracker / RTK) covering the **whole** trajectory.
2. Wheeled ground robot.
3. Enough sensors for 4 *different* SLAM approaches (camera, IMU, odometry, LiDAR).
4. Small sequences (laptops, download, repeated runs).
5. Citable, ideally with published reference results.

### Candidates

| Dataset | Robot | Sensors | Ground truth | Size per sequence | Verdict |
|---|---|---|---|---|---|
| **M3DGR** (Zhang et al., IROS 2025) | differential drive | Livox MID-360 + Avia (3D), RealSense D435i RGB-D + IMU, **wheel odometry** 20 Hz, 360° camera, GNSS | OptiTrack (indoor, 360 Hz), RTK (outdoor) | indoor approx. 1.2–2.1 GB | **Recommended.** All modalities, small bags, MIT license, paper contains a baseline table with ~40 systems |
| M2DGR (Yin et al., RA-L 2022) | GAEA | Velodyne VLP-32C, 7 cameras, D435i, IMU, thermal, event | Vicon (room), Leica tracker (hall, position only), RTK | 14–36 GB | Backup: well cited, but **no wheel odometry**, very large bags |
| FusionPortableV2 (Wei et al., IJRR 2024) | Ackermann UGV | Ouster OS1-128, global-shutter stereo, STIM300 IMU, encoders | RTK-INS (6-DoF outdoor only) | n/a | Best sensors, but mostly outdoor, no RGB-D |
| OpenLORIS-Scene (Shi et al., ICRA 2020) | Segway | D435i, T265, 2D Hokuyo, odometry | Mocap only in "office", elsewhere LiDAR SLAM (**biased** towards LiDAR methods) | small | Visual-only comparisons only |
| TUM RGB-D fr2/pioneer (Sturm et al., IROS 2012) | Pioneer 3 | Kinect RGB-D | Mocap | 0.6–1.5 GB | Very well cited, but visual only |
| Own Gazebo simulation | TurtleBot4 / TurtleBot3 | freely configurable | perfect (simulator pose) | – | Private sandbox for testing only (see below): not bit-identical across machines, so not used for results |

Excluded: Newer College, Hilti, TUM-VI, ETH3D (handheld), GrandTour (legged), KITTI (car), Intel/FR079 logs and the Cartographer museum bag (no ground-truth trajectory).

### Recommendation: M3DGR, 3 indoor sequences with motion capture

Proposal (check in the README that GT really is mocap before downloading):

| Role | Sequence | Duration | What it tests |
|---|---|---|---|
| easy | `Varying-illu01` | approx. 154 s | nominal case |
| medium | `Dynamic01` | approx. 175 s | moving people/objects |
| hard | `Dark03` or `Sha-turn01` | approx. 140–170 s | darkness (bad for cameras) or sharp turns / wheel slip (bad for odometry) |

The "hard" sequences are chosen so that each sensor modality struggles **differently**. This drives the discussion in the paper: cameras need texture and light, LiDAR needs geometry, odometry needs traction.

Known pitfalls (resolve in week 1):
- **LiDAR format:** The LiDAR is stored as **Livox CustomMsg**, not `PointCloud2`. Some LiDAR systems read it natively; others need a small Python conversion script (`rosbags`).
- **ROS version:** The bags are ROS 1. Convert to ROS 2 with `rosbags-convert` as described in the README, passing the Livox message definitions.
- **Image compression:** Images are compressed (`compressed`, `compressedDepth`). Extract them once into image folders, which the RGB-only systems need anyway.
- **Synchronisation:** Sensors are synchronised in software only, and the cameras are rolling shutter. Mention this as a limitation in the paper.
- **Ground-truth frame:** The mocap marker frame is not the robot's `base_link`. Apply the extrinsics from `calibration.md`.

### Working principle: everyone works independently

- **Results:** Everyone downloads the public M3DGR sequences and the ground truth themselves. Nobody has to wait for another member's data, recordings or machine.
- **Evaluation:** The evaluation code in the repository is identical for everyone. Each member can compute their own numbers.
- **Simulation (optional, private sandbox):** Anyone may run a local Gazebo simulation (TurtleBot4 or TurtleBot3, ROS 2 Jazzy) to debug their own runner without downloading M3DGR. Simulation output is **not** used for the paper.
  - Reason: simulation runs are not bit-identical across machines (physics step timing, rendering, noise). Every member would effectively get a different dataset, and the results would not be comparable.
- **Fallback if M3DGR fails in week 1:** switch the whole group to another **public** dataset with independent ground truth from Section 2, e.g. M2DGR or OpenLORIS-Scene office. Again, everyone downloads it themselves.

---

## 3. SLAM systems

### 3.1 Core comparison: one system per member

**Fixed for everyone:** dataset and sequences (Section 2), metrics and alignment rules (Section 5), result format (Section 6).
**Chosen by each member:** the SLAM system. Do your own research. Rules for your choice:

1. It must run on the shared M3DGR sequences, using any subset of the recorded sensors: 3D LiDAR, IMU, wheel odometry, RGB, RGB-D.
2. It must output a trajectory that can be converted to TUM format, plus a map or reconstruction.
3. It must have a citable paper, so the theory paragraph in the paper can be backed by literature.
4. **Prefer a different sensor modality or paradigm than the others.** Ideal is a "modality ladder" from the cheapest sensor (one RGB camera) to multi-sensor fusion. The comparison is only interesting if the systems differ, e.g.:
   - LiDAR vs. camera;
   - with vs. without IMU or odometry;
   - filter vs. graph optimisation;
   - with vs. without loop closure;
   - classical vs. learned.
5. Register your choice in `team.yaml` (see the README). Coordinate in the group chat before you start, so two people do not pick the same modality.

| Member | System | Sensors | Paradigm | Loop closure | Hardware |
|---|---|---|---|---|---|
| Elias Bitsch | **MASt3R-SLAM** (Murai et al., CVPR 2025) | monocular RGB | 3D foundation model + dense SLAM | yes | RTX 3080 Ti / GPU cluster |
| Philip Stix | *your choice* | | | | |
| Viktoriia Ovdiienko | **ORB-SLAM3** (Campos et al., T-RO 2021) | RGB camera (RealSense D435i) + IMU | feature-based visual-inertial, keyframe graph optimisation | yes | Intel i5-1155G7, 8 GB RAM, no GPU |
| Jiayi Zhou | *your choice* | | | | |

### 3.2 RGB-only deep dive (Elias, optional extension)

All of these run on **the same monocular RGB images**. They all share the same fundamental limitation: a single camera cannot observe metric scale. They differ in paradigm:

| Priority | System | Paradigm | Code | GPU notes |
|---|---|---|---|---|
| 1 (core) | **MASt3R-SLAM** (Murai, Dexheimer, Davison, CVPR 2025) | 3D foundation model, dense | ✅ image folder or video input, WSL branch available | authors tested on RTX 4090 (24 GB); 12 GB uncertain → GPU cluster |
| 2 | **cuVSLAM** (Korovko et al., NVIDIA, 2025) | classical geometric, CUDA-accelerated | ✅ PyCuVSLAM wheel, Ubuntu 22.04/24.04, CUDA 12/13, NVIDIA Community License | light, runs on Jetson |
| 3 | **DPV-SLAM** (Lipson, Teed, Deng, ECCV 2024) | learned sparse patches + loop closure | ✅ Python | mid-range GPU |
| 4 | **Gaussian Splatting SLAM / MonoGS** (Matsuki et al., CVPR 2024) | photorealistic 3D Gaussian map | ✅ monocular supported, TUM loader | authors tested on RTX 4090; slow (~10 fps speed-up branch) |

Not included: FoundationSLAM (Wu et al., AAAI 2026), because no public code was found. GPU-heavy alternatives such as DROID-SLAM are mentioned in related work only.

How it fits the paper:
- **Main table:** only the 4 core systems. The deep dive gets one sentence plus a figure in the main text and full results in the **appendix**, which the course allows. It also makes a strong slide in the final presentation.
- **Theory angle:** Classical geometric (cuVSLAM) vs. learned (DPV-SLAM) vs. foundation model (MASt3R-SLAM) vs. neural rendering (MonoGS). The key question is whether learned priors make monocular SLAM **more robust** (darkness, dynamics) and whether foundation models recover approximately correct scale. To answer it, report the estimated Sim(3) scale factor.
- **MonoGS bonus:** It reports **PSNR / SSIM / LPIPS**, the standard rendering-quality metrics in Gaussian-splatting SLAM papers. PSNR is explicitly listed on the kick-off slides. This gives a quantitative map metric without a reference map.
- **Time-box:** Work through the systems in priority order. Stop when the time budget for the deep dive is used up.

**Fairness rule (all systems):** Every system runs with the authors' **default parameters** for the matching sensor. Only sensor calibration and topic names are adapted. Any other change is documented and equally allowed for everyone.

---

## 4. Framing, research question and contribution

### How much weight does the application story get?

**The application is motivation only (2–3 sentences in the introduction). The core is the scientific comparison.** Reasons:
- The grading focuses on methodology, comparison and **critical reflection in the context of theory**, not on a use case.
- A strong story (self-driving car, factory, …) promises claims our data cannot support. The template says *every claim must be backed by literature or our results*. An indoor robot dataset says nothing about cars on a motorway.
- The application should **follow from the dataset**, not the other way round. The M3DGR indoor sequences show a robot inside buildings with changing light, darkness, people and tight turns. That motivates **indoor service and intralogistics robots** (warehouse, hospital, office) operating in poor light and among people.
- A self-driving car does not fit (different scale, different sensors, not a ground-robot dataset). Warehouse or factory fits, but only as a motivating sentence, without drawing conclusions from it.

In the conclusion we may give a **cautious** recommendation, e.g. "for indoor robots under changing illumination, LiDAR-based SLAM is more robust than camera-only SLAM". It must follow directly from our results.

### Research question (draft, finalise once all systems are chosen)

> *How do SLAM systems using different sensor modalities (e.g. monocular RGB with a foundation-model prior vs. <the modalities chosen by the others>) compare in accuracy and robustness on the same ground robot, and which sensor-specific failure modes (e.g. scale unobservability, darkness, geometric degeneracy) explain the differences?*

**Contribution:**
- A reproducible comparison (containers + automated pipeline) of four sensor modalities on identical data.
- An interpretation of the failure modes based on theory: observability, drift without loop closure, degeneracy under darkness or poor geometry.
- A sanity check of our numbers against the M3DGR baseline table.

---

## 5. Evaluation methodology: established metrics only

Principle: **we do not invent metrics.** Every number has an original reference and is computed with a standard tool (evo).

### 5.1 Trajectory

| Metric | Definition / reference | Tool | Setting |
|---|---|---|---|
| **ATE** (Absolute Trajectory Error), translational RMSE | Sturm et al. 2012 | `evo_ape` | Umeyama (1991) alignment over all poses |
| Drift in % per distance *(optional)* | Geiger et al. 2012 (KITTI) | KITTI definition | only if sequences are long enough |

**Alignment.** The alignment follows what each sensor can observe (Zhang & Scaramuzza 2018):
- **Metric systems** (LiDAR, RGB-D, stereo, visual-inertial): SE(3) alignment (`-a`).
- **Monocular systems** (MASt3R-SLAM and the RGB deep dive): **Sim(3)** alignment (`-as`). The estimated scale factor is reported as well.
- **Transparency check:** *All* systems are additionally evaluated with Sim(3). This shows how much of the metric systems' error is pure scale error, so the monocular systems do not appear unfairly favoured.
- **2D values:** Since this is a ground robot, we also report planar values (`--project_to_plane xy`) as a supplement, not a replacement.

**Time association:** `--t_max_diff 0.01` s (evo default). Report the number of matched poses.

**Statistics (all from evo):** RMSE, mean, median, standard deviation, max.

### 5.2 Robustness (as in the ORB-SLAM3 paper)
- **Success rate** k/N over N runs.
- **Fraction of trajectory tracked** (tracked duration / GT duration).
- Failed runs are **never** silently dropped. They are marked in the table.

### 5.3 Repetitions and statistical testing
- **N = 5 runs per system and sequence.** ORB-SLAM3 uses 10, so use 10 if time allows; with the GPU cluster, N = 10 is cheap for the GPU systems.
  - Reason: multithreading and GPU non-determinism make SLAM non-deterministic.
  - Report the **median** and **IQR** over runs.
- **Comparing systems:**
  - Kruskal–Wallis test.
  - Pairwise Mann–Whitney U tests with Holm correction.
  - With N = 5, phrase the results as descriptive support only.

### 5.4 Resources
These are measurements, not metrics, following the practice in SLAMBench and Trejos et al. 2022.
- **What we measure:** CPU [% of one core], peak RAM [MB], GPU memory (for CUDA systems), processing time per frame/scan, real-time factor.
- **How:** `psutil` sampling at 10 Hz over the SLAM process tree; `nvidia-smi` for the GPU.
- **Hardware differs per system.** Consequences:
  - Resource numbers are reported **per system with the hardware stated** (CPU, GPU, RAM, OS, ROS distro), but **not used for ranking**. A CUDA runtime on a GPU is not comparable to a CPU runtime on a laptop.
  - This is mentioned as a limitation in the paper. The core comparison is accuracy and robustness.
  - Within the RGB deep dive, all systems should run on the **same GPU type** (e.g. all on the cluster). There, resource numbers *are* comparable.

### 5.5 Maps
- **Required by the slides:** one map/reconstruction per system and a **map overlay**.
  - Render each point cloud / grid map in the GT-aligned frame, same top-down view, same scale.
- **Quantitative only if a reference map exists** (check for M3DGR; can be exported from Gazebo):
  - Point clouds: accuracy / completeness / F-score at threshold τ (Knapitsch et al. 2017), plus Chamfer distance.
  - 2D grids: mean nearest-neighbour distance to the reference map (Santos et al. 2013).
- **Rendering quality** for MonoGS: PSNR / SSIM / LPIPS as defined in the Gaussian-splatting SLAM literature.
- Without a reference map, the map evaluation stays qualitative, and we say so honestly in the paper.

### 5.6 Figures and tables for the paper
1. **Trajectory overlay** of all 4 systems + GT, top-down, same alignment as the metric *(required)*
2. **One map per system** side by side, same scale *(required)*
3. **Box plot of ATE RMSE** over N runs per system, individual runs as dots, failures annotated
4. **APE over time** (shows drift and loop-closure jumps), optional
5. **Main table:** system × {ATE RMSE (median, IQR), success k/N, % tracked, hardware, runtime}

All plots share one style file:
- Serif font (Times / Latin Modern) at body-text size.
- 300 DPI or vector PDF.
- Grayscale-safe: line styles and markers, not just colour.

---

## 6. Automated test pipeline (Python + Jupyter)

### Principle
Every SLAM system gets a **runner with an identical interface**. The pipeline only knows this interface, and the evaluation only knows the files in `results/`.

```
shared dataset  ->  runner per system (any machine / container)  ->  results/<system>/<sequence>/run_<k>/
                                                                         trajectory_tum.txt
                                                                         map.(pcd|ply|pgm)
                                                                         resources.csv
                                                                         meta.json  (version, commit hash, params, hardware, runtime, success)
                                                                   ->  evaluate.py (evo Python API)  ->  metrics.csv
                                                                   ->  notebooks  ->  paper/figures/*.pdf, paper/tables/*.tex
```

### Repository layout

```
bEARL/
├── README.md                  # installation + how to reproduce everything
├── requirements.txt           # evo, rosbags, numpy, pandas, matplotlib, scipy, psutil, open3d, jupyter
├── Makefile                   # data / run / eval / figures / paper
├── config/
│   ├── sequences.yaml         # sequences, GT file, extrinsics
│   └── systems/<system>.yaml  # topics, parameters, container image, number of runs
├── docker/<system>/Dockerfile # one image per SLAM system, pinned base image
├── data/                      # (git-ignored) bags, extracted images, GT in TUM format
├── pipeline/
│   ├── prepare_data.py        # ROS1→ROS2, Livox→PointCloud2, image extraction, GT→TUM
│   ├── runners/<system>.py    # common function run(seq, out_dir) -> result files
│   ├── run_all.py             # systems × sequences × N runs, with resource logging
│   ├── resources.py           # psutil / nvidia-smi sampler
│   └── evaluate.py            # ATE / robustness -> metrics.csv
├── cluster/                   # Slurm job scripts for batch runs on the GPU cluster
├── notebooks/
│   ├── 01_dataset.ipynb       # sensor overview, GT plot, timestamp checks
│   ├── 02_trajectories.ipynb  # overlay, APE over time, box plots, table -> LaTeX
│   ├── 03_maps.ipynb          # maps side by side, overlays
│   └── 04_resources.ipynb     # CPU / RAM / GPU / runtime
├── results/                   # raw outputs of all runs (large maps -> release / cloud)
├── paper/                     # IEEE template, figures/, tables/, references.bib
└── docs/                      # this plan, literature
```

### Key details
- **Offline instead of real time.** This is what keeps the comparison fair across different machines.
  - Every system processes **every** frame/scan. File-based systems read from files. ROS-based systems replay the bag slowly enough (e.g. `--rate 0.5`) and verify that no messages are dropped.
  - Accuracy then does not depend on how fast a given laptop is.
  - With ROS, always use `--clock` and `use_sim_time:=true`.
  - The real-time factor is measured separately, as context only.
- **The result format is the contract.** Where and how a runner executes does not matter. It must deliver `trajectory_tum.txt`, the map, `resources.csv` and `meta.json` (including hardware) in the agreed folder. Results are merged via Git, or cloud storage for large maps. The **evaluation** (`evaluate.py` + notebooks) runs centrally and is therefore identical for everyone.
- **GPU cluster (H200).** For GPU systems and repeated runs, `cluster/` holds Slurm array jobs: one job per system × sequence × run. Containers are converted to Apptainer/Singularity images if the cluster does not allow Docker. Check the cluster's usage rules before the first batch.
- **Pinned versions.** The commit hash of each SLAM repo, the evo version and `pip freeze` are written to `meta.json` automatically.
- **evo as a library** in the notebooks (`evo.core.sync`, `evo.core.metrics`, `evo.tools.plot`), not only via the CLI. This guarantees identical alignment and parameters for every system.
- **Shared Matplotlib style file** `paper/ieee.mplstyle`, with evo's plot settings switched over to it.
- **One command reproduces everything:** `make run` (slow) and `make eval figures` (seconds). The notebooks read `results/` only and never re-run SLAM.

---

## 7. Timeline along the lab sessions

| Phase | Due | Deliverable | Who |
|---|---|---|---|
| 0 | now | GitHub repo, structure, roles agreed, dataset decision | all |
| 1 | **lab session 1** | M3DGR sequence plays back in ROS 2 (playback + RViz), GT as TUM file, list of SLAM candidates + metrics (required by the slides) | data owner + all |
| 2 | lab session 2 | every runner produces a TUM trajectory + map for 1 sequence; `evaluate.py` gives first ATE | each member for their system |
| 3 | lab session 3 | every member: N runs × 3 sequences on their own machine or the cluster, results in the repo; all notebooks produce figures | pipeline + evaluation |
| 4 | lab session 4 | full paper draft (3 pages), final figures, discussion of failure modes | all, paper lead coordinates |
| 5 | final presentation | 8–10 min slides, paper PDF uploaded to Moodle by **midnight**, repo link submitted | all |

**Buffer:** Phase 2 is usually the riskiest (builds, calibration, topic names). Everyone should pick a fallback system alongside their main choice.

### Roles (in addition to your own SLAM system)

| Role | Responsibilities |
|---|---|
| Data | download, ROS1→ROS2 conversion, Livox→PointCloud2, GT→TUM, extrinsics, `01_dataset.ipynb` |
| Pipeline | runner interface, `run_all.py`, resource measurement, containers, Makefile, cluster jobs |
| Evaluation | `evaluate.py`, all plots/tables, style file, statistics |
| Paper | LaTeX template, literature/BibTeX, structure, final editing, page count |

Each member writes the theory paragraph for **their** system (3–5 sentences) and the discussion of its failure modes.

---

## 8. Paper outline (IEEE, exactly 3 pages, English)

| Section | Approx. length | Content |
|---|---|---|
| Abstract | 8–10 lines | motivation, contribution, verifiability (repo link) |
| I. Introduction | 0.3 p. | why SLAM on ground robots, why compare modalities, contribution in 2–3 points |
| II. State of the Art | 0.5 p. | SLAM formulation (Cadena 2016, Grisetti 2010), the 4 chosen systems briefly, existing comparisons (Trejos 2022, M3DGR benchmark), recent learned monocular SLAM, *so what* → our gap |
| III. Materials and Methods | 0.7 p. | dataset + sequences, systems + configuration, metrics (ATE/alignment/N runs), hardware, pipeline |
| IV. Experimental Results | 0.8 p. | overlay plot, maps, box plot, main table |
| V. Summary and Outlook | 0.5 p. | failure modes ↔ theory, limitations (software sync, default parameters, small N, different hardware, possibly no reference map), outlook |
| References | 0.2 p. | approx. 12–18 entries |
| Appendix (optional) | – | RGB-only deep dive |

---

## 9. Open questions for the group

1. **Dataset:** M3DGR as the shared, public dataset. Gazebo is only a private test sandbox. Everyone works independently.
2. **OS:** Who has Ubuntu (native or WSL2) with ROS 2 Humble/Jazzy? On Windows only via WSL2 + Docker.
3. **Hardware per system:** Who has which machine? MASt3R-SLAM runs on an RTX 3080 Ti and the H200 GPU cluster.
4. **Dates:** Fill in the dates of lab sessions 2–5.
5. **Which system does each of you choose, and who takes which role?** Fill in `team.yaml`.
