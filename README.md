# bEARL: Comparing SLAM Systems on a Shared Ground-Robot Dataset

Group project for the course *Einsatz Autonomer Robotersysteme Lab* (FH Technikum Wien).

Each member evaluates **one SLAM system of their own choice**. Everything else is fixed and identical for everyone, so the results are scientifically comparable:

- **Same dataset and sequences:** M3DGR, 3 indoor sequences with motion-capture ground truth.
- **Same metrics:** ATE via evo, with standard alignment rules; no self-invented metrics.
- **Same result format and one central evaluation pipeline.**

| Member | SLAM system | Sensors | Status |
|---|---|---|---|
| Elias Bitsch | VGGT-Long | monocular RGB | done (3 sequences x 5 runs) |
| Philip Stix | GLIM | 3D LiDAR (Livox MID-360) + IMU | system chosen, runner to do |
| Viktoriia Ovdiienko | ORB-SLAM3 | RGB camera (RealSense D435i) + IMU | system chosen, runner to do |
| Jiayi Zhou | *to be chosen* | | |

📄 **Read first:** [docs/PLAN.md](docs/PLAN.md) (full plan, methodology, timeline) and [docs/literature.md](docs/literature.md).

---

## Your first steps (each member)

### 1. Choose your SLAM system
Do your own research. The system must:
- run on the M3DGR sensors (3D LiDAR, IMU, wheel odometry, RGB, RGB-D, or any subset of them);
- produce a trajectory that can be converted to TUM format, plus a map;
- have a citable paper.

**Prefer a different sensor modality or paradigm than the others.** Post your choice in the group chat before you start. Criteria: [PLAN.md, Section 3](docs/PLAN.md#3-slam-systems).

### 2. Register it (one command)
```bash
python tools/register_system.py --member "Philip Stix" --system "Name Of SLAM" --slug name_of_slam \
    --sensors lidar3d imu --alignment se3 --loop-closure true --paper bibkey2024 \
    --hardware "CPU / GPU / RAM of your machine"
```
This fills in your entry in [team.yaml](team.yaml) and creates two files for you from the templates:
- `config/systems/<slug>.yaml`: source repo, pinned commit, parameters (defaults only!).
- `pipeline/runners/<slug>.py`: the function that runs your system on one sequence.

Use `--alignment sim3` only for monocular (scale-ambiguous) systems, otherwise `se3`.

### 3. Implement your runner
Run your system however you like (pip, Docker, ROS 1/2, CPU or CUDA), but **offline**, so that every frame is processed. Your runner must write to its `out_dir`:

| File | Content |
|---|---|
| `trajectory_tum.txt` | `t x y z qx qy qz qw` per line, `t` in seconds on the ground-truth clock |
| `map.<ext>` | `.ply`/`.pcd` point cloud, or `.pgm` + `.yaml` grid map |

If your system fails (crash, lost tracking), **raise an exception**. Failures are recorded and reported, never dropped.

### 4. Run and evaluate
```bash
python -m pipeline.run_all --system <slug> --sequence Dynamic01 --run 0   # quick test
python -m pipeline.run_all --system <slug>                                # all sequences x N runs
python -m pipeline.evaluate                                               # -> results/metrics.csv, summary.csv
```
Commit your `results/<slug>/` folder. Large maps are git-ignored; share them via the cloud folder.

### 5. Paper
Add your system's paper to [paper/references.bib](paper/references.bib) and write your system's theory paragraph in [paper/main.tex](paper/main.tex).

---

## Installation (evaluation environment)

```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```
Tested with Python 3.10–3.12. ROS is **not** required for the evaluation. It is only needed if your SLAM system needs it.

## Data

1. Download the three sequences listed in [config/sequences.yaml](config/sequences.yaml) **and their GT files** from the [M3DGR repository](https://github.com/sjtuyinjie/M3DGR) (Section 5, OneDrive) into `data/M3DGR/` (git-ignored). Name them `<seq>.bag` and `GT<seq>.txt`, e.g. `Dynamic01.bag` and `GTDynamic01.txt` (about 2 GB per bag, ~4 MB per GT file). Check the bags against `checksums.txt` from the M3DGR repo.
   - Viewing: [Foxglove](https://foxglove.dev) or [Lichtblick](https://github.com/lichtblick-suite/lichtblick) open the ROS 1 bags directly, no ROS installation needed. The Livox topics (`CustomMsg`) are not shown as point clouds until converted.
2. Prepare them once:
   ```bash
   python -m pipeline.prepare_data --bag data/M3DGR/Dynamic01.bag --sequence Dynamic01
   ```
   This copies the GT to `data/M3DGR/<seq>/gt_tum.txt` and extracts the RGB images (JPEG, unchanged from the bag).
   With `--ros2` it also writes a ROS 2 Humble copy (`data/M3DGR/<seq>_ros2/`, MCAP, ~4.6 GB) that contains every topic
   plus `sensor_msgs/PointCloud2` versions of the Livox topics (`/livox/mid360/lidar_points`, `/livox/avia/lidar_points`;
   fields `x y z intensity tag line`, `t` = seconds since scan start, `timestamp` = absolute seconds).
   Still open: depth images.
3. Play a sequence in ROS 2 and watch it live (lab session 1: playback + visualisation):
   ```bash
   docker build -t bearl/ros2_playback docker/ros2_playback
   docker run --rm --network host -v $PWD/data/M3DGR:/data:ro bearl/ros2_playback Dynamic01_ros2
   ```
   Then open Foxglove or Lichtblick, *Open connection* -> *Foxglove WebSocket* -> `ws://localhost:8765`
   (`-e PORT=8766` if the port is taken) and import `tools/lichtblick_live_layout.json` (point cloud, camera, odometry).
   `python -m pipeline.visualize` writes an offline replay of a SLAM run (GT vs. estimate, maps, driving state) for
   `tools/lichtblick_layout.json`.

## Repository layout

```
team.yaml                 who evaluates which system (single source of truth)
config/sequences.yaml     shared sequences, topics, evaluation settings (fixed for everyone)
config/systems/           one config per SLAM system
pipeline/                 prepare_data, run_all, evaluate, resources, runners/<slug>.py
tools/register_system.py  register your system
notebooks/                01 dataset, 02 trajectories, 03 maps, 04 resources -> paper/figures, paper/tables
cluster/                  Slurm array job for batch runs on a GPU cluster
docker/                   one container per SLAM system (optional)
paper/                    IEEE template (main.tex), references.bib, ieee.mplstyle, figures, tables
slides/                   final presentation (FH Technikum template)
docs/                     plan and literature review
results/                  raw outputs of all runs: results/<slug>/<sequence>/run_<k>/
```

## Reproducing everything
```bash
make install && make run && make eval && make figures && make paper
```

## Rules we agreed on
- **Defaults only:** use the authors' default parameters. Only calibration and topic names may be adapted, and every deviation is documented in `config/systems/<slug>.yaml`.
- **Standard metrics only:** ATE (Sturm et al. 2012), Umeyama alignment, SE(3) for metric systems and Sim(3) for monocular ones (Zhang & Scaramuzza 2018).
- **Repeated runs:** N runs per system and sequence; we report the median and IQR.
- **Report failures:** failed runs are reported, never removed.
- **Resources are context only:** CPU, RAM and GPU numbers are always reported together with the hardware and never used for ranking.
