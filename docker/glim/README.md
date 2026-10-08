# GLIM (Philip Stix)

GLIM runs in the authors' Docker image `koide3/glim_ros2:jazzy_cuda13.1` (no own Dockerfile needed).
Needs Docker with NVIDIA GPU support (Linux, or Docker Desktop + WSL2 on Windows) and an NVIDIA driver
that supports CUDA 13.1. Without a GPU: use `koide3/glim_ros2:jazzy`, switch the three `*_gpu.json`
entries in `config/config.json` to `*_cpu.json`, set `gpus: false` in `config/systems/glim.yaml`,
and document the CPU variant as a deviation.

## 1. Data (manual download, OneDrive needs a browser)

Download the bag and the GT file of each sequence from the [M3DGR README](https://github.com/sjtuyinjie/M3DGR)
into `data/M3DGR/` (git-ignored):

| Sequence | Bag (OneDrive) | GT (OneDrive) |
|---|---|---|
| Varying-illu01 (1.84 GB, 154 s) | [bag](https://1drv.ms/u/c/2b4bfc0edf421186/EYYRQt8O_EsggCvwDwAAAAABxWIYcPmhqIz9obJGlAJe_g?e=n9QZa1) | [gt](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsREAAAAAAB46eVZ0ErQZ9r1OS348S_5Q?e=qlUyK7) |
| Dynamic01 (2.14 GB, 175 s) | [bag](https://1drv.ms/u/c/2b4bfc0edf421186/EYYRQt8O_EsggCv0DwAAAAAB-86r95z48cuIi_MTyIoq8A?e=IiMGzk) | [gt](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsOEAAAAAABoct7u6wv4vWo3w3qZMOmtg?e=Lv6zoE) |
| Dark03 (2.01 GB, 170 s) | [bag](https://1drv.ms/u/c/2b4bfc0edf421186/EYYRQt8O_EsggCvtDwAAAAABaYJ_eK1OpYXLC_kXjU67nQ?e=fdMMHJ) | [gt](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsUEAAAAAABKHLQ0e1-Vp2saX0_ZDNYig?e=rrsULa) |

Links copied from the M3DGR README on 2026-10-08. If they break, take the current ones from there.

## 2. Convert the bag (once per sequence, pure Python, no ROS needed)

```bash
pip install -r requirements.txt
python tools/glim_prepare_bag.py --bag data/M3DGR/Varying-illu01.bag --sequence Varying-illu01
# -> data/M3DGR/Varying-illu01/glim_ros2/   (IMU + PointCloud2 with per-point times, ROS 2 Jazzy)
```

## 3. Run

```bash
docker pull koide3/glim_ros2:jazzy_cuda13.1
python -m pipeline.run_all --system glim --sequence Varying-illu01 --run 0
```

Output: `results/glim/<seq>/run_00/{trajectory_tum.txt, map.ply, glim.log, meta.json}`.
GLIM's full dump is kept in `data/M3DGR/<seq>/glim_dumps/run_00/` and can be opened with
`ros2 run glim_ros offline_viewer` (File > Open Map) for screenshots.

## Config

`config/` is the complete default config of GLIM master `1a197c9` (2026-10-08); check it against the
image version (`/opt/ros/*/share/glim/config`) after the pull. The only changes (all listed in
`config/systems/glim.yaml` under `deviations_from_defaults`):

- `config_ros.json`: topics `/livox/mid360/imu` and `/livox/mid360/lidar`, `acc_scale: 9.80665`
  (Livox IMU reports g), viewer extension modules removed for headless runs.
- `config_sensors.json`: `T_lidar_imu = [0.011, 0.02329, -0.04412, 0, 0, 0, 1]`. M3DGR `calibration.md`
  gives `mid360 -> mid360_imu` as `p_imu = p_lidar + [-0.011, -0.02329, 0.04412]` (identity rotation);
  GLIM wants the inverse (`p_lidar = T_lidar_imu * p_imu`).

## Open points

- Pin `source.commit` and the image digest in `config/systems/glim.yaml`.
- Check on a real bag: the Livox header stamp vs. `timebase` (converter handles both), that the IMU
  acceleration is really in g, and that GLIM's log shows no per-point-time warnings.
- Trajectory frame (`traj_imu.txt` vs. `traj_lidar.txt`) vs. the mocap body frame, once
  `gt_to_base_extrinsic` in `config/sequences.yaml` is filled in.
- resources.csv does not see processes inside the container. Use `docker stats` if needed.
