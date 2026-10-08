# Monocular RGB SLAM options (Elias)

Candidate SLAM systems that use only a single RGB camera. MASt3R-SLAM is the main system for the group comparison; the others form the optional RGB-only deep dive (see [PLAN.md, Section 3.2](PLAN.md#32-rgb-only-deep-dive-elias-optional-extension)).

| System | Approach | Strength | Hardware |
|---|---|---|---|
| **MASt3R-SLAM** (Murai et al., CVPR 2025), main choice | 3D foundation model, dense map | robust, no camera calibration required | large GPU (authors used an RTX 4090) |
| **cuVSLAM** (Korovko et al., NVIDIA, 2025) | classical geometric, CUDA-accelerated | very fast, even runs on Jetson | low |
| **DPV-SLAM** (Lipson et al., ECCV 2024) | learned image patches + loop closure | good trade-off between learning and speed | mid-range GPU |
| **MonoGS / Gaussian Splatting SLAM** (Matsuki et al., CVPR 2024) | photorealistic 3D Gaussian map | map quality measurable (PSNR / SSIM / LPIPS) | large GPU, slow |
| ORB-SLAM3 monocular (Campos et al., T-RO 2021) | classical, feature points | standard reference, CPU only | low |

## Not included

- **FoundationSLAM** (Wu et al., AAAI 2026): no public code found.
- **DROID-SLAM** (Teed and Deng, NeurIPS 2021): needs at least 11 GB of GPU memory. The available RTX 3080 Ti Laptop GPU has 16 GB, so it may be feasible; not tested yet.

## Evaluation notes

- A single camera cannot observe metric scale, so all of these systems are evaluated with **Sim(3)** alignment (Zhang and Scaramuzza 2018), and the estimated scale factor is reported.
- Feasibility on the RTX 3080 Ti Laptop GPU (16 GB) is being tested on TUM RGB-D fr1/desk: peak GPU memory, runtime and ATE. Results will be added here.
