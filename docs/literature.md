# Literature Review: SLAM Comparison on Ground Robots

Status 2026-10-08. Almost all entries were checked against Crossref, arXiv, publisher pages or the PDF.
Entries marked **[verify]** must be double-checked before submission.
BibTeX keys in parentheses refer to [references.bib](../paper/references.bib).
⭐ = especially important for the 3-page paper.

## A. Foundations and surveys (for "State of the Art")

| Reference | Use |
|---|---|
| ⭐ Cadena et al., *Past, Present, and Future of SLAM: Toward the Robust-Perception Age*, IEEE T-RO 32(6), 2016 (`cadena2016past`) | Standard reference: SLAM as MAP estimation / factor graph, front-end vs. back-end, robustness |
| Durrant-Whyte & Bailey, *Simultaneous Localization and Mapping: Part I*, IEEE RAM 13(2), 2006 (`durrantwhyte2006slam`) | Classic introduction, probabilistic formulation, EKF-SLAM |
| Bailey & Durrant-Whyte, *SLAM: Part II*, IEEE RAM 13(3), 2006 (`bailey2006slam`) | Complexity, data association |
| ⭐ Grisetti et al., *A Tutorial on Graph-Based SLAM*, IEEE ITS Magazine 2(4), 2010 (`grisetti2010tutorial`) | Pose-graph formulation (back-end of most modern SLAM systems) |
| Thrun, Burgard, Fox, *Probabilistic Robotics*, MIT Press, 2005 (`thrun2005probabilistic`) | Textbook: Bayes filters, EKF, occupancy grids |
| Yue, Zhang, He, *LiDAR-based SLAM for robotic mapping: state of the art and new frontiers*, Industrial Robot 51(2), 2024 (`yue2024lidar`) | Recent LiDAR SLAM survey |
| Tourani et al., *Visual SLAM: What Are the Current Trends and What to Expect?*, Sensors 22(23), 2022 (`tourani2022visual`) | Recent visual SLAM survey |

## B. Core systems

Each member adds the paper(s) for their own chosen system here and to `paper/references.bib`.

| Reference | System |
|---|---|
| ⭐ Murai, Dexheimer, Davison, *MASt3R-SLAM: Real-Time Dense SLAM with 3D Reconstruction Priors*, CVPR 2025 (`murai2025mast3rslam`) | Monocular, 3D foundation-model prior, dense (Elias) |
| Leroy, Cabon, Revaud, *Grounding Image Matching in 3D with MASt3R*, ECCV 2024 (`leroy2024mast3r`) | The foundation model behind MASt3R-SLAM |
| Wang et al., *DUSt3R: Geometric 3D Vision Made Easy*, CVPR 2024 (`wang2024dust3r`) | Predecessor of MASt3R |
| *(system chosen by Philip Stix)* | |
| *(system chosen by Viktoriia Ovdiienko)* | |
| *(system chosen by Jiayi Zhou)* | |

## C. RGB-only deep dive (appendix)

| Reference | System |
|---|---|
| Korovko et al., *cuVSLAM: CUDA accelerated visual odometry and mapping*, arXiv:2506.04359, 2025 (`korovko2025cuvslam`) | NVIDIA, classical geometric, GPU-accelerated |
| Lipson, Teed, Deng, *Deep Patch Visual SLAM*, ECCV 2024 (`lipson2024dpvslam`) **[verify arXiv ID]** | Learned sparse patches + loop closure |
| Teed, Lipson, Deng, *Deep Patch Visual Odometry*, NeurIPS 2023 (`teed2023dpvo`) | Front-end of DPV-SLAM |
| Matsuki, Murai, Kelly, Davison, *Gaussian Splatting SLAM*, CVPR 2024 (`matsuki2024gaussian`) | First monocular SLAM based purely on 3D Gaussian splatting (MonoGS) |
| Teed & Deng, *DROID-SLAM*, NeurIPS 2021 (`teed2021droid`) | Related work only (too GPU-heavy) |
| Campos et al., *ORB-SLAM3*, IEEE T-RO 37(6), 2021 (`campos2021orbslam3`) | Classical monocular fallback; evaluation protocol: **median of 10 runs** |

## D. Datasets

| Reference | Role |
|---|---|
| ⭐ Zhang et al., *Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework* (M3DGR), IROS 2025, arXiv:2507.08364 (`zhang2025m3dgr`) **[verify author list]** | Recommended dataset; baseline table for sanity checks |
| Yin et al., *M2DGR: A Multi-Sensor and Multi-Scenario SLAM Dataset for Ground Robots*, IEEE RA-L 7(2), 2022 (`yin2022m2dgr`) | Backup |
| Wei et al., *FusionPortableV2*, IJRR 2024, arXiv:2404.08563 (`wei2024fusionportablev2`) | Alternative |
| Shi et al., *Are We Ready for Service Robots? The OpenLORIS-Scene Datasets for Lifelong SLAM*, ICRA 2020 (`shi2020openloris`) | Alternative (visual only) |

## E. Metrics and evaluation (for "Materials and Methods")

| Reference | Use |
|---|---|
| ⭐ Sturm et al., *A Benchmark for the Evaluation of RGB-D SLAM Systems*, IROS 2012 (`sturm2012benchmark`) | **Definition of ATE and RPE** |
| ⭐ Zhang & Scaramuzza, *A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry*, IROS 2018 (`zhang2018tutorial`) | **Choice of alignment** per sensor modality (SE(3) / Sim(3) / 4-DoF), repeated runs |
| ⭐ Umeyama, *Least-Squares Estimation of Transformation Parameters Between Two Point Patterns*, IEEE TPAMI 13(4), 1991 (`umeyama1991least`) | Alignment method used by evo |
| ⭐ Grupp, *evo: Python package for the evaluation of odometry and SLAM*, GitHub, 2017 (`grupp2017evo`) | Tool (state the version) |
| Kümmerle et al., *On Measuring the Accuracy of SLAM Algorithms*, Autonomous Robots 27(4), 2009 (`kuemmerle2009measuring`) | Why a relative metric (RPE) is needed in addition to ATE |
| Geiger, Lenz, Urtasun, *Are we ready for autonomous driving? The KITTI vision benchmark suite*, CVPR 2012 (`geiger2012kitti`) | Drift in % per distance (optional) |
| Knapitsch et al., *Tanks and Temples*, ACM TOG 36(4), 2017 (`knapitsch2017tanks`) | Accuracy / completeness / F-score for point-cloud maps (reference map required) |
| Santos, Portugal, Rocha, *An Evaluation of 2D SLAM Techniques Available in ROS*, SSRR 2013 (`santos2013evaluation`) | 2D map error against a reference map |
| Wang, Bovik, Sheikh, Simoncelli, *Image Quality Assessment: From Error Visibility to Structural Similarity*, IEEE TIP 13(4), 2004 (`wang2004ssim`) | SSIM (rendering quality, MonoGS) |
| Zhang, Isola, Efros, Shechtman, Wang, *The Unreasonable Effectiveness of Deep Features as a Perceptual Metric*, CVPR 2018 (`zhang2018lpips`) | LPIPS (rendering quality, MonoGS) |

## F. Reproducibility, resources, comparison studies (role models)

| Reference | Use |
|---|---|
| ⭐ Trejos et al., *2D SLAM Algorithms Characterization, Calibration, and Comparison Considering Pose Error, Map Accuracy as Well as CPU and Memory Usage*, Sensors 22(18), 2022 (`trejos2022slam`) | **Closest role model for our study**: ground robot, pose + map + CPU/RAM, hypothesis tests |
| Bujanca et al., *SLAMBench 3.0*, ICRA 2019 (`bujanca2019slambench3`) **[verify DOI]** | Automated, reproducible evaluation (accuracy, runtime, memory) |
| Radulov et al., *A Framework for Reproducible Benchmarking and Performance Diagnosis of SLAM Systems*, IROS 2024, arXiv:2410.04242 (`radulov2024slamfuse`) | Argument for containers |
| Fontan et al., *VSLAM-LAB*, arXiv:2504.04457, 2025 (`fontan2025vslamlab`) | Box plots over repeated runs because of non-determinism |
| Hamer, Albonico, Malavolta, *Resource Utilization of 2D SLAM Algorithms in ROS-Based Systems*, JBCS 31(1), 2025 (`hamer2025resource`) | CPU/energy, parameter sensitivity (defaults vs. tuned) |
| Prokhorov et al., *Measuring Robustness of Visual SLAM*, arXiv:1910.04755, 2019 (`prokhorov2019robustness`) | Report robustness separately from accuracy |

## G. Simulation (only if plan B Gazebo)

| Reference | Use |
|---|---|
| Wang et al., *TartanAir*, IROS 2020, arXiv:2003.14338 | Argument about the sim-to-real gap |
| TartanGround, arXiv:2505.10696, 2025 **[verify]** | Simulated ground-robot trajectories with GT |

## Key takeaways for the methodology

1. **ATE:** ATE measures the global consistency of the trajectory. The relative pose error (RPE) was dropped after feedback from the lecturer. Kümmerle et al. 2009 point out that ATE over-weights early errors; mention this as a limitation.
2. **Alignment:** The alignment follows what the sensor cannot observe (Zhang & Scaramuzza).
   - LiDAR and RGB-D are metric, so they get SE(3).
   - A monocular RGB camera cannot observe scale, so it gets Sim(3), and the scale factor is reported.
3. **Repeated runs:** SLAM is **non-deterministic**, so we run it several times and report the median instead of a single value (ORB-SLAM3, VSLAM-LAB). Failed runs are reported, not dropped (Prokhorov et al.).
4. **Resources:** Resource usage belongs to a fair comparison and must be reported together with the hardware it was measured on (SLAMBench, Trejos et al.).
5. **Reproducibility:** Containers and pinned versions make the results reproducible (SLAMFuse).
