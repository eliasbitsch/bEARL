"""Run PyCuVSLAM on a TUM RGB-D sequence and write a TUM-format trajectory.

Modes:
  mono  - OdometryMode.Mono on the RGB stream (Brown distortion model, raw images)
  rgbd  - OdometryMode.RGBD on undistorted RGB + depth pairs (pinhole model)

With --slam a SLAM config is passed to the tracker (loop closure + pose graph,
synchronous mode) and the final trajectory is taken from get_all_slam_poses().
Written against the PyCuVSLAM v17.0.0 wheel API (cuvslam.Tracker(rig, odom_cfg, slam_cfg)).
Dataset loading follows examples/tum/dataset_utils.py of the upstream repository.
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
from PIL import Image

import cuvslam

# TUM freiburg1 RGB intrinsics (https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
FX, FY, CX, CY = 517.3, 516.5, 318.6, 255.3
K1, K2, P1, P2, K3 = 0.2624, -0.9531, -0.0054, 0.0026, 1.1633
WIDTH, HEIGHT = 640, 480
DEPTH_SCALE = 5000.0


def read_list(path):
    out = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            out.append((float(parts[0]), parts[1]))
    return out


def match_pairs(rgb, depth, max_dt=0.02):
    """Greedy timestamp matching as in upstream dataset_utils.find_matching_pairs."""
    i = j = 0
    pairs = []
    while i < len(rgb) and j < len(depth):
        dt = abs(rgb[i][0] - depth[j][0])
        if dt < max_dt:
            pairs.append((rgb[i][0], rgb[i][1], depth[j][1]))
            i += 1
            j += 1
        elif rgb[i][0] < depth[j][0]:
            i += 1
        else:
            j += 1
    return pairs


def load_bgr(path):
    frame = np.array(Image.open(path).convert('RGB'))
    return np.ascontiguousarray(frame[:, :, ::-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--mode', choices=['mono', 'rgbd'], default='mono')
    ap.add_argument('--slam', action='store_true')
    ap.add_argument('--undistort', action='store_true',
                    help='mono: undistort with OpenCV and use a pinhole model instead of Brown')
    ap.add_argument('--gray', action='store_true', help='feed mono8 instead of BGR images')
    ap.add_argument('--odom-opt', action='append', default=[], metavar='KEY=VALUE',
                    help='extra OdometryConfig field, e.g. use_motion_model=0 (repeatable)')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rgb_list = read_list(os.path.join(args.dataset, 'rgb.txt'))
    kmat = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
    dist = np.array([K1, K2, P1, P2, K3], dtype=np.float64)

    camera = cuvslam.Camera()
    camera.size = (WIDTH, HEIGHT)
    camera.focal = (FX, FY)
    camera.principal = (CX, CY)

    map1, map2 = cv2.initUndistortRectifyMap(kmat, dist, None, kmat, (WIDTH, HEIGHT), cv2.CV_32FC1)
    if args.mode == 'mono':
        if not args.undistort:
            # cuVSLAM Brown order: k1, k2, k3, p1, p2
            camera.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Brown, [K1, K2, K3, P1, P2])
        frames = [(t, os.path.join(args.dataset, f), None) for t, f in rgb_list]
        odom_mode = cuvslam.Tracker.OdometryMode.Mono
        rgbd_settings = None
    else:
        depth_list = read_list(os.path.join(args.dataset, 'depth.txt'))
        frames = [(t, os.path.join(args.dataset, r), os.path.join(args.dataset, d))
                  for t, r, d in match_pairs(rgb_list, depth_list)]
        camera.border_top = 20
        camera.border_bottom = 20
        camera.border_left = 10
        camera.border_right = 50
        odom_mode = cuvslam.Tracker.OdometryMode.RGBD
        rgbd_settings = cuvslam.Tracker.OdometryRGBDSettings()
        rgbd_settings.depth_scale_factor = DEPTH_SCALE
        rgbd_settings.depth_camera_id = 0
        rgbd_settings.enable_depth_stereo_tracking = False

    odom_kwargs = dict(async_sba=False, odometry_mode=odom_mode)
    for kv in args.odom_opt:
        key, val = kv.split('=', 1)
        odom_kwargs[key] = float(val) if '.' in val else bool(int(val))
    if rgbd_settings is not None:
        odom_kwargs['rgbd_settings'] = rgbd_settings
    odom_cfg = cuvslam.Tracker.OdometryConfig(**odom_kwargs)

    rig = cuvslam.Rig([camera])
    if args.slam:
        slam_cfg = cuvslam.Tracker.SlamConfig(sync_mode=True)
        tracker = cuvslam.Tracker(rig, odom_cfg, slam_cfg)
    else:
        tracker = cuvslam.Tracker(rig, odom_cfg)

    odom_rows, slam_rows = [], []
    lost = 0
    track_times = []
    t_start = time.perf_counter()
    for idx, (ts, rgb_path, depth_path) in enumerate(frames):
        img = load_bgr(rgb_path)
        if args.undistort or depth_path is not None:
            img = np.ascontiguousarray(cv2.remap(img, map1, map2, cv2.INTER_LINEAR))
        if args.gray:
            img = np.ascontiguousarray(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        ts_ns = int(round(ts * 1e9))
        if depth_path is None:
            t0 = time.perf_counter()
            odom, slam_pose = tracker.track(ts_ns, [img])
        else:
            depth = np.array(Image.open(depth_path)).astype(np.uint16)
            depth = np.ascontiguousarray(cv2.remap(depth, map1, map2, cv2.INTER_NEAREST))
            t0 = time.perf_counter()
            odom, slam_pose = tracker.track(ts_ns, [img], depths=[depth])
        track_times.append(time.perf_counter() - t0)
        if odom.world_from_rig is None:
            lost += 1
            continue
        p = odom.world_from_rig.pose
        odom_rows.append((ts, *p.translation, *p.rotation))
        if slam_pose is not None:
            slam_rows.append((ts, *slam_pose.translation, *slam_pose.rotation))
    t_loop = time.perf_counter() - t_start

    stamp = {int(round(t * 1e9)): t for t, _, _ in frames}
    final_rows = odom_rows
    n_slam_final = 0
    lc_count = None
    if args.slam:
        poses = tracker.get_all_slam_poses()
        final_rows = [(stamp.get(ps.timestamp_ns, ps.timestamp_ns * 1e-9),
                       *ps.pose.translation, *ps.pose.rotation) for ps in poses]
        n_slam_final = len(final_rows)
        try:
            lc = tracker.get_loop_closure_poses()
            lc_count = len(lc)
        except Exception:
            lc_count = None

    def write(path, rows):
        with open(path, 'w') as f:
            for r in rows:
                f.write('%.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n' % tuple(r))

    write(os.path.join(args.out, 'traj_odom_tum.txt'), odom_rows)
    if args.slam:
        write(os.path.join(args.out, 'traj_slam_per_frame_tum.txt'), slam_rows)
    write(os.path.join(args.out, 'traj_tum.txt'), final_rows)

    tt = np.array(track_times)
    summary = dict(
        mode=args.mode, slam=args.slam, undistort=args.undistort, gray=args.gray, odom_opts=args.odom_opt, frames_in=len(frames), frames_tracked=len(odom_rows),
        frames_lost=lost, slam_poses=n_slam_final, loop_closures=lc_count,
        loop_wall_s=round(t_loop, 3), fps_loop=round(len(frames) / t_loop, 2),
        track_ms_mean=round(1e3 * tt.mean(), 2), track_ms_median=round(1e3 * np.median(tt), 2),
        track_ms_max=round(1e3 * tt.max(), 2),
    )
    with open(os.path.join(args.out, 'run_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
