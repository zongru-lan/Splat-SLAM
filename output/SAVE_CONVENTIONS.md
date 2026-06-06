# Splat-SLAM Railway Output Save Conventions

Last updated: 2026-06-06

This document records the clean output convention for running Splat-SLAM on the railway monocular RGB sequences under `/home/leizongru/lzr_ws/railway_data`.

## 1. Output Root

All clean railway run outputs are saved under:

```text
/home/leizongru/lzr_ws/Splat-SLAM/output/
```

Each sequence uses one directory named by the sequence name:

```text
/home/leizongru/lzr_ws/Splat-SLAM/output/<sequence_name>/
```

Example:

```text
/home/leizongru/lzr_ws/Splat-SLAM/output/scene_14_train/
```

Only this convention document is intended to be tracked under `output/`. Sequence result directories are run artifacts and should remain untracked.

## 2. Expected Directory Layout

After a normal railway run finishes, the railway export stage keeps the result directory clean:

```text
output/<sequence_name>/
├── config.yaml
├── renders/
│   └── <gt_frame_id>.png
├── poses/
│   ├── <gt_frame_id>.txt
│   ├── estimated_c2w.csv
│   ├── estimated_c2w_tum.txt
│   └── README.md
└── metrics/
    ├── render_metrics.csv
    ├── render_metrics_summary.json
    ├── ate_metrics.json
    └── ate_aligned_trajectory.csv
```

Splat-SLAM may create internal temporary artifacts while running, such as `mono_priors/`, `video.npz`, `traj/`, `psnr/`, and `plots_after_refine/`. The railway export stage runs at the end of `SLAM.terminate()` and then leaves only `config.yaml`, `renders/`, `poses/`, and `metrics/` in the sequence directory.

## 3. Final-Map Render Images

Render images are saved under:

```text
output/<sequence_name>/renders/
```

Only keyframes selected by Splat-SLAM are exported. Each image is rendered after final refinement using the final Gaussian map. Online or intermediate renders are not kept.

Each render is a single RGB image, not a multi-panel visualization. Splat-SLAM renders at the configured frontend resolution and the railway export stage upsamples the result to the original railway RGB resolution:

```text
height = 2504
width  = 4112
```

The render filename is derived from the corresponding GT image filename prefix before the first underscore.

Example:

```text
GT image:
/home/leizongru/lzr_ws/railway_data/scene_05_train/048_1637935055.000000000.png

Saved render:
/home/leizongru/lzr_ws/Splat-SLAM/output/scene_05_train/renders/048.png
```

For metric computation, the GT reference RGB is loaded at full resolution and passed through the same optional undistortion used by `RailwayDataset` before resizing in the main SLAM pipeline.

## 4. Estimated Pose Files

Pose files are saved under:

```text
output/<sequence_name>/poses/
```

For each exported keyframe, a per-frame pose matrix is saved as:

```text
poses/<gt_frame_id>.txt
```

Each `.txt` file stores one `4 x 4` homogeneous matrix.

The pose convention is:

```text
T_world_camera / c2w
```

Meaning: the matrix transforms homogeneous points from the camera frame to the Splat-SLAM internal world/map frame.

Coordinate frames:

```text
camera frame: +x right, +y down, +z forward
world/map frame: initialized by Splat-SLAM during monocular SLAM; not GPS, ENU, latitude/longitude, or the raw railway gt_poses global frame
```

## 5. Pose CSV For ATE Evaluation

The main trajectory file is:

```text
poses/estimated_c2w.csv
```

Each row corresponds to one exported keyframe pose and includes explicit GT association fields to avoid ATE matching mistakes.

Important columns:

```text
gt_frame_index   zero-based index in the sorted image list loaded by RailwayDataset
gt_frame_id      prefix before the first underscore in the GT image filename, e.g. 048
gt_image_name    original GT image filename, e.g. 048_1637935055.000000000.png
gt_image_path    full path to the corresponding GT image
gt_timestamp     timestamp parsed from the GT image filename and used for matching gt_poses/*.parquet
gt_pose_index    matched row index in gt_poses/<sequence_name>.parquet
gt_pose_time_error_sec  timestamp difference between image and matched GT pose
video_idx         Splat-SLAM internal keyframe index
render_name      saved render filename, e.g. 048.png
pose_file        saved 4x4 pose matrix filename, e.g. 048.txt
```

Pose values in the CSV:

```text
tx ty tz          translation from T_world_camera
qx qy qz qw       quaternion from T_world_camera rotation
m00 ... m33       flattened 4x4 T_world_camera matrix, row-major
```

Recommended ATE matching key:

```text
gt_timestamp
```

## 6. TUM-Style Trajectory

A TUM-style trajectory is also saved:

```text
poses/estimated_c2w_tum.txt
```

Format:

```text
gt_timestamp tx ty tz qx qy qz qw
```

This file uses the same `T_world_camera / c2w` convention as the CSV and per-frame `.txt` pose files.

## 7. Rendering Metrics

Rendering metrics are saved under:

```text
output/<sequence_name>/metrics/
```

Files:

```text
render_metrics.csv
render_metrics_summary.json
```

`render_metrics.csv` contains one row per final-map rerendered keyframe.

Metric convention:

```text
PSNR/SSIM: computed between the saved final render and the full-resolution GT reference RGB at 2504 x 4112.
LPIPS: computed after resizing both RGB images to width 1024 while preserving aspect ratio.
```

## 8. ATE Metrics

ATE results are saved under:

```text
output/<sequence_name>/metrics/
```

Files:

```text
ate_metrics.json
ate_aligned_trajectory.csv
```

ATE convention:

```text
estimated trajectory: translation from poses/estimated_c2w.csv, T_world_camera / c2w in the Splat-SLAM internal world/map frame
GT trajectory: RailwayDataset c2w poses matched by gt_timestamp and normalized to the first GT pose
matching key: gt_timestamp / gt_pose_index saved in estimated_c2w.csv
primary metric: Sim(3) aligned ATE RMSE
```

`ate_metrics.json` also stores raw and SE(3)-aligned ATE for reference. For monocular VO, use the Sim(3)-aligned RMSE as the primary value.
