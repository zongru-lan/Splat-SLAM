import csv
import json
import math
import os
import shutil
from pathlib import Path

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from skimage.metrics import structural_similarity
from torchmetrics.image.lpip import LearnedPerceptualImagePatchSimilarity


FINAL_DIRS = ("renders", "poses", "metrics")
CLEAN_OUTPUT_ITEMS = {"config.yaml", *FINAL_DIRS}


POSE_FIELDS = [
    "gt_frame_index",
    "gt_frame_id",
    "gt_image_name",
    "gt_image_path",
    "gt_timestamp",
    "gt_pose_index",
    "gt_pose_time_error_sec",
    "video_idx",
    "render_name",
    "pose_file",
    "tx",
    "ty",
    "tz",
    "qx",
    "qy",
    "qz",
    "qw",
] + [f"m{r}{c}" for r in range(4) for c in range(4)]


RENDER_FIELDS = [
    "gt_frame_index",
    "gt_frame_id",
    "gt_image_name",
    "gt_image_path",
    "gt_timestamp",
    "render_name",
    "reference_width",
    "reference_height",
    "render_width",
    "render_height",
    "psnr",
    "ssim",
    "lpips",
]


ATE_TRAJ_FIELDS = [
    "gt_frame_index",
    "gt_frame_id",
    "gt_image_name",
    "gt_timestamp",
    "est_tx",
    "est_ty",
    "est_tz",
    "gt_tx",
    "gt_ty",
    "gt_tz",
    "sim3_est_tx",
    "sim3_est_ty",
    "sim3_est_tz",
    "raw_error",
    "se3_error",
    "sim3_error",
]


def export_railway_outputs(slam, mapper, dataset):
    """Write the clean railway output package after normal Splat-SLAM termination."""
    if slam.cfg.get("dataset") != "railway":
        return None
    if mapper is None or getattr(mapper, "gaussians", None) is None:
        raise RuntimeError("Railway export requires a finished mapping run with a Gaussian map.")

    save_dir = Path(slam.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    final_dirs = {name: save_dir / name for name in FINAL_DIRS}
    for path in final_dirs.values():
        _replace_dir(path)

    _write_config(slam.cfg, save_dir / "config.yaml")

    lpips_device = _metric_device(slam.device)
    lpips_metric = LearnedPerceptualImagePatchSimilarity(
        net_type="alex", normalize=True
    ).to(lpips_device)
    lpips_metric.eval()

    records, render_rows = _export_keyframes(
        mapper=mapper,
        dataset=dataset,
        renders_dir=final_dirs["renders"],
        poses_dir=final_dirs["poses"],
        lpips_metric=lpips_metric,
        lpips_device=lpips_device,
    )
    if not records:
        raise RuntimeError("No keyframe records were available for railway export.")

    _write_csv(final_dirs["poses"] / "estimated_c2w.csv", POSE_FIELDS, [r["pose_csv"] for r in records])
    _write_tum(final_dirs["poses"] / "estimated_c2w_tum.txt", records)
    _write_pose_readme(final_dirs["poses"] / "README.md")

    _write_csv(final_dirs["metrics"] / "render_metrics.csv", RENDER_FIELDS, render_rows)
    _write_render_summary(final_dirs["metrics"] / "render_metrics_summary.json", render_rows, dataset)
    _write_ate_outputs(final_dirs["metrics"], records, dataset)

    _cleanup_internal_artifacts(save_dir)
    return {
        "save_dir": str(save_dir),
        "num_keyframes": len(records),
        "renders_dir": str(final_dirs["renders"]),
        "poses_dir": str(final_dirs["poses"]),
        "metrics_dir": str(final_dirs["metrics"]),
    }


def _export_keyframes(mapper, dataset, renders_dir, poses_dir, lpips_metric, lpips_device):
    records = []
    render_rows = []
    pairs = list(zip(mapper.keyframe_idxs, mapper.video_idxs))

    with torch.no_grad():
        for order, (gt_frame_index, video_idx) in enumerate(pairs):
            gt_frame_index = int(gt_frame_index)
            video_idx = int(video_idx)
            if video_idx not in mapper.cameras:
                continue

            frame = mapper.cameras[video_idx]
            info = _frame_info(dataset, gt_frame_index)
            render_name = f"{info['gt_frame_id']}.png"
            pose_name = f"{info['gt_frame_id']}.txt"
            render_path = renders_dir / render_name
            pose_path = poses_dir / pose_name

            image = _render_final_rgb(frame, mapper, order)
            reference_rgb = _load_reference_rgb(dataset, gt_frame_index)
            render_rgb = _resize_render_to_reference(image, reference_rgb.shape[:2])
            _write_png_rgb(render_path, render_rgb)

            c2w = _camera_c2w(frame)
            np.savetxt(pose_path, c2w, fmt="%.9f")

            pose_csv = _pose_csv_row(info, video_idx, render_name, pose_name, c2w)
            records.append({
                "info": info,
                "video_idx": video_idx,
                "render_name": render_name,
                "pose_file": pose_name,
                "c2w": c2w,
                "pose_csv": pose_csv,
            })

            metrics = _render_metrics(reference_rgb, render_rgb, lpips_metric, lpips_device)
            render_rows.append({
                "gt_frame_index": gt_frame_index,
                "gt_frame_id": info["gt_frame_id"],
                "gt_image_name": info["gt_image_name"],
                "gt_image_path": info["gt_image_path"],
                "gt_timestamp": info["gt_timestamp"],
                "render_name": render_name,
                "reference_width": int(reference_rgb.shape[1]),
                "reference_height": int(reference_rgb.shape[0]),
                "render_width": int(render_rgb.shape[1]),
                "render_height": int(render_rgb.shape[0]),
                **metrics,
            })

    return records, render_rows


def _render_final_rgb(frame, mapper, order):
    from thirdparty.gaussian_splatting.gaussian_renderer import render

    rendering_pkg = render(frame, mapper.gaussians, mapper.pipeline_params, mapper.background)
    if rendering_pkg is None:
        raise RuntimeError(f"Renderer returned no image for video_idx={frame.uid}.")
    image = rendering_pkg["render"].detach()
    if order > 0:
        image = torch.exp(frame.exposure_a.detach()) * image + frame.exposure_b.detach()
    return torch.clamp(image, 0.0, 1.0)


def _resize_render_to_reference(image, reference_hw):
    render_rgb = image.detach().cpu().permute(1, 2, 0).numpy()
    render_rgb = np.clip(render_rgb, 0.0, 1.0)
    ref_h, ref_w = reference_hw
    render_full = cv2.resize(render_rgb, (ref_w, ref_h), interpolation=cv2.INTER_LINEAR)
    return np.clip(np.rint(render_full * 255.0), 0, 255).astype(np.uint8)


def _load_reference_rgb(dataset, index):
    path = dataset.color_paths[index]
    image_bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f"Failed to read railway RGB image: {path}")
    if getattr(dataset, "distortion", None) is not None:
        k_mat = np.eye(3, dtype=np.float64)
        k_mat[0, 0] = dataset.fx_orig
        k_mat[0, 2] = dataset.cx_orig
        k_mat[1, 1] = dataset.fy_orig
        k_mat[1, 2] = dataset.cy_orig
        image_bgr = cv2.undistort(image_bgr, k_mat, dataset.distortion)
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def _write_png_rgb(path, image_rgb):
    ok = cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError(f"Failed to write render image: {path}")


def _render_metrics(reference_rgb, render_rgb, lpips_metric, lpips_device):
    return {
        "psnr": _psnr_uint8(reference_rgb, render_rgb),
        "ssim": float(structural_similarity(reference_rgb, render_rgb, channel_axis=2, data_range=255)),
        "lpips": _lpips_uint8(reference_rgb, render_rgb, lpips_metric, lpips_device),
    }


def _psnr_uint8(reference_rgb, render_rgb):
    diff = reference_rgb.astype(np.float64) - render_rgb.astype(np.float64)
    mse = float(np.mean(diff * diff))
    if mse == 0.0:
        return float("inf")
    return float(20.0 * math.log10(255.0 / math.sqrt(mse)))


def _lpips_uint8(reference_rgb, render_rgb, lpips_metric, lpips_device, width=1024):
    ref_small = _resize_width(reference_rgb, width)
    render_small = _resize_width(render_rgb, width)
    ref_tensor = _rgb_uint8_to_tensor(ref_small).unsqueeze(0).to(lpips_device)
    render_tensor = _rgb_uint8_to_tensor(render_small).unsqueeze(0).to(lpips_device)
    lpips_metric.reset()
    with torch.no_grad():
        return float(lpips_metric(render_tensor, ref_tensor).detach().cpu().item())


def _resize_width(image_rgb, width):
    h, w = image_rgb.shape[:2]
    if w == width:
        return image_rgb
    new_h = max(1, int(round(h * width / w)))
    return cv2.resize(image_rgb, (width, new_h), interpolation=cv2.INTER_AREA)


def _rgb_uint8_to_tensor(image_rgb):
    return torch.from_numpy(image_rgb).permute(2, 0, 1).float() / 255.0


def _camera_c2w(frame):
    w2c = np.eye(4, dtype=np.float64)
    w2c[:3, :3] = _to_numpy(frame.R)
    w2c[:3, 3] = _to_numpy(frame.T)
    return np.linalg.inv(w2c)


def _to_numpy(value):
    if torch.is_tensor(value):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _frame_info(dataset, index):
    frame_id = _safe_index(getattr(dataset, "frame_ids", None), index, f"{index:05d}")
    image_name = _safe_index(getattr(dataset, "image_names", None), index, os.path.basename(dataset.color_paths[index]))
    image_path = dataset.color_paths[index]
    timestamp = _safe_index(getattr(dataset, "image_timestamps", None), index, float(index))
    gt_pose_index = _safe_index(getattr(dataset, "gt_pose_indices", None), index, "")
    gt_pose_time_error = _safe_index(getattr(dataset, "gt_pose_time_errors", None), index, "")
    return {
        "gt_frame_index": int(index),
        "gt_frame_id": str(frame_id),
        "gt_image_name": str(image_name),
        "gt_image_path": str(image_path),
        "gt_timestamp": float(timestamp),
        "gt_pose_index": "" if gt_pose_index == "" else int(gt_pose_index),
        "gt_pose_time_error_sec": "" if gt_pose_time_error == "" else float(gt_pose_time_error),
    }


def _safe_index(values, index, default):
    if values is None:
        return default
    try:
        return values[index]
    except (IndexError, KeyError, TypeError):
        return default


def _pose_csv_row(info, video_idx, render_name, pose_name, c2w):
    quat = Rotation.from_matrix(c2w[:3, :3]).as_quat()
    row = {
        **info,
        "video_idx": int(video_idx),
        "render_name": render_name,
        "pose_file": pose_name,
        "tx": float(c2w[0, 3]),
        "ty": float(c2w[1, 3]),
        "tz": float(c2w[2, 3]),
        "qx": float(quat[0]),
        "qy": float(quat[1]),
        "qz": float(quat[2]),
        "qw": float(quat[3]),
    }
    for r in range(4):
        for c in range(4):
            row[f"m{r}{c}"] = float(c2w[r, c])
    return row


def _write_tum(path, records):
    with open(path, "w", encoding="utf-8") as fp:
        fp.write("# timestamp tx ty tz qx qy qz qw\n")
        fp.write("# pose convention: T_world_camera / c2w in Splat-SLAM internal map frame\n")
        for record in records:
            row = record["pose_csv"]
            fp.write(
                f"{row['gt_timestamp']:.9f} {row['tx']:.9f} {row['ty']:.9f} {row['tz']:.9f} "
                f"{row['qx']:.9f} {row['qy']:.9f} {row['qz']:.9f} {row['qw']:.9f}\n"
            )


def _write_ate_outputs(metrics_dir, records, dataset):
    est_xyz = np.asarray([record["c2w"][:3, 3] for record in records], dtype=np.float64)
    gt_c2w = np.asarray([dataset.poses[record["info"]["gt_frame_index"]] for record in records], dtype=np.float64)
    gt_xyz = gt_c2w[:, :3, 3]

    raw_aligned = est_xyz.copy()
    se3_aligned, se3_transform = _align_points(est_xyz, gt_xyz, with_scale=False)
    sim3_aligned, sim3_transform = _align_points(est_xyz, gt_xyz, with_scale=True)

    raw_errors = np.linalg.norm(raw_aligned - gt_xyz, axis=1)
    se3_errors = np.linalg.norm(se3_aligned - gt_xyz, axis=1)
    sim3_errors = np.linalg.norm(sim3_aligned - gt_xyz, axis=1)

    traj_rows = []
    for idx, record in enumerate(records):
        info = record["info"]
        traj_rows.append({
            "gt_frame_index": info["gt_frame_index"],
            "gt_frame_id": info["gt_frame_id"],
            "gt_image_name": info["gt_image_name"],
            "gt_timestamp": info["gt_timestamp"],
            "est_tx": float(est_xyz[idx, 0]),
            "est_ty": float(est_xyz[idx, 1]),
            "est_tz": float(est_xyz[idx, 2]),
            "gt_tx": float(gt_xyz[idx, 0]),
            "gt_ty": float(gt_xyz[idx, 1]),
            "gt_tz": float(gt_xyz[idx, 2]),
            "sim3_est_tx": float(sim3_aligned[idx, 0]),
            "sim3_est_ty": float(sim3_aligned[idx, 1]),
            "sim3_est_tz": float(sim3_aligned[idx, 2]),
            "raw_error": float(raw_errors[idx]),
            "se3_error": float(se3_errors[idx]),
            "sim3_error": float(sim3_errors[idx]),
        })
    _write_csv(metrics_dir / "ate_aligned_trajectory.csv", ATE_TRAJ_FIELDS, traj_rows)

    output = {
        "sequence": getattr(dataset, "input_folder", ""),
        "num_keyframes": len(records),
        "pose_convention": "estimated T_world_camera / c2w in Splat-SLAM internal map frame",
        "gt_convention": "RailwayDataset normalized c2w, initialized at the first GT pose",
        "matching_key": "gt_timestamp",
        "gt_pose_path": str(getattr(dataset, "gt_pose_path", "")),
        "primary_metric": "sim3_aligned.rmse",
        "raw": _error_stats(raw_errors),
        "se3_aligned": {
            **_error_stats(se3_errors),
            "scale": 1.0,
            "rotation_est_to_gt": se3_transform["rotation"].tolist(),
            "translation_est_to_gt": se3_transform["translation"].tolist(),
        },
        "sim3_aligned": {
            **_error_stats(sim3_errors),
            "scale": float(sim3_transform["scale"]),
            "rotation_est_to_gt": sim3_transform["rotation"].tolist(),
            "translation_est_to_gt": sim3_transform["translation"].tolist(),
        },
    }
    _write_json(metrics_dir / "ate_metrics.json", output)


def _align_points(src, dst, with_scale):
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("ATE alignment expects source and target arrays with shape Nx3.")
    if len(src) < 2:
        rotation = np.eye(3, dtype=np.float64)
        scale = 1.0
        translation = dst.mean(axis=0) - src.mean(axis=0)
        aligned = src + translation
        return aligned, {"scale": scale, "rotation": rotation, "translation": translation}

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    covariance = (dst_centered.T @ src_centered) / len(src)
    u_mat, singular_values, vt_mat = np.linalg.svd(covariance)
    det_fix = np.eye(3, dtype=np.float64)
    if np.linalg.det(u_mat @ vt_mat) < 0:
        det_fix[-1, -1] = -1.0
    rotation = u_mat @ det_fix @ vt_mat
    if with_scale:
        variance = float(np.sum(src_centered * src_centered) / len(src))
        scale = float(np.trace(np.diag(singular_values) @ det_fix) / variance) if variance > 0 else 1.0
    else:
        scale = 1.0
    translation = dst_mean - scale * (rotation @ src_mean)
    aligned = (scale * (rotation @ src.T)).T + translation
    return aligned, {"scale": scale, "rotation": rotation, "translation": translation}


def _error_stats(errors):
    errors = np.asarray(errors, dtype=np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "std": float(np.std(errors)),
        "min": float(np.min(errors)),
        "max": float(np.max(errors)),
    }


def _write_render_summary(path, rows, dataset):
    summary = {
        "num_keyframes": len(rows),
        "final_map_rerender": True,
        "render_directory": "renders",
        "render_naming": "<gt_frame_id>.png",
        "reference_resolution": {
            "height": int(dataset.H),
            "width": int(dataset.W),
        },
        "saved_render_resolution": {
            "height": int(dataset.H),
            "width": int(dataset.W),
        },
        "lpips_resize_width": 1024,
        "reference_image_preprocessing": "full-resolution RGB with the same optional undistortion used by RailwayDataset",
        "mean_psnr": _mean_finite([row["psnr"] for row in rows]),
        "mean_ssim": _mean_finite([row["ssim"] for row in rows]),
        "mean_lpips": _mean_finite([row["lpips"] for row in rows]),
    }
    _write_json(path, summary)


def _mean_finite(values):
    finite = [float(value) for value in values if np.isfinite(float(value))]
    return float(np.mean(finite)) if finite else None


def _write_pose_readme(path):
    path.write_text(
        "# Pose Convention\n\n"
        "All pose files in this directory store `T_world_camera / c2w`.\n\n"
        "The camera frame uses +x right, +y down, +z forward. The world frame is the "
        "Splat-SLAM internal map frame initialized during monocular SLAM. It is not GPS, "
        "ENU, latitude/longitude, or the raw railway `gt_poses` global frame.\n\n"
        "`estimated_c2w.csv` stores one row per exported keyframe and includes GT frame "
        "association fields (`gt_frame_index`, `gt_frame_id`, `gt_image_name`, "
        "`gt_timestamp`, `gt_pose_index`) so ATE matching can be reproduced safely.\n",
        encoding="utf-8",
    )


def _write_config(cfg, path):
    plain_cfg = _to_plain(cfg)
    try:
        import yaml
        with open(path, "w", encoding="utf-8") as fp:
            yaml.safe_dump(plain_cfg, fp, sort_keys=False)
    except Exception:
        _write_json(path, plain_cfg)


def _to_plain(value):
    if isinstance(value, dict):
        return {str(k): _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    return value


def _write_csv(path, fieldnames, rows):
    with open(path, "w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_json(path, payload):
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, indent=2, ensure_ascii=False, allow_nan=False)


def _replace_dir(path):
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def _cleanup_internal_artifacts(save_dir):
    for path in save_dir.iterdir():
        if path.name in CLEAN_OUTPUT_ITEMS:
            continue
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()


def _metric_device(config_device):
    if torch.cuda.is_available() and str(config_device).startswith("cuda"):
        return torch.device(config_device)
    return torch.device("cpu")
