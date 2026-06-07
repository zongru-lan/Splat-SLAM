# Splat-SLAM 铁路数据集批跑结果汇总

- 报告生成时间：2026-06-07 09:37:08 CST
- 批跑日志目录：`/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629`
- 输出根目录：`/home/leizongru/lzr_ws/Splat-SLAM/output`
- 数据集根目录：`/home/leizongru/lzr_ws/railway_data`
- 初始 GPU 设置：`2,3`
- 最终主要使用 GPU：`3`
- 批跑开始时间：`2026-06-06 19:46:29`
- 批跑结束时间：`2026-06-06 23:06:25`
- 总耗时：`3h 19m 56s`
- 最终有效结果：`7/7` 个序列完成
- 批跑过程中的失败尝试：`1` 次，`scene_13_train` 在 GPU2 上发生 CUDA OOM，随后已切到 GPU3 重新跑完
- 最终状态：成功完成，但部分序列的 ATE 明显异常，需要后续重点复查

## 总体结论

本次 Splat-SLAM 铁路数据集批跑最终完成了全部 7 个序列：`scene_05_train`、`scene_11_train`、`scene_13_train`、`scene_14_train`、`scene_16_train`、`scene_17_train`、`scene_19_train`。

每个序列都生成了约定的干净输出目录，包含最终 Gaussian map 重渲图、逐关键帧位姿、渲染指标和 ATE 指标。最终对用户暴露的目录结构保持为：

```text
output/<sequence_name>/
├── config.yaml
├── renders/
├── poses/
└── metrics/
```

需要特别注意的是，虽然输出口径已经通过校验，但轨迹质量并不完全可靠。`scene_05_train`、`scene_13_train`、`scene_19_train` 的 Sim(3) 对齐 ATE RMSE 明显偏大，其中 `scene_05_train` 重跑后仍然异常。因此这些序列的 ATE 不建议直接作为可信最终结果使用，应优先检查轨迹估计是否失败、GT 关联是否存在特殊情况、以及坐标/尺度对齐是否还有问题。

## 聚合指标

- 7 个序列 Sim(3) ATE RMSE 平均值：`33.062153`
- 最好 Sim(3) ATE RMSE：`scene_11_train` = `0.022580`
- 最差 Sim(3) ATE RMSE：`scene_05_train` = `168.927770`
- 7 个序列平均 PSNR：`19.8527`
- 7 个序列平均 SSIM：`0.5395`
- 7 个序列平均 LPIPS：`0.6421`

## 逐序列摘要

`输出校验` 表示保存内容是否符合我们约定的铁路输出口径；`轨迹备注` 只评价 ATE 数值是否需要进一步关注。

| 序列 | 状态 | 最终成功运行耗时 | 输入帧数 | 关键帧 | 渲染图 | 渲染尺寸 | PSNR Mean | SSIM Mean | LPIPS Mean | ATE Sim3 RMSE | ATE 匹配 | 最大时间误差(s) | 输出校验 | 轨迹备注 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `scene_05_train` | OK | 32m 28s | 259 | 57 | 57 | 2504x4112 | 17.7903 | 0.5449 | 0.7106 | 168.927770 | 57/57 | 0.000000238 | PASS | WARN |
| `scene_11_train` | OK | 14m 18s | 288 | 25 | 25 | 2504x4112 | 22.0924 | 0.6132 | 0.5918 | 0.022580 | 25/25 | 0.000000238 | PASS | OK |
| `scene_13_train` | OK | 15m 18s | 149 | 33 | 33 | 2504x4112 | 18.3284 | 0.4915 | 0.7180 | 50.570547 | 33/33 | 0.000000238 | PASS | WARN |
| `scene_14_train` | OK | 31m 05s | 298 | 52 | 52 | 2504x4112 | 22.6425 | 0.6947 | 0.5291 | 0.301003 | 52/52 | 0.000000238 | PASS | OK |
| `scene_16_train` | OK | 13m 33s | 179 | 24 | 24 | 2504x4112 | 19.5212 | 0.4970 | 0.6434 | 0.341635 | 24/24 | 0.000000000 | PASS | OK |
| `scene_17_train` | OK | 43m 15s | 235 | 67 | 67 | 2504x4112 | 20.0471 | 0.5727 | 0.6205 | 1.265123 | 67/67 | 0.000000238 | PASS | CHECK |
| `scene_19_train` | OK | 31m 23s | 289 | 52 | 52 | 2504x4112 | 18.5467 | 0.3622 | 0.6811 | 10.006415 | 52/52 | 0.000000238 | PASS | WARN |

## ATE 细节

单目 VO/SLAM 的主指标采用 Sim(3) 对齐后的 ATE RMSE。Raw 和 SE(3) 对齐结果保留为参考。

| 序列 | Raw RMSE | SE(3) RMSE | Sim(3) RMSE | 未匹配帧 |
|---|---:|---:|---:|---:|
| `scene_05_train` | 380.672866 | 169.042879 | 168.927770 | 0 |
| `scene_11_train` | 76.962941 | 25.330961 | 0.022580 | 0 |
| `scene_13_train` | 146.367317 | 51.178589 | 50.570547 | 0 |
| `scene_14_train` | 176.793050 | 72.767281 | 0.301003 | 0 |
| `scene_16_train` | 118.121476 | 46.281901 | 0.341635 | 0 |
| `scene_17_train` | 327.701183 | 147.494297 | 1.265123 | 0 |
| `scene_19_train` | 148.894599 | 60.616991 | 10.006415 | 0 |

## 输出校验

预期输出口径为：只保留最终 Gaussian map 重渲图，渲染图尺寸为 `2504 x 4112`，每张渲染图有一条对应位姿记录，ATE 通过 `gt_timestamp` / `gt_pose_index` 与 GT 关联。

| 序列 | Render PNGs | Render CSV Rows | Pose CSV Rows | Pose TXT Files | Metrics Files | PLY | Output Dir | Log File |
|---|---:|---:|---:|---:|---:|---|---|---|
| `scene_05_train` | 57 | 57 | 57 | 57 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_05_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_05_train.gpu3.attempt1.log` |
| `scene_11_train` | 25 | 25 | 25 | 25 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_11_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_11_train.gpu2.attempt1.log` |
| `scene_13_train` | 33 | 33 | 33 | 33 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_13_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_13_train.gpu3.attempt1.log` |
| `scene_14_train` | 52 | 52 | 52 | 52 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_14_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_14_train.gpu3.attempt1.log` |
| `scene_16_train` | 24 | 24 | 24 | 24 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_16_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_16_train.gpu3.attempt1.log` |
| `scene_17_train` | 67 | 67 | 67 | 67 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_17_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_17_train.gpu3.attempt1.log` |
| `scene_19_train` | 52 | 52 | 52 | 52 | 4 | 不对外保留 | `/home/leizongru/lzr_ws/Splat-SLAM/output/scene_19_train` | `/home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_20260606_194629/scene_19_train.gpu3.attempt1.log` |

## 报错扫描

最终成功日志中未发现新的 `Traceback`、`RuntimeError` 或 CUDA OOM。

批跑过程中有一个历史失败尝试：

```text
scene_13_train.gpu2.attempt1.log
RuntimeError: CUDA out of memory. Tried to allocate 1.07 GiB
```

该失败来自 GPU2 后来被其他任务占用导致显存不足。`scene_13_train` 已经被重新加入队列，并在 GPU3 上成功完成，最终输出目录以 GPU3 成功结果为准。

此外，成功日志尾部出现过 PyTorch CUDA IPC 资源释放提示：

```text
[W CudaIPCTypes.cpp:15] Producer process has been terminated before all shared CUDA tensors released.
```

该提示没有中断运行，也没有影响最终铁路导出阶段完成。

## 结果说明

- 渲染指标只基于最终 Gaussian map 的重渲图计算，不保留在线中间渲染图。
- PSNR 和 SSIM 在保存后的 GT 分辨率 `2504 x 4112` 上计算。
- LPIPS 计算时将渲染图和 GT 图等比例缩放到宽度 `1024`。
- 渲染图命名沿用铁路数据集 GT 帧 id，例如 `048.png`。
- 位姿文件为 `T_world_camera / c2w`，坐标系是 Splat-SLAM 内部单目 SLAM map frame，不是铁路 GT 原始全局坐标系。
- `poses/estimated_c2w.csv` 中保存了 `gt_frame_index`、`gt_frame_id`、`gt_image_name`、`gt_image_path`、`gt_timestamp`、`gt_pose_index`、`gt_pose_time_error_sec` 等字段，后续计算 ATE 时应优先使用这些字段做匹配。
- 这次 Splat-SLAM 的输出整理阶段遵循我们约定的跨项目原则：不大改算法流程，只在 `terminate()` 末尾增加铁路专用结果整理/导出阶段，让项目内部运行文件与最终用户可见结果分开。

## 后续建议

1. 优先检查 `scene_05_train`、`scene_13_train`、`scene_19_train` 的 ATE 异常，尤其是 `scene_05_train`，因为它重跑后仍然异常。
2. 对异常序列分别查看 `poses/estimated_c2w.csv` 和 `metrics/ate_aligned_trajectory.csv`，确认估计轨迹是否整体漂移、方向错误、尺度异常，还是 GT 对齐逻辑存在特殊情况。
3. 当前渲染指标和输出文件可以作为保存口径验证结果使用；轨迹指标在异常序列复查前不建议直接用于论文或正式对比。
