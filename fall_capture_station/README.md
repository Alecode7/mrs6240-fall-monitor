# MRS6240 fall capture station

This directory contains two separate upper-computer tools for dataset work.

## Directories

- `data/`: captured sessions. Each session is one subdirectory.
- `capture_app.py`: live acquisition station.
- `annotation_app.py`: synchronized review and manual labeling station.

## Capture station

控制口面板中的“安装高度”使用米为单位。连接控制口并停止雷达后点击“设置高度”，采集上位机会换算为厘米发送 `fall_height <cm>`；固件保存成功后，重新启动时会回显实际高度。

Run:

```powershell
uv run python ..\fall_capture_station\capture_app.py
```

The capture station can:

- open the PC camera and save `camera.mp4`;
- connect to the MRS6240 HIF data port through `HifMsgDataCollectionLib.dll`;
- save motion and presence point-cloud CSV files in the same format used by `datadealer_loader`;
- open a control serial port, manually send one command, or send the built-in radar start/stop command sequence;
- write camera/radar synchronization metadata for later annotation.

Each session is saved under:

```text
fall_capture_station/data/<session_name>/
```

Important files:

```text
session.json
camera.mp4
camera_frames.csv
<session>_motion_point_cloud.csv
<session>_presence_point_cloud.csv
radar_events.jsonl
operator_marks.csv
serial_control.log
```

`camera_frames.csv` and `radar_events.jsonl` both contain `sync_frame_index`. It is a 50 ms common timeline bucket, not the radar hardware frame number. Use `relative_ms` as the canonical camera/radar alignment clock and use `radar_events.jsonl.frame_index` when the original board frame number is required.

## V2 采集流程

开始记录前，先确认界面中的 V2 采集配置与真实物理布置一致。配置通过校验后才会创建会话目录，开始记录后这些字段会锁定，停止记录后恢复编辑。`session.json` 的 `metadata` 会保存：

| 字段 | 操作要求 |
| --- | --- |
| `mount_height_m` | 雷达安装高度，单位 m，允许 `0.5~5.0`；当前 3.1 m 部署填写 `3.10`。 |
| `pitch_deg` | 雷达俯角，单位度；当前 30° 数据必须填写 `30`。 |
| `subject_id` | 使用脱敏人员编号，例如 `P01`，不要填写姓名。 |
| `environment_id` | 使用稳定的场地编号，例如 `room-a`。 |
| `scenario` | 选择本次采集任务，例如 `normal`、`crouch`、`object_assisted`、`fall`、`mixed` 或 `blind_test`。 |
| `protocol_version` | 当前采集协议版本，默认 `fall-v2.0`。 |
| `radar_profile` | 雷达命令与 FFT 配置的稳定标识，默认 `mrs6240-50ms-r256-d16`。同一实验只使用一种配置。 |

标注工具仍只提供“无人、有人、蹲下、弯腰、坐下、跌倒”六个终态标签。“上下文”是可选附加信息，例如 `held_stool` 或 `held_board`；它写入 `operator_marks.csv` 的 `note`，不会修改主标签。相邻终态事件应至少 10 秒，避免一个 2 秒窗口同时覆盖语义相反的动作。

V2 采集配置不会修改历史会话，也不会向旧 `session.json` 补写 V2 字段。缺少 V2 metadata 的旧数据仍可回放，但不能作为新的 V2 合格会话；标注工具原有的 `operator_marks.csv` 同步引用补全逻辑仍按下文工作，这不属于 V2 元数据迁移。采集上位机只负责同步录制，不运行或输出跌倒模型结果。

## Annotation station

Run:

```powershell
uv run python ..\fall_capture_station\annotation_app.py
```

The annotation station can:

- load a captured session;
- show camera video synchronized to radar time;
- show XY, XZ and YZ point-cloud views;
- automatically play synchronized radar/camera frames at `1/2/5/10/20 FPS`;
- pause playback with the `播放/暂停` button or the space key;
- automatically pause before manual frame navigation, slider scrubbing, or writing a label;
- append manual labels to `operator_marks.csv`.

The annotation status bar shows the recording event time, original radar frame, nearest camera frame and camera/radar time delta. `operator_marks.csv` keeps the old columns for compatibility and adds explicit recording references:

| Column | Meaning |
| --- | --- |
| `wall_time` | Time when the operator clicked the label during review; this is not the recorded event time. |
| `radar_frame_index` | Zero-based replay row used by the annotation and visualization tools. |
| `radar_timestamp_ms` | Event time relative to the start of recording. |
| `capture_wall_time` | Local wall-clock time when the selected radar frame was captured. |
| `source_radar_frame_index` | Original frame number emitted by the board. |
| `source_timestamp_ms` / `source_timestamp_text` | Timestamp supplied by the data source for the original frame. |
| `camera_frame_index` / `camera_video_time_ms` | Nearest frame in `camera.mp4` and its video time. |
| `camera_time_delta_ms` | `camera_relative_ms - radar_timestamp_ms`; values near zero indicate good synchronization. |
| `camera_offset_ms` | Manual camera offset selected in the annotation UI when the mark was written. |

Loading an older session safely upgrades the CSV header and backfills these references from `session.json`, `radar_events.jsonl`, and `camera_frames.csv`. Existing labels and operator click times are preserved.

## Optional Python packages

Camera capture needs `opencv-python`.

Control serial needs `pyserial`.

The radar data port keeps using the existing SDK DLL / pythonnet path already used by `fall_detection/device_source.py`.
