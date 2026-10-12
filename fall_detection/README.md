#  此处用户预留缓冲地（不修改）
uv run .\visualize_datadealer.py --data-dir ..\SDK\Software_Kit\05_Data --stem dunxia2026-07-06-17-35-32 --track-gate 1.0 --hold-ms  5000 --switch-confirm-frames 3 --cluster-radius 0.45 --min-cluster-points 3 --max-cluster-width 0.5 --max-cluster-depth 0.5

uv run .\live_visualizer.py --source replay --data-dir ..\SDK\Software_Kit\05_Data --stem dun2026-07-07-17-43-12 --interval-ms 50 --poll-ms 5 --draw-3d-every 1 --draw-timeline-every 10

# MRS6240 跌倒检测离线验证模块

本目录用于把 `MRS6240-P2512/datadealer/data` 或 `MRS6240-P2512/SDK/Software_Kit/05_Data` 里的点云 CSV 先离线跑通，再把同一套算法接回实时上位机。

坐标约定已经按当前项目确认：

- `z_m`：现实高度。
- `y_m`：雷达正前方距离。
- `x_m`：雷达左侧为正方向。

## 数据格式

`datadealer/data` 里的点云 CSV 没有表头。每行结构是：

```text
时间字符串, 毫秒时间戳, x数组..., y数组..., z数组..., velocity数组..., snr数组...
```

点数：

```text
point_count = (列数 - 2) / 5
```

当前优先使用：

- `*_motion_point_cloud.csv`：用于捕捉跌倒瞬间运动、速度突变。
- `*_presence_point_cloud.csv`：用于捕捉人体存在、跌倒后低姿态和静止状态。

`SDK/Software_Kit/05_Data` 里的点云 CSV 是表格格式，支持两类文件：

- `*_Point_cloud_conversion.csv`：运动点云，对应 `motion`。
- `*_Point_cloud_MPC_conversion.csv`：MPC 存在点云，对应 `presence`。

表头格式示例：

```text
FrameIndex,PointIndex,X,Y,Z,Velocity,SNR
```

也支持极坐标表头：

```text
FrameIndex,PointIndex,Range,Azimuth,Elevation,Velocity,SNR
```

其中 `FrameIndex` 是雷达发帧序号，`PointIndex` 是一帧内的点序号。笛卡尔坐标 `X/Y/Z` 单位是 `m`；极坐标 `Range` 单位是 `m`，`Azimuth/Elevation` 单位是度；速度单位 `m/s`，SNR 单位 `dB`。如果某帧没有点，该帧不会出现在文件中。

注意：`05_Data` 点云 CSV 只有 `FrameIndex`，没有真实毫秒时间戳。当前加载层为了兼容现有 `CombinedFrame.timestamp_ms` 字段，会暂时把 `FrameIndex` 填入 `timestamp_ms`。这对逐帧可视化和聚类跟踪没有影响，但 `fall_detector` 里按毫秒计算的确认窗口在 SDK 离线数据上只是近似；后续如果要严谨评估跌倒时间，需要补实际帧周期参数。

`rawData.bin` 暂时不作为第一版跌倒检测入口。它可以后续用来复核 HIF/PSIC 原始帧。

## 运行

在 `MRS6240-P2512` 目录运行：

```powershell
python -m fall_detection.analyze_datadealer --data-dir datadealer\data
```

指定采集文件前缀：

```powershell
python -m fall_detection.analyze_datadealer --data-dir datadealer\data --stem 2026_07_03_111005
```

## 点云聚类可视化界面

在 `MRS6240-P2512` 目录运行：

```powershell
fall_detection\.venv\Scripts\python.exe -m fall_detection.visualize_datadealer --data-dir datadealer\data
```

指定采集文件前缀：

```powershell
fall_detection\.venv\Scripts\python.exe -m fall_detection.visualize_datadealer --data-dir datadealer\data --stem 2026_07_03_111005
```

界面包含两个视图：

- `XY 2D 俯视图`：横轴是 `x_m`，雷达左侧为正；纵轴是 `y_m`，雷达正前方距离。
- `3D 坐标轴视图`：Matplotlib 3D 坐标系，可用鼠标左键拖动旋转，用来观察人体高度变化和点云空间形态。

如果当前目录在 `fall_detection` 内，也可以直接运行：

```powershell
uv run .\visualize_datadealer.py --data-dir ..\datadealer\data
```

读取 SDK `05_Data`：

```powershell
uv run .\visualize_datadealer.py --data-dir ..\SDK\Software_Kit\05_Data --stem 12026-07-04-09-50-04
```

### 安装高度与地面标定

向下俯视安装时，离线可视化按以下顺序把雷达坐标转换为近似世界坐标：

```text
YZ 俯仰旋转 -> 加安装高度 -> 拟合地面 z=ax+by+c -> 地面归一到 Z=0
```

例如目录名为 `45`、实际向下 `55` 度、雷达离地约 `1.1m`，使用前 200 帧标定地面：

```powershell
uv run .\visualize_datadealer.py --data-dir ..\fall_capture_station\data --stem 45 --yz-rotation-deg 55 --mount-height-m 1.1 --ground-calibration-frames 200 --ground-band-m 0.25 --min-snr 10 --track-gate 1.0 --hold-ms 5000 --switch-confirm-frames 3 --cluster-radius 0.45 --min-cluster-points 3 --max-cluster-width 0.5 --max-cluster-depth 0.5
```

- `--mount-height-m`：雷达安装点到地面的高度。
- `--z-min` / `--z-max`：世界坐标高度 ROI；点云先完成旋转和高度平移，再按该范围过滤。
- `--ground-calibration-frames`：使用开头多少帧拟合地面；`0` 表示关闭地面标定。
- `--ground-band-m`：以安装高度换算出的预期 `Z=0` 为中心，只取该范围内的静态存在点参与拟合。

界面顶部也可以直接修改“俯角”“安装高度”“Z 下限/上限”“标定帧数”“地面带”，点击“应用/标定”重新加载。右侧状态会显示地面内点数量、RMSE 和坡度；XZ/YZ 图中的绿色虚线是标定后的 `Z=0` 地面。

标定时前 N 帧应尽量包含可见地面，并减少人员遮挡。如果提示候选点不足，先检查安装高度和俯角，再适当扩大地面带；不要直接把地面带调得很大，否则可能把人体或墙面拟合成地面。

### 1.67m / 向下 60 度实测配置

`1.67-60` 和 `1.67-60-2` 开头可用于地面拟合的点过少，因此这两组记录必须关闭自动地面标定。安装高度平移后直接使用 `Z=0..2.2m` ROI。

在 `MRS6240-P2512/fall_detection` 目录运行：

```powershell
uv run .\visualize_datadealer.py `
  --data-dir ..\fall_capture_station\data `
  --stem 1.67-60 `
  --yz-rotation-deg 60 `
  --mount-height-m 1.67 `
  --z-min 0 `
  --z-max 2.2 `
  --ground-calibration-frames 0 `
  --min-snr 10 `
  --track-gate 1.0 `
  --hold-ms 5000 `
  --switch-confirm-frames 3 `
  --cluster-radius 0.45 `
  --min-cluster-points 3 `
  --max-cluster-width 0.5 `
  --max-cluster-depth 0.5
```

回放第二组数据时只需把 `--stem` 改为 `1.67-60-2`。

当前默认规则的回放结果：

| 记录 | 疑似跌倒帧 | 确认跌倒帧 | 持续站立后复位帧 |
| --- | --- | --- | --- |
| `1.67-60` | 651、2107 | 678、2185 | 1207、2640 |
| `1.67-60-2` | 1083、1922 | 1109、1963 | 1429、2145 |
| `163-30` | 791、1993、2484、3136、3544 | 815、2018、2509、3161、3568 | 1067、2249、2665、3319、3689 |
| `1.63-30-2` | 499、869、1331 | 523、899、1383 | 786、1149、1550 |

`1.67-60` 中标注的两次蹲下、`163-30` 中标注的四次蹲下，以及 `1.63-30-2` 中标注的五次蹲下、两次弯腰和三次坐下都没有产生疑似或确认事件。以上帧号是回放回归基线，不是跨人员、跨房间的准确率结论。

`163-30-7-15` 和 `163-30-7-15-2` 使用固定的 `1.63m / 30°` 部署参数单独回归。回放按固件顺序先放运动点，再放存在点，并在世界坐标 ROI 和 SNR 过滤后截断到 `FALL_MAX_POINTS=192`。部署配置使用世界坐标 `Z=0..2.2m`、`min_snr=15`、疑似终态持续 `700ms`、终态低位比例 `92%`，并要求候选期间相对跌落前站立锚点至少产生 `0.60m` 的 XY 位移后才允许红色确认。正样本的 4 次跌倒分别在 708、840、1029、1171 帧确认；负样本包含 6 次持物或普通蹲下和 1 次坐下，只在 288、561 帧产生蓝色疑似，没有红色确认。

当前部署数据中，4 次真实跌倒的候选最大 XY 位移最低为 `0.70m`，持凳子或纸板下蹲误报候选最高为 `0.51m`，因此暂取 `0.60m`。该门限只经过当前安装位置和测试人员数据验证，不代表所有跌倒都必然有 `0.60m` 水平位移；垂直瘫倒、原地滑落等样本补齐后必须重新评估。通用 Python 默认值仍保留 `500ms / 75% / 0m`，因为旧的 `60°` 回归数据存在水平位移较小的真实跌倒，不应把当前部署门限直接推广到其他安装条件。

回放当前部署数据时使用：

```powershell
uv run .\visualize_datadealer.py `
  --data-dir ..\fall_capture_station\data `
  --stem 163-30-7-15-2 `
  --yz-rotation-deg 30 `
  --mount-height-m 1.63 `
  --z-min 0 `
  --z-max 2.2 `
  --min-snr 15 `
  --track-gate 0.5 `
  --hold-ms 3000 `
  --switch-confirm-frames 5 `
  --fall-suspect-duration-ms 700 `
  --fall-suspect-low-ratio 0.92 `
  --fall-confirm-min-horizontal-displacement 0.60
```

`Falldown/src/prj_config.h` 中的部署参数固定为 `Z=0..220cm`、疑似低位比例 `92%` 和确认水平位移 `60cm`。确认低位持续时间现在从蓝色疑似事件产生之后重新计算，不再把疑似前的低位历史计入 `1200ms`。新固件启动时会打印 `FALL_CFG alg=3 ... roi_z=0..220 terminal=40 suspect_ms=700 suspect_ratio=92 confirm_ms=1200 confirm_xy=60`，可用来确认烧录镜像确实包含本次规则。

界面会自动读取点云文件同目录的 `operator_marks.csv`：重复的 `(radar_frame_index, mark)` 会去重，越出当前回放范围的标注会忽略。时间线使用红色表示跌倒、橙色表示蹲下、绿色表示站起，当前帧标注同时显示在底部状态区。

## 实时可视化界面

`live_visualizer.py` 使用和 `visualize_datadealer.py` 相同的 XY、3D、聚类框、目标跟踪和时间曲线，但输入改成实时流：

```text
LiveSource
↓
LatestFrameBuffer 保存全部待处理帧
↓
LiveFrameWindow 按帧顺序完成世界坐标变换、聚类、跟踪和告警
↓
GUI 每轮只绘制最后一个处理结果
```

因此界面短暂变慢时，积压帧仍会依次进入跌倒状态机，不会因为“只画最新帧”而跳过中间的下降过程。`发布`是数据源收到的帧数，`使用`是已经进入算法的帧数；正常运行一段时间后两者应基本追平。

默认模式是 CSV 实时回放，不连接设备、不发送串口/SPI 命令：

```powershell
uv run .\live_visualizer.py --source replay --data-dir ..\datadealer\data
```

读取 SDK `05_Data` 并按 20Hz 左右回放：

```powershell
uv run .\live_visualizer.py --source replay --data-dir ..\SDK\Software_Kit\05_Data --stem dunxia2026-07-06-17-35-32 --interval-ms 50 --poll-ms 5 --draw-3d-every 1 --draw-timeline-every 10
```

显式连接开发板时使用设备模式。该模式通过 SDK 自带 `HifMsgDataCollectionLib.dll` 接收 `dataType=4` 的 point_cloud 回调，转换为本项目的 `CombinedFrame` 后进入同一套可视化和跟踪链路：

```powershell
uv run .\live_visualizer.py --source device --device-transport spi --spi-index 0 --spi-freq-mode 0
```

当前 `1.67m / 向下 60°` 安装使用下面的设备命令：

```powershell
uv run .\live_visualizer.py `
  --source device `
  --device-transport spi `
  --spi-index 0 `
  --spi-freq-mode 0 `
  --yz-rotation-deg 60 `
  --mount-height-m 1.67 `
  --z-min 0 `
  --z-max 2.2 `
  --min-snr 10 `
  --track-gate 1.0 `
  --hold-ms 5000 `
  --switch-confirm-frames 3 `
  --cluster-radius 0.45 `
  --min-cluster-points 3 `
  --max-cluster-width 0.5 `
  --max-cluster-depth 0.5
```

实时链路会先执行 `YZ 旋转 -> 加安装高度`，之后再使用 `Z=0..2.2m` ROI 和世界高度跌倒阈值。当前四组记录已通过 `LiveFrameWindow` 回归，事件帧与离线可视化完全一致。

当前 `1.63m / 向下 30°` 安装使用：

```powershell
uv run .\live_visualizer.py `
  --source device `
  --device-transport spi `
  --spi-index 0 `
  --spi-freq-mode 0 `
  --yz-rotation-deg 30 `
  --mount-height-m 1.63 `
  --z-min 0 `
  --z-max 2.2 `
  --min-snr 10 `
  --track-gate 1.0 `
  --hold-ms 5000 `
  --switch-confirm-frames 3 `
  --cluster-radius 0.45 `
  --min-cluster-points 3 `
  --max-cluster-width 0.5 `
  --max-cluster-depth 0.5 `
  --fall-terminal-height 0.40
```

`--fall-terminal-height` 是世界坐标中的人体目标中心终态高度，不是雷达原始 Z。安装高度或俯角配置错误会直接改变该值。

近距离验证时，如果人或静止存在点离雷达很近，可以显式放宽前向 ROI 和 SNR 门限。例如 `DataCollection_demo.py` 实测出现过 `presence_point_cloud` 首点 `y=0.090m`，默认 `--y-min 0.15` 会把这类点过滤掉：

```powershell
uv run .\live_visualizer.py --source device --device-transport spi --spi-index 0 --spi-freq-mode 0 --y-min 0.05 --min-snr 10
```

UART 方式：

```powershell
uv run .\live_visualizer.py --source device --device-transport uart --uart-com COM12 --uart-baud 1000000
```

设备模式当前只执行：

```text
OpenSpiDevice/OpenUartDevice
StartCollectingData
ConvertPointCloudData
StopCollectingData
CloseSpiDevice/CloseUartDevice
```

不会自动 `SendConfigFile`，也不会自动 `StopRadar` / `StartRadar`。如果固件没有主动上报点云，界面只会显示已连接但没有点；后续再按实际固件流程决定是否加入“安全的一键配置上报”。

已验证的 SDK `DataCollection_demo.py` 点云回调格式：

- `dataType=4` 表示 `point_cloud`。
- `timeStamp` 可能是 `2026-07-07-12:27:26.769` 这种字符串，不一定是纯数字毫秒。
- `signalName=motion_point_cloud` 会映射为本项目的 `motion` 点。
- `signalName=presence_point_cloud` 会映射为本项目的 `presence` 点。
- `x/y/z` 是坐标，`w` 是速度，`u` 是 SNR。

命令行分析 SDK `05_Data`：

```powershell
uv run .\analyze_datadealer.py --data-dir ..\SDK\Software_Kit\05_Data --stem 12026-07-04-09-50-04
```

颜色含义：

- 灰色：通过 ROI/SNR 过滤后的背景点或小聚类点。
- 蓝色：presence 点聚类。
- 橙色：motion 点。
- 黄色框：当前被算法选中的人体目标聚类。
- 红色十字：人体目标中心点。

人体目标聚类约束：

- 当前算法把人体候选的俯视尺寸限制为 `0.5m x 0.5m`。
- 这个限制发生在聚类阶段，不是只把 2D 显示框裁剪成 0.5m。
- 如果多个点在 XY 俯视下跨得过宽或过深，即使距离阈值能把它们连起来，也不会被合并成同一个人体目标。

目标锁定和静止保持：

- 一旦锁定人体目标，后续优先只在上一次目标中心附近 `0.5m` 的 XY 范围内继续跟踪。
- 锁定状态下，DBSCAN 只负责给出候选簇；目标中心只使用上一目标 `0.5m x 0.5m` 核心区域内的点更新。
- 如果候选簇在上一目标核心区域内没有任何点，就不允许用整个候选簇更新中心，直接进入保持，避免边缘弱点把框逐帧拉走。
- `motion` 点可以推动目标的 XY 中心更新；纯 `presence` 静止点只能维持旧 XY 中心，并刷新 Z 高度、姿态和存在状态。
- 这样人静止时还能保留高度/低姿态信息，但静止杂点不会像运动目标一样快速抢走或拖走框。
- 如果人停下导致当前帧附近点很弱，跟踪器会保持上一次目标位置，默认保持 `3000ms`。
- 保持窗口内，远处新出现的 motion 点不会计入切换确认，也不会抢走人体目标。
- 保持窗口结束后，远处候选必须连续 `5` 帧都像同一个目标，才允许切换。
- 保持状态下，2D/3D 界面仍显示上一次目标框，标记为保持，而不是让目标框消失或跳到其他聚类。
- 如果没有近处匹配，也没有通过连续帧确认的新目标，保持超时后会进入“无目标”状态，而不是立即跳到远处噪点。
- 门限只看 XY 平面距离，不用 Z 高度距离；这样跌倒时高度快速下降不会被误判为目标跳远。

`visualize_datadealer.py` 只读取离线 CSV；`live_visualizer.py --source device` 会连接设备采集接口，但仍不会烧录固件。实时状态上位机连接控制串口后会发送只读的 `fall_config` 查询当前部署配置，不会修改雷达参数或启动状态。

## 板端日志上位机

实时串口控制台支持设置板端安装高度。输入厘米值（`100~500cm`）后点击“设置高度”，上位机会发送 `fall_height <cm>`；雷达停止运行时才接受修改，固件会校验并保存到 KVF Flash，重启后自动加载。连接后上位机会自动发送 `fall_config`，也可点击“读取配置”手动查询运行中的 `FALL_CFG`；因此自动启动模式下不必依赖启动日志来核验高度、俯角和 ROI。

`serial_event_live_visualizer.py` 是一个不在主机重新计算跌倒的显示程序：点云只用于绘制
XY、XZ、YZ 和原始时间线，跌倒告警以开发板串口输出的 `FALL_EVENT` 为准。设备模式下，
点云走 SPI/HIF，日志走控制串口：

```powershell
uv run .\serial_event_live_visualizer.py --source device --spi-index 0 --spi-freq-mode 0 --serial-port COM3 --serial-baud 115200
```

采集结束后，可以直接回放 `fall_capture_station` 生成的完整 session，不会打开 SPI 或 COM：

```powershell
uv run .\serial_event_live_visualizer.py `
  --source replay `
  --session-dir ..\fall_capture_station\data\1.67-60 `
  --replay-speed 1.0
```

回放加速或循环：

```powershell
uv run .\serial_event_live_visualizer.py `
  --source replay `
  --session-dir ..\fall_capture_station\data\163-30 `
  --replay-speed 4 `
  --replay-loop
```

回放层会读取：

- `<session>_motion_point_cloud.csv` 和 `<session>_presence_point_cloud.csv`：原始运动点、存在点；
- `serial_control.log`：按日志时间戳回放 `收←`、`发→`、`FALL` 和 `FALL_EVENT`；
- `session.json`：会话开始时间，用于把串口墙上时间转换成相对时间。

点云和串口日志由同一个回放时钟投递，因此时间线和告警事件保持相对同步。没有
`serial_control.log` 的旧 session 仍可显示点云，但不能复现板端告警。回放模式下串口页
只读，不会向开发板发送命令。

采集流程仍然是先运行 `fall_capture_station\capture_app.py`，停止记录后再打开回放；不要让
两个程序同时打开同一个 SPI 数据口。

可调参数：

```powershell
fall_detection\.venv\Scripts\python.exe -m fall_detection.visualize_datadealer --data-dir datadealer\data --min-snr 18 --cluster-radius 0.5 --min-cluster-points 4 --max-cluster-width 0.5 --max-cluster-depth 0.5 --track-gate 0.5 --hold-ms 3000 --switch-confirm-frames 5
```

## 当前算法链路

```text
datadealer CSV
↓
逐帧还原 motion / presence 点云
↓
按 timestamp_ms 合并两路点云
↓
ROI + SNR 过滤
↓
带人体俯视尺寸约束的半径聚类
↓
上一帧中心门控、静止保持、远处新目标连续帧确认
↓
高度/速度/运动点特征
↓
规则跌倒检测
```

当前默认规则按世界坐标高度工作：

```text
内部下降候选（不显示红色弹窗）：
    3.5 秒窗口内目标中心高度下降至少 0.80m
    且速度达到 0.45m/s 或运动点达到 20 个
    且最近 500ms 内至少 75% 的有效帧低于 0.55m

疑似跌倒：
    内部候选仍有效
    且完整 500ms 终态窗口内至少 75% 的有效帧低于 0.40m
    且窗口内至少有 3 个新鲜高度样本

确认跌倒：
    内部候选后的 7 秒确认窗口内，从疑似事件产生后重新覆盖完整 1200ms 终态低姿态
    且对应窗口至少 70% 的有效帧低于 0.40m
    且窗口内至少有 6 个新鲜高度样本
    当前 1.63m / 30° 固件部署还要求候选最大 XY 位移至少 0.60m
    跌倒后仍在活动不会直接否定跌倒，只改变事件原因描述

重新布防：
    确认后目标中心必须连续 1000ms 高于 0.85m
    单纯等待超时不会自动重新布防
```

新鲜高度样本要求当前人体目标至少有 3 个点。纯 `HOLD` 帧的 `point_count=0`，只维持目标框和身份，不再重复累计上一帧低高度。`STATIC_LOCK` 虽然冻结 XY，但包含当前静态点并刷新 Z，只要点数达到门限，仍然可以参与终态判断。

## 文件职责

- `datadealer_loader.py`：读取 datadealer CSV，生成逐帧点云。
- `tracking.py`：ROI 过滤、聚类、人体目标跟踪。
- `rule_detector.py`：第一版规则跌倒检测。
- `analyze_datadealer.py`：命令行摘要工具。
- `recording_replay.py`：读取采集 session，并同步投递点云与串口日志。
- `serial_event_live_visualizer.py`：设备实时或录制回放的原始点云、板端日志显示。
- `tests/`：离线解析、跟踪和规则检测测试。

## 当前限制

当前已用 `fall_capture_station/data/1.67-60`、`1.67-60-2`、`163-30`、`1.63-30-2`、`163-30-7-15` 和 `163-30-7-15-2` 做回放回归，合计覆盖 16 次受控跌倒、25 次蹲下、4 次弯腰和 6 次坐下。前四组使用通用 Python 默认规则，后两组使用上述固定安装部署配置。它只能证明当前代码在这些采集上可重复，不是跨人员、跨房间的准确率结论，也不能直接作为安全产品验收结果。

后续需要补标签数据：

```text
empty
standing
walking
sitting
sit_down
stand_up
bend
squat
lie_down
fall_forward
fall_backward
fall_side
post_fall_still
```

尤其要采集 `sit_down`、`lie_down`、`bend`、`squat` 这些负样本，否则规则很容易把坐下/躺下误判为跌倒。
