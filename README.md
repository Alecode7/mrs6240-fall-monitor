# MRS6240-P2512 跌倒检测项目

这是一个基于 POSSUMIC MRS6240-P2512 毫米波雷达的跌倒检测工程。项目包含三部分：

- **板端固件**：接收运动点云与存在点云，在雷达板上完成单目标跟踪和跌倒状态机判断。
- **实时跌倒状态上位机**：读取板端串口状态/事件，读取 SPI/HIF 点云做可视化、健康检查、日志与事件留证；不在 PC 端重新判定是否跌倒。
- **采集、标注、回放工具**：用于同步采集相机、点云与串口日志，人工标注并离线回归算法。

> 本项目当前的算法阈值和测试结果仅针对已有安装方式、房间环境和受试动作验证，不等同于医疗器械认证或对所有人员、场地的安全保证。二次部署前必须重新做场地标定和回归测试。

## 公开仓库范围

本仓库只公开本项目自行维护的应用层代码，便于复现、二次开发和审阅。为避免未经授权分发供应商内容、个人测试数据或可直接烧录的固件，以下内容**不会**上传：

- POSSUMIC 原厂 SDK、CDK、烧录工具、CH347/HIF DLL、驱动与硬件文档；
- 固件构建产物和历史 `.img` 镜像；
- 点云采集、相机视频、标注、运行日志、报警留证和个人测试数据；
- Python 虚拟环境、缓存和第三方二进制库。

板端仅保留 Falldown 应用层的 `src/` 源码与 MRS6240-P2512 工程描述，路径仍与厂商 SDK 对应。要编译固件，请从合法渠道取得兼容版本的供应商 SDK/CDK，将本仓库中的 Falldown 目录覆盖到 SDK 的同名目录后，在 CDK App 中打开工程。仓库不提供可直接烧录的 `.img` 文件。

实时上位机在连接真实 HIF/SPI 数据口时同样需要用户自行从供应商 SDK 安装对应 DLL 及驱动；仅运行单元测试或解析串口文本时不需要硬件。

## 1. 项目边界与总体架构

```text
                    控制口 UART (COM3, 115200)
雷达板 ────────────────────────────────────────────────> 实时上位机
  │                                                            │
  │ 板端：坐标转换、ROI、聚类、单人跟踪、跌倒状态机              ├─ 状态/报警界面
  │                                                            ├─ 运行日志
  │                                                            └─ 事件留证 JSON/PNG
  │
  └──────────── SPI/HIF（CH347，默认 index=0）──────────────> 原始点云可视化

采集上位机 ── 同步保存 相机 + 点云 + 串口 ──> 标注上位机 / 离线回放 / 回归测试
```

核心原则：**“是否跌倒”以板端串口输出的 `FALL_EVENT` 为准**。PC 端显示的点云、目标框和折线图用于观测、排障、留证，不会覆盖板端判断结果。

## 2. 当前运行基线

| 项目 | 当前约定 |
| --- | --- |
| 雷达型号 | MRS6240-P2512 |
| 控制串口 | `COM3`，`115200` baud |
| 点云数据口 | CH347 SPI/HIF，默认 `SPI index=0`、`freqMode=0` |
| 实时上位机 | `fall_detection/serial_event_live_visualizer.py` |
| 启动方式 | 根目录 `启动实时跌倒状态上位机.ps1` |
| 上位机连接方式 | 实时上位机“一键直连”默认不主动发送 `mmwc start`；是否已开始测量必须以持续的 `FALL frame=...` 状态帧为准 |
| 安装高度 | 支持 `fall_height <cm>`，范围 `100~500cm`，保存到板端 Flash；当前工程默认值为 `310cm` |

历史测试已使用过不同的安装高度、俯角和 ROI。**不要把本表或代码中的历史数值直接当作新场地配置。**每次烧录、上电或改安装配置后，都应查看启动日志中的以下信息：

```text
FALL_HEIGHT loaded=<实际高度cm>
FALL_CFG alg=3 mount_h=... pitch=... roi_x=... roi_y=... roi_z=...
```

`FALL_CFG` 是本次运行镜像的编译配置，`FALL_HEIGHT` 是 Flash 中加载的可配置高度。二者都应与实际安装一致后再测试。固件自动启动后若错过了启动日志，可在串口发送 `fall_config` 主动读取当前 `FALL_CFG`，不需要重启雷达。

已部署固件的运行日志已验证：发送 `fall_height 310` 后返回 `FALL_HEIGHT set=310cm saved=1`，无参数查询返回 `FALL_HEIGHT 310cm`。当前本地 `SDK` 内的 Falldown 工程已包含这套命令和 Flash 持久化实现；公开仓库只保留其应用层源码和工程描述，开发时必须将其置于兼容的供应商 SDK 中，而不是使用外部历史 CDK 副本。

当前工程的编译基线为：默认高度 `310cm`、俯角 `90°`、ROI `x=-250..250cm` / `y=-250..250cm`、确认阶段最小水平位移 `30cm`。运行期通过 `fall_height` 修改高度后，其他参数不会随之自动改变。

## 3. 目录说明

```text
.
├─ fall_detection/                 实时状态上位机、离线分析、回放和测试
│  ├─ serial_event_live_visualizer.py  实时跌倒状态上位机主程序
│  ├─ serial_event.py                   FALL / FALL_EVENT 协议解析
│  ├─ device_source.py                  SDK HIF 数据源适配
│  ├─ tracking.py                       PC 端点云聚类与跟踪（用于显示/离线分析）
│  ├─ rule_detector.py                  PC 离线规则回放，不是实时告警依据
│  ├─ recording_replay.py               采集 session 回放
│  ├─ visualize_datadealer.py           离线点云可视化
│  ├─ analyze_datadealer.py             离线数据摘要分析
│  └─ tests/                            上位机单元测试
├─ fall_capture_station/            采集和人工标注上位机
│  ├─ capture_app.py                    相机/点云/串口同步采集
│  ├─ annotation_app.py                 回放和人工标注
│  └─ data/<session>/                   每次采集会话输出目录
├─ datadealer/                      本地历史点云数据及处理数据（不公开）
├─ logs/                            本地实时运行日志和事件留证（不公开）
│  └─ realtime_fall/<时间戳>/evidence/ 事件 JSON 与时间线 PNG
├─ SDK/                             本地供应商 SDK；公开仓库仅保留 Falldown 应用层源码
├─ MRS6240文档类/                   本地原始硬件/协议文档（不公开）
├─ 启动实时跌倒状态上位机.ps1
├─ 启动采集上位机.ps1
└─ 启动标注上位机.ps1
```

本地开发环境中，固件 CDK 工程位于：

```text
D:\MRS6240-P2512\MRS6240-P2512\SDK\Software_Kit\02_SDK\psdf_sdk\project\mmwave\mmwave_application\Falldown
```

`D:\MRS6240_P2512_CDK\...\Falldown` 是较早的外部副本，仍包含 `163cm / 30°` 等历史配置，不能作为当前版本的开发或烧录来源。

板端核心文件：

```text
Falldown.cdkws                         CDK 工作区
cdk_6240_cpuf\project.cdkws            MRS6240 P2512 CPUF 子工程
cdk_6240_cpuf\project.cdkproj          构建配置
src\prj_config.h                       固定配置与默认阈值
src\fall_pipeline.c/.h                 点云转换、ROI、聚类和目标跟踪
src\fall_rule.c/.h                     跌倒状态机
src\mmw_app_entry.c                    雷达帧回调、串口协议和应用入口
cdk_6240_cpuf\output\mrs6240_p2512.img 当前工程生成的烧录镜像
```

## 4. 硬件连接与资源占用

### 4.1 两条独立通道

| 通道 | 作用 | 上位机用途 |
| --- | --- | --- |
| 控制口 UART | 启动日志、`FALL` 状态帧、`FALL_EVENT`、配置命令 | 状态显示、告警、设置高度、保存文本日志 |
| 数据口 SPI/HIF | motion / presence 原始点云 | XY/XZ/YZ 点云图、目标关联、数据健康检查 |

串口可以单独连接并观察板端事件；SPI 点云可视化不负责替代板端告警。实时上位机使用两个通道时，串口和 SPI 都应连接。

### 4.2 排他性规则

同一时刻，**采集上位机和实时跌倒状态上位机不能同时打开同一个 SPI/HIF 数据口**。需要录制数据时先关闭实时点云连接，再打开采集上位机；需要实时监控时关闭采集程序后再连接实时上位机。

控制串口也不应被多个串口终端同时占用。

## 5. 快速开始

### 5.1 推荐启动方式

1. 给雷达板上电，等待固件初始化完成。
2. 确认 Windows 设备管理器中的控制口为 `COM3`，CH347 已正确识别。
3. 双击或在 PowerShell 执行：

```powershell
.\启动实时跌倒状态上位机.ps1
```

4. 在上位机点击“一键直连”。程序会连接 `COM3` 和 SPI/HIF，默认不主动发送 `mmwc start`。
5. 检查串口控制台是否持续出现 `FALL frame=...`，并确认点云区持续更新。若没有状态帧，说明本次镜像未开始测量，应按当前镜像的流程使用界面的“启动雷达”按钮或发送 `mmwc start`。

根目录脚本固定使用：

```text
fall_capture_station\.venv\Scripts\python.exe
fall_detection\serial_event_live_visualizer.py
```

若虚拟环境不存在，先按第 6 节准备 Python 环境，或修改脚本中的 Python 路径。

### 5.2 命令行直接启动

在项目根目录执行：

```powershell
fall_capture_station\.venv\Scripts\python.exe `
  .\fall_detection\serial_event_live_visualizer.py `
  --source device --spi-index 0 --spi-freq-mode 0 `
  --serial-port COM3 --serial-baud 115200
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--source device` | 连接真实硬件；`replay` 为回放采集 session |
| `--serial-port COM3` | 控制口及日志口 |
| `--serial-baud 115200` | 控制口波特率 |
| `--spi-index 0` | CH347 SPI 设备索引 |
| `--spi-freq-mode 0` | SPI 频率模式 |
| `--device-transport uart` | 使用 UART 点云输入时启用；默认使用 SPI |
| `--point-uart-com COM13` | UART 点云口，只有 UART 数据模式才使用 |

### 5.3 设置安装高度

已部署固件可在实时上位机的“安装高度”输入框填写厘米值并点击设置，或在控制串口发送：

```text
fall_height 310
```

命令格式为 `fall_height <cm>`，范围是 `100~500`。设置流程必须是：

```text
mmwc stop
fall_height <实际高度cm>
等待 FALL_HEIGHT set=<高度>cm saved=1
mmwc start
```

雷达运行中发送高度设置会被板端拒绝，并返回 `FALL_HEIGHT stop radar before changing`。保存成功后，固件会立即用新高度重新初始化点云处理和跌倒状态机；重启后可发送不带参数的 `fall_height` 查询当前值，并通过 `FALL_HEIGHT loaded=...` 验证 Flash 加载结果。

不要把“设置高度”理解为完整安装标定：俯角、ROI、坐标转换和算法阈值仍取决于当前烧录镜像的编译配置。当前本地 `SDK` 内工程已经保留高度持久化实现，修改或重编译时不要切换到外部的历史 CDK 副本。

### 5.4 采集、标注与回放

采集：

```powershell
.\启动采集上位机.ps1
```

标注：

```powershell
.\启动标注上位机.ps1
```

实时状态上位机回放一次已采集 session：

```powershell
fall_capture_station\.venv\Scripts\python.exe `
  .\fall_detection\serial_event_live_visualizer.py `
  --source replay `
  --session-dir .\fall_capture_station\data\<session_name> `
  --replay-speed 1.0
```

采集 session 主要文件：

```text
session.json                         本次采集配置和时间基准
camera.mp4                           相机录像
camera_frames.csv                    相机帧时间信息
<session>_motion_point_cloud.csv     运动点云
<session>_presence_point_cloud.csv   存在/静态点云
radar_events.jsonl                   雷达帧与事件记录
serial_control.log                   原始串口收发日志
operator_marks.csv                   人工标签
```

`relative_ms` 是相机与雷达对齐的主时间轴；需要追溯板端原始帧时使用 `frame_index` / `source_radar_frame_index`。

## 6. PC 环境准备

### 6.1 软件与驱动

- Windows 10/11，64 位。
- Python 3.10+（以项目已有虚拟环境的实际版本为准）。
- CH347 驱动，用于 SPI/HIF 数据口。
- 控制串口的 USB 转串口驱动（按实际硬件芯片安装，例如 CH340）；它与 CH347 用途不同，可以共存。
- SDK 中提供的 `HifMsgDataCollectionLib.dll` 及其依赖库；上位机通过 `device_source.py` 调用。
- CDK App，仅在修改/编译板端固件时需要；本项目不依赖 Keil 5。

### 6.2 Python 依赖

项目已有可用虚拟环境时优先沿用，避免随意升级 NumPy、SciPy 或 pythonnet 导致 SDK DLL 兼容性变化。主要依赖类别如下：

```text
pyserial         控制串口
pythonnet        调用 HIF .NET DLL
numpy            点云和曲线数据
matplotlib       点云/时间线绘图
opencv-python    相机采集（采集上位机）
pytest           上位机测试
```

示例（仅在需要重建环境时执行）：

```powershell
py -3 -m venv fall_capture_station\.venv
.\fall_capture_station\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install pyserial pythonnet numpy matplotlib opencv-python pytest
```

安装依赖后，应先启动实时上位机验证 DLL 与 CH347 数据口，而不是直接假定 Python 环境可用。

## 7. 板端算法与状态机

### 7.1 输入与单目标处理

板端每个雷达帧同时处理：

- `motion point cloud`：运动点，主要提供动作、速度和动态变化。
- `presence point cloud`：存在/静态点，主要用于人体静止、低姿态和跌倒后的持续存在。

数据依次经过：坐标处理、ROI/SNR 过滤、半径聚类、人体尺寸约束、单目标关联、短时保持与新目标切换确认。`FALL` 状态帧中的 `x/y/z/h` 是板端选中目标的诊断输出，不是原始点云的逐点坐标。

### 7.2 状态机

```text
NORMAL（正常）
  │ 满足下降候选 + 低姿态持续条件
  ▼
SUSPECTED（疑似跌倒）
  │ 继续满足确认窗口、有效高度样本和水平位移条件
  ▼
CONFIRMED（确认跌倒，报警保持）
  │ 目标连续站起达到恢复条件
  ▼
RECOVERED 事件 -> NORMAL（重新布防）
```

确认跌倒后不会仅因时间流逝自动消失；报警持续到板端判定目标已经恢复，或设备重启/重新初始化。

### 7.3 当前源码默认规则

以下数值来自固件 `src/prj_config.h`，是**当前该源码树的编译默认值**，不是所有历史镜像或所有安装位置都通用的产品参数：

| 阶段 | 主要条件 |
| --- | --- |
| 内部下降候选 | 在 `3.5s` 窗口内高度下降至少 `80cm`；同时速度达到 `45cm/s` 或运动点数达到 `20`；并结合低姿态历史判断 |
| 疑似跌倒 | 终态低于 `40cm`，低位持续 `700ms`，低位比例至少 `92%`，至少 `3` 个新鲜高度样本 |
| 确认跌倒 | 疑似后重新计算 `1200ms` 终态窗口，低位比例至少 `70%`，至少 `6` 个新鲜样本；当前源码另要求相对候选锚点水平位移至少 `30cm` |
| 恢复/重新布防 | 确认后目标连续 `1000ms` 高于 `85cm`，且与确认低位中心距离不超过 `80cm`，或与跌倒前锚点距离不超过 `100cm` |

“新鲜高度样本”要求当前帧目标至少有 `3` 个点。候选阶段如果跟踪进入 `HOLD`，固件最多容忍 `800ms` 的短暂丢点并维持高度连续性；超过该时长会清空新鲜样本，避免陈旧低高度在后续误确认。候选从开始起最多保留 `7s`，疑似后仍未确认会输出 `recovered` 事件，原因是 `suspected_expired`，随后回到正常状态。

相关宏按职责分布如下：

| 文件 | 二次开发时优先查看/修改的内容 |
| --- | --- |
| `src/prj_config.h` | 安装默认值、ROI、SNR、聚类、跟踪、跌倒阈值、状态帧周期 |
| `src/fall_pipeline.c` | 点云坐标、聚类、目标获取/保持/切换逻辑 |
| `src/fall_rule.c` | 候选、疑似、确认、恢复的状态转换逻辑 |
| `src/mmw_app_entry.c` | 每帧调用顺序、串口 `FALL` / `FALL_EVENT` 输出格式 |

调参原则：一次只改一个假设相关的参数组，并保留旧镜像、启动日志、测试视频/点云和结果。不要仅凭单次成功动作降低高度阈值或缩短确认时间。

## 8. 串口协议与运行诊断

### 8.1 周期状态帧

板端约每 10 帧输出一条状态：

```text
FALL frame=31850 state=0 track=1 target=1 x=-64 y=44 z=137 h=82 \
pts=34 mot=34 v=7 raw=40 clusters=3 fresh=1 low=0 overflow=0
```

主要字段：

| 字段 | 含义 |
| --- | --- |
| `frame` | 雷达帧号 |
| `state` | `0=正常`、`1=疑似`、`2=确认` |
| `track` | 跟踪状态；用于判断目标是否实时匹配、静态锁定或保持 |
| `target` | `1` 表示当前有有效人体目标，`0` 表示无目标 |
| `x/y/z` | 目标中心坐标，单位 cm |
| `h` | 目标高度特征，单位 cm |
| `pts` | 当前目标使用的点数 |
| `mot` | 目标中的运动点数 |
| `v` | 目标最大绝对速度，单位 cm/s |
| `raw` | 进入板端处理的原始点数 |
| `clusters` | 当前帧聚类数量 |
| `fresh` | 当前帧是否为可用于高度窗口的新鲜样本 |
| `low` | 当前目标是否处于终态低高度 |
| `overflow` | 板端固定容量截断累计计数；稳定运行应关注其是否持续增长 |

`overflow` 非零不必然代表本帧算法已失效，但说明处理容量可能不足或输入点异常密集。测试/部署应记录其变化趋势，而不能忽略。

`track` 的板端编码为：`0=LOST`（无目标）、`1=TRACKING`（实时跟踪）、`2=STATIC_LOCK`（静态人体锁定）、`3=HOLD`（短时保持上一目标）、`4=REACQUIRE`（等待远处候选连续出现后重新获取）。

### 8.2 事件帧

```text
FALL_EVENT state=suspected frame=... t=... drop_cm=... z=... reason=...
FALL_EVENT state=confirmed frame=... t=... start_frame=... drop_cm=... z=... reason=...
FALL_EVENT state=recovered frame=... t=... drop_cm=... z=... reason=...
```

- `suspected`：满足疑似条件，界面提示预警。
- `confirmed`：板端确认跌倒，实时上位机显示/保持报警并留证。
- `recovered`：目标连续恢复到站立高度，系统回到正常并重新布防。
- `drop_cm`：相对候选前高度锚点的下降量。
- `z`：事件时的目标中心高度特征。
- `reason`：板端对触发/恢复原因的文本描述，用于日志分析。

### 8.3 控制命令

| 命令 | 用途 | 注意事项 |
| --- | --- | --- |
| `fall_height <cm>` | 设置并保存安装高度 | 支持 `100~500cm`；必须在雷达停止时设置 |
| `fall_height` | 查询当前保存的安装高度 | 预期返回 `FALL_HEIGHT <cm>` |
| `fall_config` | 查询当前跌倒部署配置 | 运行中可安全读取；返回当前 `FALL_CFG`，包括高度、俯角和 ROI |
| `mmwc start` | 手动启动雷达 | 一键直连默认不发送；只有未持续输出 `FALL` 状态帧时，才按当前镜像的流程发送 |
| `mmwc stop` | 停止雷达 | 修改运行期配置、排障或安全断开前使用 |
| `mmwc report points ...` | 厂商诊断/点云报告配置 | 仅在明确理解固件接口时使用 |

## 9. 实时上位机行为

实时上位机 `serial_event_live_visualizer.py` 已实现：

- 串口与 SPI/HIF 一键直连，串口断开后自动重连。
- 连续 3 秒没有新的点云帧时提示数据超时，并尝试重新建立点云连接。
- 显示板端状态帧频率、`overflow`、控制口和数据口连接状态。
- 顶部“部署自检”会汇总控制串口、点云、板端 `FALL` 状态帧、`overflow` 增长和 `FALL_CFG` 配置回显，输出“可监护”“配置待核验/不一致”或“数据异常”。控制串口连接后会自动发送 `fall_config`，无需依赖一次性的启动日志。
- 显示板端目标与点云的关联偏差；关联显示只用于诊断，不修改板端跟踪。
- 根据 `FALL_EVENT` 更新正常/疑似/确认/恢复状态；确认报警在恢复事件前保持。
- 板端进入确认跌倒后，上位机默认每约 1.2 秒播放一次上扬再下滑的电子警笛；“声音告警”开关可静音，收到恢复事件或板端退出确认状态后自动停止。
- 每次疑似、确认、恢复事件立即保存一次留证，并在 5 秒后补存一次。

除研发调试界面外，项目根目录还提供 `启动简化跌倒监护界面.ps1`。该入口只显示开始/停止监护、部署自检、当前报警、声音开关、链路摘要、安装配置和最近事件，不显示点云、串口命令、阈值或调试控制。它使用相同的 `COM3`、SPI/HIF、自动配置查询、事件留证和日志逻辑；正常使用时应优先打开该界面，研发排障才使用完整实时上位机。

部署自检的判定规则：

| 结果 | 含义与处理 |
| --- | --- |
| `可监护` | 串口、点云和板端状态帧均健康，`overflow` 未增长，且收到的 `FALL_CFG` 与当前部署基线及输入高度一致。 |
| `配置待核验` | 连接后仍未收到点云、`FALL` 状态帧或 `FALL_CFG`。上位机已自动请求 `fall_config`；若仍未返回，检查当前板端是否为包含该命令的新固件。 |
| `配置不一致` | 板端回显的安装高度、俯角或 ROI 与当前部署基线不一致。停止测试，确认所烧录镜像和实际安装。 |
| `数据异常` | 控制串口/板端状态帧/点云异常，或本会话中 `overflow` 继续增长。先处理数据链路，不应继续作为监护状态使用。 |

“读取高度”按钮发送不带参数的 `fall_height`，可在雷达运行中安全查询 Flash 中的当前安装高度。“读取配置”按钮发送 `fall_config`，可在雷达运行中主动读取当前高度、俯角和 ROI；控制口连接成功时也会自动发送一次，因此自动启动模式下无需重启板子来完成部署自检。

留证位置：

```text
logs\realtime_fall\<启动时间>\
├─ mrs6240_realtime_<时间>.log
└─ evidence\
   ├─ <时间>_suspected_frame..._event.json
   ├─ <时间>_suspected_frame..._event_timeline.png
   ├─ <时间>_confirmed_frame..._event.json
   └─ ..._after_5s.json / ..._after_5s_timeline.png
```

JSON 中保存事件字段、最近点云帧和板端状态序列；PNG 为当时的时间线图。测试后优先保留该目录，而不是只截图界面。

## 10. 固件二次开发、编译与烧录

### 10.1 修改位置与范围

1. 在 CDK App 中打开以下工作区，再选择 MRS6240 P2512 对应的 `cdk_6240_cpuf` 子工程：

   ```text
   D:\MRS6240-P2512\MRS6240-P2512\SDK\Software_Kit\02_SDK\psdf_sdk\project\mmwave\mmwave_application\Falldown\Falldown.cdkws
   ```

   不要打开 `D:\MRS6240_P2512_CDK` 下同名工作区，它是旧副本。
2. 安装参数、ROI、跟踪和规则阈值优先从 `src/prj_config.h` 着手。
3. 需要修改策略而非数值时，再编辑 `fall_pipeline.c` 或 `fall_rule.c`。
4. 改动安装高度持久化、命令校验、部署配置查询或启动加载时，编辑 `src/mmw_app_entry.c` 中的 `fall_height_cmd`、`fall_config_cmd`、`fall_mount_height_save` 和 `fall_mount_height_load`。
5. 变更串口协议后同步修改 PC 端 `fall_detection/serial_event.py` 及其测试。

`src/mmw_app_entry.c` 的调用顺序是：接收点云 -> `fall_pipeline` 生成目标 -> `fall_rule` 状态机 -> 输出 `FALL_EVENT` / `FALL`。改变该顺序、单位或事件格式会直接影响上位机兼容性。

### 10.2 构建产物

CDK 编译成功后，目标烧录镜像为：

```text
D:\MRS6240-P2512\MRS6240-P2512\SDK\Software_Kit\02_SDK\psdf_sdk\project\mmwave\mmwave_application\Falldown\cdk_6240_cpuf\output\mrs6240_p2512.img
```

构建中间产物位于 `cdk_6240_cpuf\Obj\`，包括 `psdf.elf`、`merge.img` 等。烧录使用厂商支持的工具和该 `.img` 文件；烧录前必须备份现有已验证镜像并记录 SHA256、日期、对应源代码修改和安装参数。

本仓库当前已生成镜像的基线信息：

```text
文件：mrs6240_p2512.img
构建产物修改时间：2026-10-09 16:29:52
文件大小：339,776 bytes
SHA256：889B716370EDD64D5DFB27B2750FAB0876361E2D050A0387B342179D1030348C
```

该哈希只用于识别当前文件，不代表已烧录板子的绝对版本；烧录后仍必须通过启动日志和实际运行状态核验。

推荐固件迭代流程：

```text
备份已验证 img
  -> 改动一组明确假设
  -> CDK 编译
  -> 保存新 img 与 SHA256
  -> 烧录并重新上电
  -> 核验启动日志 FALL_CFG / FALL_HEIGHT
  -> 空场、正常活动、负样本、正样本回归
  -> 保存实时日志和事件留证
```

### 10.3 烧录后必做核验

1. 控制口能看到固件版本、`FALL_CFG` 和 `FALL_HEIGHT`；其安装高度、俯角与 ROI 应与实际部署一致。
2. 上位机连续收到 `FALL frame=...`，`overflow` 不持续增长。
3. SPI 点云持续刷新，且目标框与实际人员位置基本一致。
4. 至少验证一组正常动作（站立、走动、蹲下）和一组跌倒后低姿态保持、站起恢复动作。
5. 保存本次完整运行日志和 `evidence` 文件夹。

## 11. 离线分析与数据回归

`fall_detection` 内的离线工具用于验证点云、跟踪和规则，不会改动板端固件：

```powershell
# 分析 datadealer 数据
fall_capture_station\.venv\Scripts\python.exe `
  -m fall_detection.analyze_datadealer --data-dir .\datadealer\data

# 点云可视化
fall_capture_station\.venv\Scripts\python.exe `
  -m fall_detection.visualize_datadealer --data-dir .\datadealer\data
```

离线数据使用时必须确认：

- 坐标系是否已经完成俯角旋转和安装高度平移。
- 高度阈值对应的是世界坐标高度，而不是原始雷达坐标。
- SDK `05_Data` 中只有帧号时，时间窗口分析是近似值，应补充实际帧周期。
- 旧数据的安装高度、俯角、SNR、ROI 与当前部署不一致时，只能作为机制回归，不能直接作为当前准确率结论。

## 12. 测试

上位机协议和界面核心测试可在根目录执行：

```powershell
$env:PYTHONPATH = 'D:\MRS6240-P2512\MRS6240-P2512'
D:\Anaconda\anaconda\python.exe -m pytest -q `
  fall_detection\tests\test_serial_event.py `
  fall_detection\tests\test_serial_event_live_visualizer.py
```

已验证的这组上位机测试为 `32 passed`。完整测试集还可能受 PC ML 历史依赖的 NumPy/SciPy ABI 差异或历史回放数据缺失影响；遇到此类问题时应单独修复环境/数据，不要把它误判为实时板端算法失效。

建议把每次物理测试按下表记录：

| 项目 | 记录内容 |
| --- | --- |
| 固件 | 镜像文件、SHA256、编译时间、源代码提交/变更说明 |
| 安装 | 高度、俯角、雷达位置、ROI、房间与主要反射物 |
| 正常样本 | 站立、走动、蹲下、弯腰、坐下、趴姿等是否误触发 |
| 跌倒样本 | 前/后/侧向、原地/横移、躺姿/趴姿、是否疑似/确认/恢复 |
| 健康指标 | 串口中断、点云超时、`track` 中断、`overflow` 变化 |
| 证据 | 实时日志、事件 JSON/PNG、采集 session、人工标注 |

## 13. 常见排障

| 现象 | 优先检查 |
| --- | --- |
| 上位机无法打开 | 虚拟环境中的 Python 是否存在；是否有其他程序占用 COM3 或 SPI；查看启动脚本报错 |
| 只有串口、没有点云 | CH347 驱动、SPI index/freqMode、SDK DLL、采集上位机是否仍占用 SPI |
| 只有点云、没有状态/报警 | COM 号、115200 波特率、控制串口是否被终端占用、固件是否输出 `FALL` |
| 连接后没有启动日志 | 很可能先上电启动、后连接串口，启动日志已错过；观察后续 `FALL` 状态帧即可，必要时重启板子 |
| `overflow` 不为零或增长 | 查看 `raw`、`clusters`、ROI/SNR、最大点数和日志是否出现异常；不要先忽略该指标 |
| 目标锁到背景 | 检查 ROI、SNR、人体聚类尺寸、背景反射物与跟踪保持/切换配置 |
| 蹲下误报或趴姿漏报 | 保存点云和事件留证，比较下降量、低位高度、终态持续、新鲜样本、水平位移和跟踪连续性；避免只调单一高度阈值 |
| 确认后无法恢复 | 观察站起后的 `z/h`、`track` 和目标 XY 偏差；恢复逻辑依赖连续高度和关联目标，不是定时自动清除 |

## 14. 二次开发建议与限制

1. 先冻结一套已验证固件和安装配置，建立可重复回放的正负样本集，再开始调参。
2. 固件开发和构建只使用根目录 `SDK` 内的 Falldown 工程；外部 `D:\MRS6240_P2512_CDK` 是历史副本，不得作为新固件发布来源。
3. 新增多人、跨房间、宠物、床/椅/桌等场景前，应明确本项目仍是单目标设计；不要把单目标状态机直接宣传为多人可靠方案。
4. 改变安装方向、房间尺寸或高度时，应先复核坐标轴、地面参考和 ROI，再讨论阈值。
5. 需要产品化时，建议把安装高度、设备编号、运行版本、健康状态、报警确认/消警和日志上传做成独立配置/监护层；它们不应直接混入底层跌倒判定代码。
6. 联网、远程告警、独立供电和云端管理属于后续系统工程，需单独定义安全、断网恢复、隐私和数据保留策略。

## 15. 参考入口

- 实时状态上位机说明：[fall_detection/README.md](fall_detection/README.md)
- 采集与标注说明：[fall_capture_station/README.md](fall_capture_station/README.md)
- 厂商 SDK 与文档：`SDK/`、`MRS6240文档类/`
- 真实运行日志：`logs/realtime_fall/`

---

维护本文档时，请同时更新：运行基线、固件路径、串口协议、默认 COM 号、构建产物路径以及任何改变状态机语义的阈值。这样 README 才能继续作为二次开发入口，而不是一份过期的项目介绍。
