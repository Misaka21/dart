# RM2026 飞镖视觉系统（dart2026）

这是一个面向 RoboMaster 飞镖场景的 C++ 视觉工程。程序会从相机读取图像，从串口或虚拟数据读取电控状态，检测画面中的绿色目标灯，计算偏航方向上的像素误差，并通过网页显示调试画面和遥测曲线。

如果你是第一次接触这个项目，建议按本文档顺序操作：先把程序编译运行起来，再改配置参数，最后再读代码。

## 目录

- [项目能做什么](#项目能做什么)
- [项目结构](#项目结构)
- [运行流程](#运行流程)
- [环境准备](#环境准备)
- [编译](#编译)
- [运行](#运行)
- [比赛模式和自启动](#比赛模式和自启动)
- [相机内参标定](#相机内参标定)
- [配置说明](#配置说明)
- [调试和日志](#调试和日志)
- [开发入门](#开发入门)
- [常见问题](#常见问题)

## 项目能做什么

当前工程主要包含下面几部分：

| 模块 | 作用 |
| --- | --- |
| `hardware` | 读取相机图像，读取或模拟串口数据，并把两者同步成一帧 `sync_frame` |
| `detector` | 对图像做绿色通道阈值分割，寻找圆形绿色灯，计算目标偏航误差 |
| `test/test_calibration.cpp` | 棋盘格相机内参标定工具，输出 `param.toml` 需要的 `camera_fx/camera_fy` |
| `plugin/param` | 读取 `config/param.toml`，支持运行时热重载部分参数 |
| `plugin/rmcv_bag` | 按配置录制 detector 可视化视频、原始视频和 IMU/串口 CSV 数据 |
| `plugin/telemetry` | 收集遥测数据，供网页曲线显示 |
| `app` | Flask 网页调试界面，默认端口 `3000` |
| `scripts` | systemd 自启动、screen 看门狗和清理脚本 |
| `umt` | 项目内部使用的发布/订阅消息系统 |

程序启动后会开启多个线程：参数线程、硬件线程、检测线程、录制线程，以及一个嵌入式 Python/Flask 网页服务。

## 项目结构

```text
.
├── CMakeLists.txt              # 顶层 CMake 配置，生成 dart2026 可执行文件
├── main.cpp                    # 程序入口，启动各个线程和网页服务
├── config/
│   ├── param.toml              # 检测相关运行时参数，支持热重载
│   ├── hardware.toml           # 相机、串口、时间同步等硬件配置
│   └── debugger.toml           # 录制、Rerun、Telemetry 配置
├── hardware/
│   ├── hardware_node.cpp       # 相机和串口同步主逻辑
│   ├── hik_cam/                # 海康相机 SDK 后端和 OpenCV fallback 后端
│   └── serial/                 # UART / USB bulk 串口通信
├── detector/
│   ├── detector.cpp            # 绿色灯检测、距离估计、偏航误差计算
│   └── detector_node.hpp       # 订阅硬件帧并发布调试图像
├── test/
│   └── test_calibration.cpp    # 相机内参标定工具
├── plugin/
│   ├── param/                  # TOML 参数读取
│   ├── rmcv_bag/               # 视频和 CSV 录制
│   ├── telemetry/              # 网页遥测曲线
│   └── debug/                  # 日志会话
├── app/
│   ├── app.py                  # Flask 服务入口
│   ├── bridge.py               # Python 侧读取 C++ 内嵌模块
│   └── templates/              # 网页模板
├── scripts/
│   ├── watchdog.sh             # screen + 心跳看门狗启动脚本
│   ├── install_service.sh      # systemd 服务安装脚本
│   ├── cleanup.sh              # 停止服务和残留进程
│   └── dart2026.service        # systemd 服务模板
└── umt/                        # 消息发布/订阅和 pybind11 导出
```

## 运行流程

理解这条数据流可以帮助你定位问题：

```text
相机 + 串口/虚拟串口
        |
        v
hardware::start_hardware_node()
发布 sync_frame
        |
        v
detector::detector_run()
检测绿灯、计算 yaw_diff、发布 Detector_Debug_Image 和 telemetry
        |
        v
Flask 网页 app
浏览器查看视频流和遥测曲线
```

默认配置里 `config/hardware.toml` 的 `Serial.use_fake_serial_data = true`，所以初次运行时不需要真的连接电控串口；但仍然需要有可打开的相机或视频采集设备。

## 环境准备

### 基础要求

- CMake `3.16` 或更高版本
- 支持 C++17 的编译器
  - macOS: Apple Clang
  - Linux: GCC `9` 或更高版本，或支持 C++17 的 Clang
- Python `3.8` 或更高版本。CMake、pybind11、Flask 和 `cv2` 必须使用同一个 Python 版本
- 一个相机设备
  - Linux + 海康 MVS SDK: 可使用海康工业相机后端
  - 其他情况: 自动使用 OpenCV `VideoCapture` fallback

### C++ 依赖

顶层 `CMakeLists.txt` 会查找这些库：

- OpenCV `4.x`
- fmt `8.0` 或更高版本
- Eigen3
- Python Development
- pybind11
- Threads
- libusb-1.0，可选；缺少时 USB bulk 串口支持会关闭，UART 仍可用

`tomlplusplus` 已经使用工程内嵌的 header-only 版本，不需要再安装系统 `libtomlplusplus-dev`。

注意：Ubuntu 20.04 源里的 `libfmt-dev` 是 `6.1.2`，太旧，编译会在 `fmt::runtime` 处失败。请安装 fmt `8.x` 或更新版本，并在 CMake 配置时通过 `CMAKE_PREFIX_PATH` 指向它。

### Python 依赖

网页调试界面需要：

- Flask
- opencv-python，也就是 Python 里的 `cv2`

`Message_cvMat` 和 `Telemetry` 不是 pip 包，它们由 C++ 程序通过 pybind11 嵌入导出。请从 `dart2026` 可执行文件启动网页，不要直接单独运行 `python app/app.py`。

如果机器上同时有多个 Python 版本，先确认选中的 Python 能导入网页依赖：

```bash
python3 - <<'PY'
import flask
import cv2
import numpy
print("python deps ok")
PY
```

### macOS 安装示例

```bash
brew install cmake ninja opencv fmt eigen pybind11 libusb
python3 -m pip install --user flask opencv-python
```

如果你使用 conda，也可以把 Python 包安装到当前 conda 环境：

```bash
python -m pip install flask opencv-python
```

### Ubuntu / Debian 安装示例

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build pkg-config
sudo apt install -y libopencv-dev libfmt-dev libeigen3-dev pybind11-dev python3-dev python3-pip
sudo apt install -y libusb-1.0-0-dev
python3 -m pip install --user flask opencv-python
```

如果系统源里的 fmt 版本低于 `8.0`，请手动安装 fmt `8.x` 或更高版本，例如安装到 `$HOME/local/fmt-8.1.1`，然后在 CMake 配置时加入：

```bash
-DCMAKE_PREFIX_PATH=$HOME/local/fmt-8.1.1
```

## 编译

建议使用独立的 `build` 目录，不要把编译产物放到源码目录里。

```bash
mkdir -p build
cd build
cmake ..
make -j8
```

编译成功后会得到：

```text
build/dart2026
```

常用 CMake 选项：

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `ENABLE_WEBVIEW` | `ON` | 启用嵌入式 Python/Flask 网页调试 |
| `USE_CCACHE` | `ON` | 如果系统安装了 ccache，则自动加速重复编译 |
| `ENABLE_RERUN` | `OFF` | 启用 Rerun 可视化记录；开启时会下载 Rerun C++ SDK |
| `RMCV_WITH_HIK_CAMERA` | 自动 | 有 `/opt/MVS/include/MvCameraControl.h` 且架构支持时启用海康 SDK 后端 |

如果机器上有多个 Python 版本，请显式指定能导入 Flask、cv2 和 numpy 的那一个，例如：

```bash
cmake .. \
  -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
  -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
  -DPython_EXECUTABLE=/usr/bin/python3.8 \
  -DPython_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
  -DPython_INCLUDE_DIR=/usr/include/python3.8
make -j8
```

如果你想强制使用普通 USB 摄像头或笔记本摄像头，可以关闭海康后端：

```bash
cmake .. -DRMCV_WITH_HIK_CAMERA=OFF
make -j8
```

如果 CMake 找不到某个库，先确认依赖已经安装；如果安装在非标准路径，可以传入对应的 `*_DIR`，例如：

```bash
cmake -S . -B build -G Ninja \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 \
  -Dpybind11_DIR=/path/to/pybind11/share/cmake/pybind11
```

## 运行

注意：当前 `main.cpp` 里给 Python 传入的脚本路径是 `../app/app.py`。因此推荐从 `build` 目录启动程序。

```bash
cd build
./dart2026
```

程序启动后，打开浏览器：

```text
http://localhost:3000
```

网页左侧是调试视频流，右侧是遥测曲线。看到视频画面和 `Telemetry` 曲线持续更新，就说明主流程已经跑通。

### 选择 OpenCV 摄像头编号

使用 fallback 摄像头后端时，默认打开设备 `0`。如果你的相机不是第 0 个设备，可以通过环境变量选择：

```bash
cd build
RMCV_CAMERA_INDEX=1 ./dart2026
```

可以依次尝试 `0`、`1`、`2`，直到画面正确。

### 第一次无电控运行

默认 `config/hardware.toml` 已经开启虚拟串口数据：

```toml
[Serial]
use_fake_serial_data = true

[Serial.fake_data]
should_detect = true
dart_number = 2
allow_fire = true
```

这表示程序会假装电控已经发送了“允许检测”的状态。初学者可以先保持这个配置，等相机和检测跑通后再接真实串口。

## 比赛模式和自启动

主程序支持 RMCV2026 同款启动参数：

```bash
cd build
./dart2026 --match
./dart2026 --log-dir ../log/manual_session
./dart2026 --help
```

`--match` 会设置全局 `match_mode = true`。录制节点会忽略 `Recorder.enable_recording = false`，强制录制 detector 可视化视频和串口/IMU CSV。

更推荐通过 `scripts/watchdog.sh` 启动，它会创建日志会话目录、启动 screen、监控心跳并在异常时重启：

```bash
./scripts/watchdog.sh --match   # 比赛模式
./scripts/watchdog.sh           # 调试模式
```

安装开机自启服务：

```bash
cd scripts
sudo ./install_service.sh
```

安装后常用命令：

```bash
sudo systemctl start dart2026
sudo systemctl stop dart2026
sudo systemctl restart dart2026
sudo systemctl status dart2026
journalctl -u dart2026 -f
screen -r dart2026
```

`scripts/dart2026.service` 默认使用 `watchdog.sh --match`，所以开机自启默认是比赛模式。需要临时停掉所有相关进程时：

```bash
./scripts/cleanup.sh
```

## 相机内参标定

`Detector.camera_fx` 和 `Detector.camera_fy` 来自相机内参标定。工程内置了 `test_calibration` 工具，可以用棋盘格采集图像并计算焦距。

先编译标定工具：

```bash
cmake --build build --target test_calibration --parallel
```

从 `build` 目录运行：

```bash
cd build
./test_calibration
```

默认标定板参数来自旧工具：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--board-width` | `11` | 棋盘格内角点列数 |
| `--board-height` | `8` | 棋盘格内角点行数 |
| `--square-size-mm` | `20.0` | 单个棋盘格边长，单位 mm |
| `--min-images` | `10` | 至少采集多少张图像后才能标定 |
| `--light-diameter-m` | `0.050` | 绿灯真实直径，单位 m |

如果你的棋盘格或绿灯尺寸不同，可以这样运行：

```bash
./test_calibration --board-width 11 --board-height 8 --square-size-mm 20.0 --light-diameter-m 0.050
```

交互按键：

| 按键 | 作用 |
| --- | --- |
| `Space` | 当前画面检测到棋盘格时采集一张图 |
| `c` | 使用已采集图像开始标定 |
| `s` | 保存已采集图像 |
| `u` | 撤销上一张采集图 |
| `q` / `Esc` | 退出 |

标定完成后，终端会输出一段可以直接复制到 `config/param.toml` 的内容：

```toml
    # 绿灯直径测距
    #@double 绿灯真实直径 (m)
    light_diameter = 0.050
    #@double 相机 x 方向焦距 (px)，需与 detector 输入图像尺寸一致
    camera_fx = 标定得到的 fx
    #@double 相机 y 方向焦距 (px)，需与 detector 输入图像尺寸一致
    camera_fy = 标定得到的 fy
```

`light_diameter` 是绿灯的实际物理直径，标定工具不会从棋盘格自动推导它；请按实物测量结果填写。`camera_fx` 和 `camera_fy` 必须和 detector 实际输入图像尺寸一致，如果后续修改了相机分辨率或裁剪方式，需要重新标定或按比例换算。

## 配置说明

配置文件都在 `config/` 目录下。修改 TOML 文件时要注意类型，文件里也写了提醒：`10.0` 不能随便改成 `10`，布尔值要写 `true` 或 `false`，字符串要加引号。

### `config/param.toml`

这是检测模块的运行时参数。程序运行时每秒读取一次，所以大多数检测参数可以边运行边修改。

常用字段：

| 参数 | 作用 |
| --- | --- |
| `Detector.threshold` | 绿色通道二值化阈值。目标不明显时可以降低，噪声太多时可以升高 |
| `Detector.min_area` / `Detector.max_area` | 轮廓面积过滤范围，用来排除太小噪声或太大误检 |
| `Detector.light_diameter` | 真实绿色灯直径，单位米，用于估算距离 |
| `Detector.camera_fx` / `Detector.camera_fy` | 相机焦距，单位像素，用于距离估计 |
| `Detector.dart_number_base` | 电控飞镖编号起始值，通常为 `1` |
| `Detector.dart_ids` | 四枚飞镖的 id，按发射顺序填写 |
| `Detector.yaw_offset_total` | 全局目标线偏移，单位像素 |
| `Detector.yaw_offset_0` 到 `yaw_offset_3` | 每枚飞镖独立目标线偏移，单位像素 |
| `Visualizer.show_window` | 是否显示 OpenCV 调试窗口 |

调阈值的基本思路：

1. 保持 `use_fake_serial_data = true`，让检测一直开启。
2. 打开网页调试画面。
3. 如果绿色灯没有被框出，降低 `Detector.threshold`。
4. 如果画面中很多绿色噪声被误检，提高 `Detector.threshold`，或调大 `min_area`。
5. 如果目标灯很大却不被识别，调大 `max_area`。

### `config/hardware.toml`

这是硬件配置，主要在程序启动时读取。改完后建议重启程序。

相机相关：

| 参数 | 作用 |
| --- | --- |
| `Camera.use_camera_sn` | 使用海康 SDK 时，是否按 SN 查找相机 |
| `Camera.camera_sn` | 海康相机 SN |
| `Camera.use_config_from_file` | 是否加载 MVS 导出的 `.mfs` 配置 |
| `Camera.config_file_path` | `.mfs` 文件名，路径基于 `config/` |
| `Camera.use_camera_config` | 是否用 `Camera.config` 覆盖相机参数 |
| `Camera.config.ExposureTime` | 曝光时间，单位微秒 |
| `Camera.config.Gain` | 增益 |
| `Camera.config.Width` | 图像宽度 |
| `Camera.config.AcquisitionFrameRate` | 采集帧率 |

串口相关：

| 参数 | 作用 |
| --- | --- |
| `Serial.protocol` | `"uart"` 或 `"usb_bulk"` |
| `Serial.use_fake_serial_data` | 是否使用虚拟串口数据。初次调试建议 `true` |
| `Serial.data_print_debug` | 是否打印串口收发调试信息 |
| `Serial.ignore_crc` | 是否忽略 CRC 校验 |
| `Serial.uart.port_name` | UART 设备路径，例如 Linux 的 `/dev/ttyUSB0` |
| `Serial.uart.baudrate` | UART 波特率 |
| `Serial.usb_bulk.*` | USB bulk 通信参数，仅在 `protocol = "usb_bulk"` 时使用 |
| `TimeSync.delta_t_us` | 相机和 IMU 数据的时间偏移，单位微秒 |

接真实 UART 串口时，常见配置是：

```toml
[Serial]
protocol = "uart"
use_fake_serial_data = false

[Serial.uart]
port_name = "/dev/ttyUSB0"
baudrate = 115200
```

### `config/debugger.toml`

这是调试、录制和遥测配置。

| 参数 | 作用 |
| --- | --- |
| `Recorder.enable_recording` | 是否启用录制 |
| `Recorder.record_raw_video` | 是否录制原始视频 |
| `Recorder.record_debug_video` | 是否录制 detector 绘制后的可视化视频 |
| `Recorder.record_imu_csv` | 是否录制 IMU/串口 CSV |
| `Recorder.camera_fps` | 录制视频使用的相机帧率 |
| `Recorder.sample_interval` | 采样间隔，`1` 表示每帧都录 |
| `--match` / `watchdog.sh --match` | 比赛模式会强制录制 debug 和 imu |
| `Telemetry.enable` | 是否启用网页遥测 |
| `Telemetry.publish_hz` | 网页曲线推送频率 |
| `Telemetry.default_time_window` | 网页默认显示时间窗口，单位秒 |
| `Rerun.enable` | 是否启用 Rerun 记录 |

录制文件会放在当前日志会话目录里，例如：

```text
log/2026-04-27_20-30-00/
├── run.log
├── config/
├── debug_001.mkv
└── imu_001.csv
```

## 调试和日志

程序每次启动都会创建一个日志会话目录：

```text
log/<启动时间>/
```

里面通常包含：

| 文件或目录 | 说明 |
| --- | --- |
| `run.log` | 程序日志 |
| `screen.log` | `scripts/watchdog.sh` 启动时的 screen 输出 |
| `watchdog.log` | 外部看门狗日志 |
| `heartbeat` | 内部 watchdog 写出的节点心跳 |
| `resources.csv` | 外部看门狗记录的进程和系统资源 |
| `config/` | 启动时的 TOML 配置快照 |
| `debug_*.mkv` | detector 绘制后的可视化视频，比赛模式强制录制 |
| `raw_*.mkv` | 原始相机视频，仅 `Recorder.record_raw_video=true` 时录制 |
| `imu_*.csv` | 录制串口/IMU 数据，取决于 `debugger.toml` |

判断是否启动成功可以看这些现象：

1. 终端打印 `Loading parameters`、`Loading Hardware`、`Loading Detector`。
2. 终端没有连续出现 `FATAL`。
3. 浏览器能打开 `http://localhost:3000`。
4. 网页中能看到调试图像。
5. 右侧遥测列表里出现 `detector/yaw_diff_px`、`detector/light_distance_m` 等曲线。

## 开发入门

### 从入口读代码

建议按这个顺序读：

1. `main.cpp`：了解程序启动了哪些线程。
2. `hardware/hardware_node.cpp`：了解相机帧和串口数据如何合成 `SyncFrame`。
3. `detector/detector_node.hpp`：了解检测线程如何订阅硬件帧。
4. `detector/detector.cpp`：了解绿灯检测和误差计算。
5. `app/app.py` 和 `app/bridge.py`：了解网页如何拿到 C++ 发布的图像和遥测。

### 新增一个检测参数

如果参数需要运行时热重载：

1. 在 `config/param.toml` 里新增字段，例如：

   ```toml
   [Detector]
   example_value = 123
   ```

2. 在 C++ 中读取：

   ```cpp
   int64_t value = plugin::get_param<int64_t>("Detector.example_value");
   ```

3. 运行程序后修改 TOML，参数线程会在约 1 秒内更新。

如果参数只需要启动时读取，参考 `config/hardware.toml` 和 `static_param::get_param`。

### 新增一个网页遥测曲线

在 C++ 里发布标量：

```cpp
telemetry::scalar("detector/my_value", value);
```

网页会自动发现新的 series。常用路径建议使用 `模块名/变量名`，例如：

```text
detector/yaw_diff_px
serial/dart_number
hardware/frame_id
```

### 新增一个调试图像

可以通过 UMT 发布 `cv::Mat`，然后在 Python 网页侧订阅。现有检测调试图像的名字是：

```text
Detector_Debug_Image
```

网页默认会取当前可用图像流列表中的第一个。

## 常见问题

### CMake 找不到 OpenCV、fmt、Eigen3、tomlplusplus 或 pybind11

先安装缺少的依赖，然后重新配置：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

如果依赖安装在非标准位置，给 CMake 传入路径：

```bash
cmake -S . -B build -G Ninja \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 \
  -Dfmt_DIR=/path/to/fmt/lib/cmake/fmt \
  -DEigen3_DIR=/path/to/eigen3/share/eigen3/cmake \
  -Dtomlplusplus_DIR=/path/to/tomlplusplus/lib/cmake/tomlplusplus \
  -Dpybind11_DIR=/path/to/pybind11/share/cmake/pybind11
```

### 运行时报 `ModuleNotFoundError: No module named 'flask'` 或 `cv2`

CMake 找到的 Python 解释器和你安装包用的 Python 可能不是同一个。先查看 CMake 使用的 Python：

```bash
cmake -S . -B build -LAH | grep Python_EXECUTABLE
```

然后用显示出来的解释器安装依赖：

```bash
/path/to/python -m pip install flask opencv-python
```

### 运行时报找不到 `Message_cvMat` 或 `Telemetry`

不要直接运行：

```bash
python app/app.py
```

这些模块是 C++ 可执行文件启动 Python 时内嵌导出的。请从 `build` 目录运行：

```bash
cd build
./dart2026
```

### 摄像头打不开

如果使用 OpenCV fallback 后端：

```bash
cd build
RMCV_CAMERA_INDEX=0 ./dart2026
RMCV_CAMERA_INDEX=1 ./dart2026
RMCV_CAMERA_INDEX=2 ./dart2026
```

如果使用海康相机：

1. 确认 MVS SDK 安装在 `/opt/MVS`。
2. 确认 `/opt/MVS/include/MvCameraControl.h` 存在。
3. 确认 `config/hardware.toml` 里的 `Camera.camera_sn` 正确。
4. 必要时重新配置：

   ```bash
   cmake -S . -B build -G Ninja -DRMCV_WITH_HIK_CAMERA=ON
   cmake --build build --parallel
   ```

### 真实串口打不开或没有权限

Linux 上先确认设备是否存在：

```bash
ls /dev/ttyUSB*
```

如果设备存在但权限不足，把当前用户加入 `dialout` 组，然后重新登录：

```bash
sudo usermod -aG dialout $USER
```

macOS 上串口路径通常类似：

```bash
ls /dev/tty.*
```

然后把 `config/hardware.toml` 里的 `Serial.uart.port_name` 改成对应路径。

### 网页能打开，但没有图像

按顺序检查：

1. 终端是否有相机打开失败的错误。
2. 是否从 `build` 目录运行了 `./dart2026`。
3. `config/hardware.toml` 中 `Serial.use_fake_serial_data` 是否为 `true`。
4. `Serial.fake_data.should_detect` 是否为 `true`。
5. 相机是否真的输出了画面。

### 网页端口 `3000` 被占用

当前端口写在 `app/app.py`：

```python
app.run(host="0.0.0.0", port=3000, threaded=True)
```

如果端口冲突，可以临时改成其他端口，例如 `3001`，然后重新运行程序。

### 修改 `param.toml` 后没有效果

`config/param.toml` 会热重载，但要注意：

1. TOML 类型不能改错，例如原来是 `1200.0` 就保持浮点数写法。
2. 字段名要和 C++ 里读取的路径一致。
3. 如果终端出现 TOML parse error，说明文件语法有问题。
4. `hardware.toml` 和 `debugger.toml` 中多数参数是启动时读取，改完需要重启。

## 推荐的第一次上手步骤

1. 安装依赖。
2. 保持 `Serial.use_fake_serial_data = true`。
3. 编译：

   ```bash
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DRMCV_WITH_HIK_CAMERA=OFF
   cmake --build build --parallel
   ```

4. 从 `build` 目录运行：

   ```bash
   cd build
   ./dart2026
   ```

5. 打开 `http://localhost:3000`。
6. 调整 `config/param.toml` 里的 `Detector.threshold`、`min_area`、`max_area`，让绿色目标灯稳定被识别。
7. 接入真实串口，把 `Serial.use_fake_serial_data` 改为 `false`，配置正确的串口路径。
8. 根据实测结果调整每枚飞镖的 `yaw_offset_*`。
