# dart2026 Scripts

## 快速开始

```bash
cd scripts
sudo ./install_service.sh
```

手动运行：

```bash
./watchdog.sh --match   # 比赛模式，强制内录
./watchdog.sh           # 调试模式，按 config/debugger.toml 控制录制
./cleanup.sh            # 停止服务、screen、watchdog 和进程
```

## 比赛模式

`watchdog.sh --match` 会把 `--match` 传给 `build/dart2026`。主程序会设置全局 `match_mode = true`，录制节点会忽略 `Recorder.enable_recording = false`，强制录制：

- `debug_*.mkv`
- `imu_*.csv`

开机自启的 `dart2026.service` 默认使用：

```ini
ExecStart=/home/dart/dart2026/scripts/watchdog.sh --match
```

需要临时改成调试模式时，去掉 `--match` 后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl restart dart2026
```

## 常用命令

```bash
sudo systemctl start dart2026
sudo systemctl stop dart2026
sudo systemctl restart dart2026
sudo systemctl status dart2026
journalctl -u dart2026 -f

screen -r dart2026
screen -d dart2026
```

## 日志

每次启动会创建一个会话目录：

```text
log/
├── 2026-04-29_13-00-00_match/
│   ├── run.log
│   ├── screen.log
│   ├── watchdog.log
│   ├── heartbeat
│   ├── resources.csv
│   ├── config/
│   ├── debug_*.mkv
│   └── imu_*.csv
```
