# Nano 水下服务部署记录：2026-11-12

目标：`nvidia@192.168.1.188`，工作目录 `/home/nvidia/surface_ws`。本次用户授权部署新水下控制服务、关闭旧 H7 串口桥，并要求 H7 可不随 Nano 每次开机连接。

以下为 2026-11-12 部署结束时的现场快照，PID、设备存在性和服务状态不作为持续在线保证。后续运维命令见 [Nano 部署指南](../DEPLOY_NANO.md#水下-h7独立服务与可选设备自启动)。

## 已安装状态

| 服务 | 开机启用 | 运行状态 | 本次核验 PID |
|---|---|---|---|
| `under-control.service` | enabled | active/running | 3004 |
| `nano-h7-bridge.service` | disabled | inactive/dead | 0 |
| `f1-operator.service` | enabled | active/running，未重启 | 2477 |
| `pavo2-scan.service` | enabled | active/running，未重启 | 1383 |

新服务监听 `0.0.0.0:8766`，水面仍监听 8765；旧服务的 5565/5566/5567 监听已退出。旧配置中 F103 与雷达模块均禁用，确认替换的是独立 H7 桥。

`/etc/under-control.env`：

```ini
UNDER_SERIAL_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
UNDER_BIND=0.0.0.0
UNDER_PORT=8766
UNDER_MOTION_ENABLED=true
```

设备路径取自原 H7 服务的 `config.h7-only.json`，没有猜测 ttyUSB 编号或改用水面 F103。该 H7 路径在部署期间不存在；当时可见的 `/dev/ttyACM0` 是水面适配器，仍由原 PID 2514 占用，新服务没有打开任何串口。

H7 可缺席开机：服务常驻，设备缺席时每秒检查一次，状态变更时记录日志，不重复增加控制代次或重启服务。设备接入后自动发现与停止握手；掉线时撤销操作，重新接入不恢复旧运动。unit 不使用设备 Requires/BindsTo/ConditionPathExists。

`UNDER_MOTION_ENABLED=true` 允许后续 App 人工控制，实际仍须设备连接、协议及停止握手通过和新的方向输入；自启动不自动运动。

## 验证证据与边界

- 本地水下服务 35 项测试通过，包括真实 H7 C 源码离线联测、模拟缺席启动和拔插、线程不重启、旧命令不能恢复。
- Nano `/usr/bin/python3` 为 3.10.12，现有 pyserial 3.5、websockets 16.1.1 可用；先在暂存目录完成无串口 localhost 协议握手测试，无需升级全局依赖。
- 工作电脑连接 `ws://192.168.1.188:8766`，只发送 hello，连续约 12 秒收到 123 条遥测：全部 OFFLINE、connected=false、available=false、owner=null、tx_count=0、reconnects=0，控制代次稳定。
- systemd `NRestarts=0`；已安装 6 个 Python 源文件和 unit 的 SHA256 与本地对应文件一致。
- 本次没有重启 Nano，没有刷固件或发送运动指令。实际 H7 停止握手、物理拔插和运动验收须等设备接入后进行；当前证据证明无 H7 时服务能够稳定待机。

## 备份与恢复

备份目录：`/home/nvidia/under-control-backup-20261112.a7ahva/`，包含旧 H7 unit、旧配置和切换前服务状态。原旧桥程序未删除。

查看新服务：

```bash
systemctl status under-control.service --no-pager
journalctl -u under-control.service -n 80 --no-pager
```

如明确需要恢复旧桥，在水下停机时执行以下操作；顺序保证不让两个服务同时占用串口：

```bash
sudo systemctl disable --now under-control.service
sudo systemctl enable --now nano-h7-bridge.service
```

若换用了不同 USB 转 RS485 适配器，按实际 `/dev/serial/by-id/` 修改 `/etc/under-control.env` 并重启新服务。
