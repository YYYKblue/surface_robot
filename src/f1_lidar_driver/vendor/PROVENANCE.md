# PAVO2 来源与迁移边界

- 导入日期：2026-10-31。
- 来源设备：`nvidia@192.168.1.188`，Ubuntu 22.04 / ROS 2 Humble / ARM64。
- 来源目录：`/home/nvidia/ros2_ws/src/my_robot_stack/pavo2_ros2`。
- 原包名与声明版本：`pavo2_ros`，`2.1.2`。目录包含现有 ROS 2 适配，不等同于未经修改的厂商发布包。
- `source_manifest.json` 记录本次读取的源文件 SHA-256（不包含 Python 缓存），用于后续比较；没有已核实的上游 Git 提交号。

`pavo2_sdk/`、`Readme.vendor`、本包 `src/*.cpp` 及 `include/f1_lidar_driver/pavo2_utils.h` 按字节保留源文件。SDK 作为包内静态库编译，不依赖原工作空间的二进制文件。包名、CMake、launch、参数文件、服务模板与项目文档为迁移适配。

`Readme.vendor` 含历史 ROS 1 命令，仅作来源资料；当前使用方式以本包 README 为准。原驱动说明 PDF 保留在 SDK 目录中。

## 许可记录

原 `package.xml` 声明 `BSD`，原文件保存在 `package.xml.upstream`；本包沿用该声明。`angles.h` 含 Willow Garage 的 BSD 许可正文，原文件及单独的 `LICENSE.angles` 均保留。源目录未提供覆盖整个 SDK 的独立 LICENSE 文件；本次没有补写或推定其他版权持有人及许可条款。

## 本次没有改变的运行语义

- 距离单位换算 `distance * 0.002`、角度减 π、越界回波置零和 `inverted` 行为。
- 扫描时间戳在读取帧前取节点时间；沿用电机转速计算的扫描时长与逐点时间。
- 发布 QoS 为 Reliable、depth=10；部署配置发布 `/pavo2_scan`，帧名为 `pavo2s_frame`。
- 首次连接失败退出；连续读取超时超过 6 次后尝试重连；退出时停止数据并关闭连接。

初次连接常驻重试、诊断话题和时间戳改进属于后续工作，不包含在本次封装中。
