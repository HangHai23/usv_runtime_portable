# USV Runtime Portable — 原版Jetson Nano移植版

2026-10-15代码更新：见[更新点、对应文件与升级步骤](docs/update_20261015.md)。包含最新联合控制职责修正、3秒丢球缓冲、持续搜索、B键脱困及红球/彩球统一选择，保留Nano ONNX和默认dry-run适配。

从`HangHai23/usv_project`当前实地开发版本整理的独立正式运行仓库。包含自动追球、球门导航、找球→拿球→送球联合控制及全部运行依赖源码、网页、配置、两种ONNX模型。原开发仓库和已有远程地址不变。

**重要：这不是“复制Orin的install/engine就能在原版Nano跑”。** 原版Nano的官方JetPack 4基于Ubuntu 18.04，而本项目使用ROS 2 Humble/Python 3.10。这里采用Ubuntu 22.04/Humble ARM64容器＋ONNX Runtime CPU后端，避免依赖Orin的TensorRT 10.3、CUDA 12和PyTorch。目标Nano实机性能、容器兼容及航行安全仍需验收，不能声称已在Nano验证。

## 最快开始

在原版Nano上，确认Docker可用后：

```bash
cd ~
git clone https://github.com/HangHai23/usv_runtime_portable.git
cd usv_runtime_portable
bash deployment/run_nano.sh image
bash deployment/run_nano.sh build
```

先修改`ros2_ws/src/*/config/*.yaml`中的Mac来源IP、串口、I²C总线和相机；具体见[完整移植指南](docs/migration_nano.md)。

断开电机或推进器，先运行不写实际PWM的联合模式：

```bash
bash deployment/run_nano.sh bench right
```

网页：`http://<Nano当前IP>:8080`，全局地图：`http://<Nano当前IP>:8080/global`。确认CH5急停、CH8模式、UDP船/球/场地、IMU方向、摄像头和推理均正常，完成安全验收后，三种**互斥**正式运行方式选一个：

```bash
# 单独自动追球
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh tracking
# 单独进右球门（左球门用left）
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh navigation right
# 找球→拿球→送入右球门
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh mission right
```

CH8低位手动、高位自动，CH5高位急停。Ctrl+C结束整套启动。仅联合模式含任务管理、持续拿球判断、最低衔接推进及受阻脱困；单独模式不含这套任务阶段逻辑。没有自动避障，停止动力不等于立即停止漂移。

不要同时运行多个模式。直接在原生Ubuntu 22.04/Humble ARM64环境运行也支持：`bash tools/build.sh`后使用根目录三个`start_*.sh`；默认`DRY_RUN=true`。**显式`DRY_RUN=false`才会输出实际PWM**。直接执行底层launch的安全参数需自行核对，不应绕过正式脚本。

## 仓库范围

- `ros2_ws/src/`：12个完整运行包，含消息定义、驱动、估计/控制、仲裁、网页、launch和配置。
- `models/`：红球和彩球v2 ONNX，两份约10MB，普通Git直接携带，不用LFS；不上传PT、Orin engine或图片。
- `vendor/imu/YbImuLib/`：运行必须的Yahboom IMU Python驱动源码及原说明。
- `tools/`：构建、正式启动及运行系统监控；不是历史实验工具。
- `deployment/`：Nano CPU容器、固定依赖、设备和日志挂载、显式运动开关。
- `docs/`：环境、结构、协议、参数、运行/验收/故障处理和迁移差异。

未上传：`validation/`、测试文件、历史路线/S型/障碍物/模拟/示教标定工具、单向电调项目、四足机器人项目、原厂家演示ROS包、数据集、运行日志、现场地图缓存、旧UI备份、SSH/访问令牌、`build/install/log`。本仓库保留最新任务状态机及安全行为，不是原项目历史全量备份。

## 文档

- [本次目标设备镜像环境与适配结论](docs/target_environment.md)
- [详细移植环境与操作](docs/migration_nano.md)
- [项目结构、节点和话题](docs/architecture.md)
- [参数调节与安全说明](docs/configuration.md)
- [UDP发送端交接协议](docs/udp_handoff.md)
- [发布范围与验证记录](docs/release_notes.md)
- [模型说明与校验](models/README.md)

修改YAML后停止并重启生效，无需重新编译；修改Python/launch/接口后重新build。容器日志持久化到宿主仓库`runtime_logs/`，不进入Git。

## 已知限制

CPU推理是兼容路径，不是Nano GPU加速或性能承诺。640输入可能无法达到所需实时频率；检测陈旧时控制器会拒绝，不应为“让它动”无限放宽超时。模型与控制已在当前Orin ARM64主机做软件验证，原版Nano容器与真实硬件未验收。Docker在当前开发主机没有访问权限，因此未声称镜像构建已通过。

公开仓库不带密钥或令牌；网页控制接口无用户登录认证，UDP来源IP过滤也不是身份认证。只用于受信任局域网，禁止公网端口转发。ROS包保留各自许可声明；第三方驱动和模型来源参见`NOTICE.md`，不能把整个项目默认视为统一MIT许可。
