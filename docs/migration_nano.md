# 原版Jetson Nano完整移植指南

## 1. 环境必须先分清

目标是**原版Jetson Nano/Tegra X1**，不是Orin Nano。本仓库运行栈为ROS 2 Humble、Python 3.10、ARM64。

已读取用户的[具体目标环境清单](target_environment.md)：JetPack4.6.5/L4T32.7.5、Ubuntu18.04.6、内核4.9.337、Python3.6.9、TensorRT8.2.1、Docker20.10.21，实际ROS为Melodic。图为T501设备树适配未升级这些软件。因此仍使用以下容器方案；不用重新卸载/覆盖ROS1，也不将已有YOLOv5环境当YOLO11后端。

| 层级 | 原版Nano方案 | 不可直接沿用 |
|---|---|---|
| 宿主 | 官方JetPack 4.x / L4T R32 / Ubuntu 18.04 | Orin JetPack 6的系统镜像 |
| 用户空间 | Docker中的Ubuntu 22.04＋Humble | 在18.04原生apt安装Humble |
| 推理 | ONNX Runtime 1.16.3 CPU、NumPy 1.24.4 | TensorRT 10.3 engine、CUDA 12库 |
| 硬件 | USB相机、IMU、UART接收机、I²C/UART PWM板 | Orin的I²C总线编号和UART用途 |
| 控制 | 保留已开发状态机/仲裁/安全检查 | 当前船体速度增益不能视为通用标定值 |

依据：[NVIDIA R32.7.6](https://developer.nvidia.com/embedded/linux-tegra-r3276)描述Ubuntu 18.04基础系统；[ROS Humble安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)以Ubuntu 22.04为二进制平台；[ONNX Runtime安装文档](https://onnxruntime.ai/docs/install/)提供CPU发行版。

JetPack 4已终止维护：[NVIDIA公告](https://forums.developer.nvidia.com/t/announcing-end-of-life-for-nvidia-jetpack-4-with-the-release-of-jetpack-4-6-6/314409)。不要将本方案用于暴露公网的设备。容器不更换内核、不赋予Orin GPU能力；CPU方案没有使用NVIDIA容器运行时，显式`--runtime=runc`。原版Nano的GPU加速另需L4T R32兼容工具链、导出/构建及全链路复测，当前不提供未经实测的GPU速度保证。

## 2. 宿主设备与资源

建议4GB Nano、稳定供电、32GB以上可用存储、可靠散热。2GB版本资源更紧，需要实际评估。整套多进程ROS与640 CPU推理会竞争CPU/内存；构建使用串行，仍可能需要按目标设备配置交换空间。不自动修改宿主功耗、交换分区、内核或网络地址。

在目标终端查看，不要把版本猜成Orin：

```bash
uname -m
head -n 1 /etc/nv_tegra_release
cat /etc/os-release
free -h
df -h
docker --version
docker info
```

应为aarch64。Docker若未安装，可先按当地JetPack镜像的安装方式准备；Ubuntu apt提供时：

```bash
sudo apt update
sudo apt install git docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

重新登录后`docker info`确认权限。Docker组具有高权限，不共享账号。命令涉及sudo时由操作者本机输入密码，不将密码写入仓库。

## 3. 下载、校验与构建

```bash
cd ~
git clone https://github.com/HangHai23/usv_runtime_portable.git
cd usv_runtime_portable
(cd models && sha256sum --check SHA256SUMS)
bash deployment/run_nano.sh image
bash deployment/run_nano.sh build
```

`image`下载官方ARM64 ROS基础镜像、安装OpenCV/cv_bridge/Python依赖及随仓库提供的IMU库。`build`在挂载工作空间编译12包、生成自定义消息和入口；产生的`build/install/log`仅留本机。首次构建需要互联网和较长时间。更换硬件或ROS环境后不能搬运旧install副本。

CPU依赖锁定在`deployment/requirements-cpu.txt`，有Linux ARM64/Python 3.10预编译wheel。不要在宿主18.04/Python 3.6执行这份pip安装；它是容器3.10用的。

## 4. 接线与设备参数（必须修改）

不要带桨、带动力在无人看管状态下测试；先保持CH5急停、CH8手动。串口TTL逻辑电平必须与Nano兼容，共地，接收机TX接Nano RX；接收机供电与信号电平不是同一概念，不将5V电源直接接UART信号。

| 配置 | 检查项 |
|---|---|
| `usv_imu_driver/config/imu_driver.yaml` | `port`/`fallback_port`，USB IMU常为ttyUSB0，也可用稳定by-id链接 |
| `usv_crsf_receiver/config/crsf_receiver.yaml` | `port`、420000波特率；Nano引脚UART与控制台不能同时占用 |
| `usv_pwm_actuator/config/pwm_actuator.yaml` | `backend`、`i2c_bus`、`i2c_address`或`port/baudrate`；Nano候选总线1不等于已经确认 |
| `usv_camera_driver/config/camera.yaml` | `device`、1280×720、MJPG、采集/发布频率 |
| `usv_udp_position/config/udp_position.yaml` | `allowed_sender_ip`改Mac当前IPv4，`boat_id`匹配标签；初始127.0.0.1有意不接受外部Mac |

宿主查看设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyTHS* /dev/i2c-* /dev/video*
v4l2-ctl --list-devices
i2cdetect -l
ip -br address
```

`i2cdetect -l`仅列总线；I²C主动扫描需确认硬件可安全扫描后自行执行。PWM板原地址0x2D（十进制45），通道1左、2右；双向电调1500us停止、2000前进、1000后退，角度映射与微调必须按真实板复测。不要沿用单向电调1190/1100启动设置到双向模式；这些历史字段当前不会在`forward_only:false`下生效。

原生运行用户权限用dialout/video/i2c组或受限udev规则，不用`chmod 777`作永久方案。Docker设备通过`--device`映射；不使用`--privileged`。先插好设备再启动容器，容器启动时不存在的新设备通常需要重启容器才能新增映射。主机串口被系统服务占用、驱动缺失的问题不能靠容器解决；需要先在宿主修好，不盲目卸载系统服务。

## 5. Mac网络与场地

Mac目标地址填**Nano当前IP**，UDP端口5700。Nano的`bind_address:0.0.0.0`保持监听网卡，`allowed_sender_ip`填Mac IP。两者不可反填。发送端需启用/标定场地或合法手动固定场地，提供球门、船位/朝向和球全局米制位置；只发送检测框、`field_status:disabled`或`xy_m:null`不能驱动全局导航。

复制官方流程对应的[协议说明](udp_handoff.md)给Mac维护者。接收端允许发送端先启动、重新启动，但发送目标IP不能错误；坐标系/会话变化按原安全机制可能要求CH8回手动再切自动。没有可靠初始船位时，IMU不能凭空生成绝对位置。

## 6. 先不上动力验收

```bash
cd ~/usv_runtime_portable
bash deployment/run_nano.sh bench right
```

此入口明确设置DRY_RUN=true，仍运行真实传感器、网络与逻辑，不输出实际PWM。另一终端进入相同宿主网络/ROS domain的诊断容器：

```bash
cd ~/usv_runtime_portable
bash deployment/run_nano.sh shell
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list
ros2 topic hz /imu/data_raw
ros2 topic echo /mission/status
ros2 topic echo /actuators/pwm_state
ros2 topic echo /ball/detection
ros2 topic echo /vision/global_observation
```

逐项检查：CH5切换对应emergency_stop_active；CH8切换对应manual/automatic；手动正反/转向PWM符号正确；IMU实转方向正确；网页船/球/门位置正确；`BallDetection.inference_ms`、消息头实际年龄及target_valid满足控制超时。检查`dry_run:true`，不要因自动无输出而直接解除保护。

## 7. 性能验收，不盲目放宽超时

原始安全参数包括IMU/目标超时约0.2秒、任务相机检测最大年龄0.25秒、20Hz控制。CPU推理可能比Orin TensorRT慢很多，不能承诺Nano保留100Hz IMU和20Hz控制同时推理实时性。

先将相机`publish_rate`适当降低如10Hz，保留1280×720采集需求，以减少ROS图像传输。`onnx_threads:2`预留CPU给控制，不开满线程。查看`/ball/detection`时间戳和推理耗时、`/mission/status`、系统日志。若检测过期，先降低负载/提高硬件能力，再评估允许的时间上限；增加时间上限会增加预测盲区和制动距离，必须保持低速、重新水上标定。静态ONNX输入固定640，不能只把`image_size`改320；需要另行导出320模型并验证。

本仓库控制参数来自原船，较高巡航/差速和最低衔接推进不是新硬件安全默认标定。第一次真实测试应调低`output_limit_percent`，检查启动阈值和制动，逐步提升；本次未替用户静默重写这些实地控制增益。

## 8. 正式运行（只选一种）

从宿主执行，先结束bench容器：

```bash
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh tracking
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh navigation right
USV_ALLOW_MOTION=YES bash deployment/run_nano.sh mission right
```

不是依次执行三条，而是按目标选一条。tracking只用船载相机追球，不启动UDP找球/送球任务；navigation依赖UDP场地与船位，直接入选球门；mission先全局找球、再视觉对准、确认拿球、送球。CH8手动/自动、CH5急停全部保留。多个模式同时运行会争用串口和自动动力，启动脚本会拒绝已有控制进程。多个主机共享ROS_DOMAIN_ID也会互相影响；不需要跨机ROS时两端统一设ROS_LOCALHOST_ONLY=1，Mac UDP仍工作。

网页端口可通过`WEB_PORT=8081`传入宿主脚本。页面根路径实时监控，`/global`全局任务，`/healthz`健康接口。别把0.0.0.0或电脑localhost当成Nano地址。

## 9. 日志与修改

宿主`runtime_logs/`挂载为容器`/root/Downloads`，包含`usv_mission_logs`、`usv_navigation_logs`、`usv_logs`、`usv_mission_system_logs`及`usv_runtime_state`。系统监控不依赖ROS回调，保留CPU/内存/I/O/线程等待、网页健康和节点完整stdout/stderr。容器默认无法读宿主journal，kernel日志可能不可用；需要操作者另行在宿主保存内核诊断，不因此增加privileged权限。

YAML在宿主源码直接修改，停止并重启；代码/launch/消息修改运行build。安装系统依赖或IMU库源代码变化要重建image；ONNX变更用SHA256核对后重启推理。

## 10. 常见故障

| 现象 | 检查与处理 |
|---|---|
| `Permission denied docker.sock` | 宿主Docker权限；重新登录确认组，不发送密码给他人 |
| 镜像apt/pip失败 | 网络/软件源及ARM64 wheel；不能把AMD64文件装到Nano |
| 容器线程创建`Operation not permitted` | 老Docker/seccomp与新glibc兼容性；优先更新可兼容的Docker。仅明确确认是seccomp、在可信环境下评估`--security-opt seccomp=unconfined`，当前脚本未默认开启 |
| 模型输入尺寸错误 | 两个ONNX都是1×3×640×640，image_size必须640 |
| 模型载入正常但无视觉动力 | 检测年龄/目标有效性/IMU方向/置信度/enable；不要禁用watchdog |
| 自动`wait_global` | 检查场地有效、船位/朝向、球全局坐标和Mac来源IP |
| 只有球门看不到船 | 地图可以缓存，船位不能当作地图永久缓存 |
| 串口/总线不存在 | 宿主设备与Nano针脚设置，不是拷贝源码能修复的内核问题 |
| 摄像头换插后找不到 | 确认宿主video节点，重启容器增加映射；可固定by-id路径 |
| 网页访问不到 | 脚本仍在运行？IP/端口/同网段？localhost健康与外部网络分别检查 |
| 受阻误触发 | 新鲜位置仍可能噪声或冻结；调stuck容差/动力/时间，先低速测，不能当真实边界传感器 |

## 11. 当前验证边界

移植代码和两份ONNX在当前Orin ARM64主机做软件构建/推理/依赖检查；并非目标Nano实机验收。开发主机没有Docker daemon权限，未执行Docker镜像构建，不隐瞒这一限制。上传完整必要源码不等于保证原版Nano实时性能；必须完成以上bench、位置/方向/PWM、CPU延迟和水上低速验收后使用。
