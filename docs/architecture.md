# 项目结构和运行数据流

```text
usv_runtime_portable/
├── start_tracking.sh              单独视觉追球（遥控可接管）
├── start_goal_navigation.sh       单独导航到left/right球门
├── start_ball_mission.sh          全局找球→视觉拿球→送入球门
├── deployment/                   Nano CPU镜像、安装依赖、宿主容器入口
├── models/                       红球和彩球ONNX及校验值
├── vendor/imu/YbImuLib/           厂家IMU串口库，构建镜像时安装
├── ros2_ws/src/                  12个正式ROS包
├── tools/                        构建、互斥启动、系统监控
├── docs/                         移植/协议/调参/验证边界
└── runtime_logs/                 运行后生成，本机持久化，不提交
```

## 12个包各做什么

| 包 | 作用/主要运行实体 |
|---|---|
| `usv_interfaces` | CRSF、动力指令、PWM状态、检测框、目标状态和全局位置消息定义；所有节点共同依赖 |
| `usv_crsf_receiver` | UART解析CRSF，发布16通道、RSSI/LQ/SNR和连接状态 |
| `usv_rc_teleop` | CH2油门＋CH4转向差速，CH8手动/自动仲裁，断联/超时保护 |
| `usv_pwm_actuator` | 最终百分比→I²C/UART PWM；双向映射、限幅、微调、启动补偿、CH5急停、看门狗 |
| `usv_imu_driver` | Yahboom串口驱动；100Hz设置、坐标/四元数、角速度、磁场和方向数据 |
| `usv_camera_driver` | USB相机自动寻找/重连，1280×720，最新图像发布 |
| `usv_yolo_detector` | `.onnx`用CPU后端，保留可选`.engine`加载接口；最新帧覆盖、不积压，框/置信度/可视化发布 |
| `usv_ball_follow` | 像素转方位、目标估计、IMU预测、偏角/角速度控制、记录CSV |
| `usv_udp_position` | UDP5700接收、来源过滤、协议检查、会话恢复；发布完整全局观测 |
| `usv_goal_navigation` | 场地/门线记忆、视觉校正＋IMU/动力预测、直达球门、穿门0.5m停动力、日志和离线日志分析 |
| `usv_mission_manager` | 联合阶段管理，选择唯一自动动力来源、确认/丢球重找、衔接推进、疑似受阻脱困 |
| `usv_web_monitor` | 同网段网页、推理画面、船/球/门地图、链路/动力/安全/系统监控及约束参数接口 |

未包括`usv_position_estimator`：正式球门导航已有自身延迟估计器，三个正式入口均不依赖独立位置估计包。没有缺失它的运行依赖。

## 动力主链

```text
遥控器 → CRSF → 遥控差速 → /propulsion/manual_command ─┐
                                                   ├→ 仲裁 → /propulsion/command → PWM板 → 电调
自动候选 → 任务管理器 → /propulsion/automatic_command ┘
```

联合模式：视觉控制只发`/mission/visual_command`，导航只发`/mission/navigation_command`；只有任务管理器能将选中的候选转成正式自动指令，避免两个算法同时控制电机。

单独追球/导航：相应控制器直接发布`/propulsion/automatic_command`，不启动联合管理器。不要同时启动它们。CRSF低位CH8接手动，高位接自动；CH5和PWM watchdog在最终执行器层再保护。

## 主要话题

| 话题 | 内容 |
|---|---|
| `/rc/channels` | 原始/微秒通道、连接与帧率 |
| `/rc/link_statistics`、`/rc/connected` | 链路质量与在线状态 |
| `/imu/data_raw` | 带ROS时间戳的姿态、角速度、加速度 |
| `/camera/image_raw` | 图像与原始采集时间戳 |
| `/ball/detection` | 框xyxy、归一化中心/面积、置信度、推理耗时 |
| `/ball/debug_image` | 推理画面（网页使用） |
| `/ball/target_state` | IMU预测后的目标偏角、跟踪状态、检测年龄 |
| `/vision/global_observation` | Mac协议观测、船位/朝向、球位、场地/门线及接收有效性 |
| `/vision/udp_diagnostics` | 接收/拒绝/来源/会话等诊断 |
| `/navigation/status` | 预测船位、路线目标、速度/角速度及降级状态 |
| `/mission/status` | 阶段、持球、确认统计、阻断原因、脱困、选择的动力 |
| `/mission/navigation_enabled`、`/mission/visual_enabled` | 联合阶段使能心跳 |
| `/control/automatic_mode` | 仲裁器模式状态，不要用另一个发布器强行覆盖 |
| `/actuators/pwm_state` | 请求/应用动力、脉宽、急停、看门狗、dry_run |

`engine_path`字段名称为历史兼容接口：现在也支持`.onnx`。ONNX后端执行letterbox、RGB归一化、YOLO11无objectness输出解析、按类NMS、还原到原图坐标；面积定义仍是检测框面积/全画面面积，不是球圆形轮廓的面积。

## 联合阶段和异常

`manual → approach_ball → visual_acquire → deliver_goal → complete`。缺可靠船/球坐标时`wait_global`，会话/坐标系变动时可能`wait_manual_rearm`。视觉丢失且全局依据有效时立即重对准；拿球可通过held＋框面积联合确认，或无人机连续新帧多数held独立确认。丢球后清除旧持球状态和候选，重新找球。

普通持球送门不倒转；疑似受阻脱困是按最新逻辑增加的例外，可能倒转一侧并导致丢球。它只覆盖联合自动阶段，不覆盖手动或独立脚本。急停、真实手动接管、IMU/关键输入失效、超时和任务完成优先于最低衔接推进和脱困。

## 日志不是上传数据

任务日志完整记录无人机观测、IMU/状态、候选和最终动力、持球证据、阶段变化；导航日志记录规划/融合预测与速度/角速度差异。系统记录器独立于ROS，HTTP检查独立线程；ROS节点stdout/stderr含自动线程栈转储。日志随本次启动时间分目录写宿主`runtime_logs/`，不把旧现场状态用于新场地初始定位。

`tools/capture_runtime_health.py`是正式运行诊断，不是历史验证项目。`navigation_analyze`是已保留的日志分析入口；路线模拟/自动调参试验、示教标定不在此发布范围。
