# 配置和调节

路径均相对仓库根目录。所有正式launch优先源码YAML；本移植版还将旧launch中的串口/I²C默认值改为读取YAML，避免旧安装副本或硬编码总线覆盖修改。CLI参数仍可显式覆盖，启动时核对日志。

## 必改设备、网络与模型

| 文件 | 关键字段 |
|---|---|
| `usv_udp_position/config/udp_position.yaml` | `allowed_sender_ip`必须Mac来源IP，`port:5700`，`boat_id`，方向偏移和坐标系 |
| `usv_imu_driver/config/imu_driver.yaml` | `port/fallback_port`，安装方向，硬件/发布频率100Hz |
| `usv_crsf_receiver/config/crsf_receiver.yaml` | `port`，`baudrate`及连接超时 |
| `usv_pwm_actuator/config/pwm_actuator.yaml` | backend、总线/地址、串口、双向电调、限幅、左右trim、前后启动补偿 |
| `usv_camera_driver/config/camera.yaml` | 相机路径、1280×720、MJPG、发布频率 |
| `usv_yolo_detector/config/yolo_detector.yaml` | ONNX路径，`onnx_threads:2`，conf0.50，固定640输入，debug图开启 |

上表包路径前缀是`ros2_ws/src/`。红球模型用`models/red_ball_yolo11n.onnx`，彩球用`models/colored_ball_v2_yolo11n.onnx`，两者类别ID均为0；不要同时修改类别编号。路径也接受绝对路径，但容器内路径与宿主不同，建议相对路径。

## 手动控制和电调

当前双向：1号左、2号右，0动力1500us，100%前进2000us，-100%后退1000us。CH2油门、CH4转向允许一前一后；CH5>1500急停，CH8>1500自动。反向和微调参数修改后先在无桨台架查看脉宽，不凭参数名称推断接线正确。

`output_limit_percent`是最终全局限幅，作用于遥控、追球、导航和脱困。左右trim为各自增益修正，零指令仍保持停止位。换设备/船体后先降低限幅做安全标定，不直接上原船100%输出。网页参数修改是运行时修改，不自动持久化到YAML；重启后以当前源码配置为准。

## 追球过冲与慢响应

`usv_ball_follow/config/ball_follow_controller.yaml`：`bearing_kp`控制偏角转速请求，`maximum_yaw_rate_rad_s`限制转速，`yaw_brake_preview_sec`利用IMU提前制动。过冲先降低bearing_kp/最大转速并适当增加提前制动；左右振荡不要一味增大转向。当前模型控制下`yaw_feedback_gain`起实际反馈作用，旧`yaw_rate_kp`并非主要模型增益。

`target_estimator.yaml`包含视场角、方向、alpha/beta滤波、预测/丢失时间；相机姿态和水平FOV改变必须重测。CPU推理太慢时先减负载，不把预测当成永远有效。

## 导航

`usv_goal_navigation/config/goal_navigation.yaml`：速度/转向相关参数在文件末段，包括巡航速度、最大角速度、heading_kp、yaw反馈、模型增益和转向阶段参数。模型m/s不是测量到的真实船速；必须根据实船日志重新标定。UDP新位置校正预测，短时断流可继续推算，超过10秒等故障停动力；地图/球门长期缓存不意味着船位也能永久缓存。

按CH8切自动从当前位置进入所选球门，穿门线后0.5m停动力；门前有朝向修正，没有路径绕障。场地缓存保存在`~/Downloads/usv_runtime_state/map_cache.json`，容器对应宿主runtime_logs，首次部署为空。

## 拿球、衔接和丢球重找

`usv_mission_manager/config/mission.yaml`保存当前原船调参快照：

| 字段 | 当前值/含义 |
|---|---|
| `capture_area_ratio` | 0.28，框占画面28%仅为面积条件，不单凭它证明拿到球 |
| `capture_hold_sec`、`drone_held_confirm_sec` | 0.05s联合证据稳定时长 |
| `drone_vote_duration_sec` | >0.08s新帧覆盖才评估 |
| `drone_vote_ratio/min_frames/window_sec` | ≥80%、至少2帧、最近0.25s；重复/过期帧不续期 |
| `capture_creep_area_ratio` | 0.08，提前开始最低推进 |
| `capture_min_forward_percent` | 30%，近球/衔接/送球平均正向动力下限，不保证恒速 |
| `capture_handover_timeout_sec` | 2s是等待首条导航候选最长容忍，不是强制等待时间 |
| `reacquire_speed_m_s` | 0.60模型速度；视觉丢失用有效全局数据重新对准 |
| `ball_loss_confirm_sec` | 新鲜非held持续0.3s判丢球 |
| `possession_timeout_sec` | 2s无新held使拿球状态失效 |

照搬高动力、低确认时长可能误认拿球；框面积更换模型也会变化。在目标Nano先bench核对状态，再低速实测。达到任一确认条件后当次周期切送门，没有额外人为等待；帧频、CPU推理、无人机自身识别和传输延迟仍真实存在。

## 疑似受阻转向

联合所有自动阶段：指令及PWM反馈均为平均前进≥10%，不同新帧的实测位置覆盖>1s且始终在10cm内，触发左前35%/右后35%。最多1s或IMU累计0.8rad，恢复任务后冷却2s。`stuck_*`参数全部在mission.yaml。它只判断疑似受阻，不证明边界；定位冻结、水草、弱动力也可能触发。持球阶段是禁止倒转规则的例外，可能造成丢球。手动、急停、故障和数据缺失优先。

## 系统日志

`tools/runtime_monitor.yaml`控制1秒系统采样、5秒详细线程/温度采样、网页0.5秒检查、压力告警阈值。它不调整动力，不自动杀控制节点；监控也无法在整机断电/磁盘完全卡死时保证落盘。容量较小的Nano注意日志增长，操作者自行归档已确认的历史日志，不上传敏感现场原始数据。
