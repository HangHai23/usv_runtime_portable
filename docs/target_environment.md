# 本次目标设备环境（用户提供的镜像核查清单）

来源：用户提供的`Jetson镜像环境清单.md`，核查日期2026-10-11；本次于2026-10-13读取。内容依据镜像文件和安装元数据的离线检查，不是本次在目标设备运行的检测结果。

| 项目 | 清单记录 |
|---|---|
| 设备 | 原版Jetson Nano，图为T501 SD启动适配/暖重启修复沿用原软件环境 |
| 宿主 | Ubuntu 18.04.6 LTS ARM64 |
| JetPack / L4T | 4.6.5 / 32.7.5 |
| 内核 | 4.9.337-tegra |
| CUDA / cuDNN | 10.2 / 8.2.1.32 |
| TensorRT | 8.2.1 |
| DeepStream | 6.0.1 |
| Python | 3.6.9 / 2.7.17 |
| PyTorch / torchvision | 1.10.0（CUDA10.2）/ 0.11.1 |
| TensorFlow | 2.7.0+nv22.1 |
| ROS实际版本 | Melodic，`/opt/ros`仅melodic；镜像名Noetic不能当实际版本 |
| OpenCV | 系统3.2、NVIDIA4.1.1、自编译4.6.0（CUDA10.2配置）共存 |
| YOLO旧环境 | YOLOv5 v6.2；不是本项目YOLO11运行环境 |
| Docker / NVIDIA runtime | 20.10.21 / 3.7.0 |
| GCC / CMake / pip | GCC7 / CMake3.10.2 / 系统pip9.0.1 |

## 对本项目的直接影响

1. 不能在宿主Python3.6运行本ROS2 Humble代码；也不能将镜像中的Melodic当成ROS2环境。无需卸载它们，使用独立Humble ARM64容器。
2. Docker已安装记录不代表daemon/权限/拉取镜像已通过；在板端先检查`docker info`。CPU容器显式runc，不注入宿主CUDA10.2/NVIDIA库。
3. 不复制宿主的`/usr/local/lib`、Python3.6用户库、ROS1工作空间或LD_LIBRARY_PATH进容器；容器用自己的cv_bridge和OpenCV，避免三个OpenCV版本混链。
4. 不把当前Orin的CUDA12/TensorRT10.3 engine放到目标Nano。现有两份ONNX用固定Python3.10 CPU依赖执行；镜像中的YOLOv5/PyTorch1.10不能直接替代YOLO11后端。
5. 目标Nano GPU优化将来需要TensorRT8.2.1实际ONNX解析/构建/算子验证及相应运行接口；本次未做，也不保证旧TRT支持全部导出算子。
6. 图为T501载板/设备树会影响UART/I²C/相机设备路径。仓库I²C1只作待核对候选，不声称是此载板实际PWM口；以板端总线和针脚记录为准。
7. 清单明确GPU推理、相机、ROS通信和Docker运行尚未执行核查。本仓库也没有目标Nano实机，必须按迁移指南验收，不能把文件存在当作测试通过。

原宿主`.bashrc`加载Melodic、CUDA10.2和/usr/local/lib。运行宿主`deployment/run_nano.sh`只将指定ROS domain/网页/安全开关传入容器，不传递整个宿主环境；没有自动修改镜像的启动/设备树，也没有触碰T501暖重启修复。
