# 来源与第三方说明

- 正式USV ROS源码/网页/参数来自HangHai23的本机`usv_project`，本发布保留各包`package.xml/setup.py`的原MIT声明。本仓库未新增一个覆盖所有素材的统一许可。
- `vendor/imu/YbImuLib`为Yahboom Team驱动，保留作者、源码和原README；输入版本未提供独立LICENSE文件，不能由本次整理替厂家授予新的许可。
- ONNX模型来自原项目红球模型及用户提供的彩球YOLO11n v2模型。训练框架/模型权利由其来源条款决定；公开上传不是重新许可。
- ROS 2、OpenCV、NumPy、ONNX Runtime、aiohttp、pyserial、Docker基础镜像等第三方组件按各自条款安装，不把依赖包二进制或用户凭据复制进仓库。
- 原版Nano官方软件生命周期及本版本测试边界见移植文档。不得将未做硬件验收的软件描述为已经证明安全或已完成Nano水上实测。
