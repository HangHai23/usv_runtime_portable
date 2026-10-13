# 随仓库携带的ONNX模型

| 文件 | 类别 | 输入/输出 |
|---|---|---|
| `red_ball_yolo11n.onnx` | 0: redball | FP32、1×3×640×640 → 1×5×8400 |
| `colored_ball_v2_yolo11n.onnx` | 0: water_ball | FP32、1×3×640×640 → 1×5×8400 |

两个模型都直接提交普通Git，clone即可获得，不依赖LFS、PT或Orin engine。校验：在本目录`sha256sum --check SHA256SUMS`。

默认使用红球。修改源码`usv_yolo_detector/config/yolo_detector.yaml`中的历史字段`engine_path`为相对ONNX路径后重启，字段名称不代表只能engine。固定640，不可只修改image_size来变320。

新CPU后端不导入Ultralytics/PyTorch；直接ONNX Runtime解码YOLO11原始输出并NMS。红球旧图4/4检出，彩球旧红球图在0.50阈值下3/4检出，另一图置信度约0.32，与原模型一致。未包含这些验证图片或验证脚本，结果记录在发布说明。

ONNX格式可移植不代表所有TensorRT版本都支持其全部算子。未来Nano GPU加速需用目标设备实际TensorRT重新解析/构建和校验，不能使用Orin TensorRT 10.3 engine。本仓库CPU路线无需先转换engine。

模型训练/来源按原项目记录，彩球原权重名为recommended_v2_yolo11n_best.pt，红球为原best.onnx。模型相关权利/条款不可因运行后端不使用Ultralytics而自动变成MIT，详见根目录NOTICE。
