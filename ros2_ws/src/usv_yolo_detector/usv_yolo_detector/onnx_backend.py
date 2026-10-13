"""Fixed-shape YOLO11 raw detection ONNX backend, no PyTorch/CUDA dependency."""
import ast
import cv2
import numpy as np


class Boxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32)
        self.cls = np.asarray(cls, dtype=np.int32)

    def __len__(self):
        return len(self.conf)


class Result:
    def __init__(self, image, boxes, names):
        self.image, self.boxes, self.names = image, boxes, names

    def plot(self):
        image = self.image.copy()
        for xy, score, cls in zip(self.boxes.xyxy, self.boxes.conf, self.boxes.cls):
            x1, y1, x2, y2 = [int(v) for v in xy]
            cv2.rectangle(image, (x1, y1), (x2, y2), (40, 220, 100), 2)
            cv2.putText(image, f'{self.names.get(int(cls), cls)} {score:.2f}',
                        (x1, max(15, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, .6, (40, 220, 100), 2)
        return image


class OnnxBackend:
    def __init__(self, path, threads=2):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(path, sess_options=options, providers=['CPUExecutionProvider'])
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError('Expected one BCHW image input')
        self.input = inputs[0]
        shape = self.input.shape
        if len(shape) != 4 or shape[:2] != [1, 3] or not all(type(v) is int for v in shape[2:]):
            raise ValueError('Portable backend requires fixed batch=1, three channels')
        self.height, self.width = shape[2:]
        metadata = self.session.get_modelmeta().custom_metadata_map
        raw_names = metadata.get('names', '{0: "ball"}')
        if len(raw_names) > 4096:
            raise ValueError('ONNX names metadata too large')
        names = ast.literal_eval(raw_names)
        if isinstance(names, list):
            names = dict(enumerate(names))
        if not isinstance(names, dict) or not names or any(type(k) is not int or not isinstance(v, str) for k, v in names.items()):
            raise ValueError('Invalid class metadata')
        if set(names) != set(range(len(names))):
            raise ValueError('Class IDs must be contiguous from zero')
        self.names = names

    def predict(self, source, conf=.5, iou=.45, imgsz=640, max_det=20, **_):
        if (self.width, self.height) != (imgsz, imgsz):
            raise ValueError(f'ONNX input is fixed at {self.width}x{self.height}; set image_size accordingly')
        h, w = source.shape[:2]
        scale = min(self.width/w, self.height/h)
        nw, nh = round(w*scale), round(h*scale)
        dw, dh = (self.width-nw)/2, (self.height-nh)/2
        left, top = round(dw-.1), round(dh-.1)
        resized = cv2.resize(source, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = cv2.copyMakeBorder(resized, top, self.height-nh-top, left,
                                   self.width-nw-left, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        dtype = np.float16 if self.input.type == 'tensor(float16)' else np.float32
        tensor = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=dtype)/255.
        prediction = self.session.run(None, {self.input.name: tensor})[0]
        if prediction.ndim != 3 or prediction.shape[0] != 1:
            raise ValueError('Expected raw YOLO11 [1,4+classes,anchors] output; NMS/segmentation exports unsupported')
        prediction = prediction[0]
        channels = 4+len(self.names)
        if prediction.shape[0] == channels:
            prediction = prediction.T
        elif prediction.shape[1] != channels:
            raise ValueError('Output class count does not match metadata')
        cls = prediction[:, 4:].argmax(axis=1)
        scores = prediction[np.arange(len(prediction)), cls+4]
        mask = (scores >= conf) & np.isfinite(prediction).all(axis=1)
        selected, scores, cls = prediction[mask], scores[mask], cls[mask]
        xy = selected[:, :4].copy()
        xy[:, :2] -= xy[:, 2:]/2
        xy[:, 2:] += xy[:, :2]
        xy[:, [0, 2]] = np.clip((xy[:, [0, 2]]-left)/scale, 0, w)
        xy[:, [1, 3]] = np.clip((xy[:, [1, 3]]-top)/scale, 0, h)
        valid = (xy[:, 2] > xy[:, 0]) & (xy[:, 3] > xy[:, 1])
        xy, scores, cls = xy[valid], scores[valid], cls[valid]
        kept = []
        for class_id in np.unique(cls):
            indices = np.flatnonzero(cls == class_id)
            boxes = xy[indices].copy()
            boxes[:, 2:] -= boxes[:, :2]
            nms = cv2.dnn.NMSBoxes(boxes.tolist(), scores[indices].tolist(), conf, iou)
            kept.extend(indices[np.asarray(nms, dtype=int).reshape(-1)].tolist())
        kept = sorted(kept, key=lambda index: float(scores[index]), reverse=True)[:max_det]
        return [Result(source, Boxes(xy[kept], scores[kept], cls[kept]), self.names)]
