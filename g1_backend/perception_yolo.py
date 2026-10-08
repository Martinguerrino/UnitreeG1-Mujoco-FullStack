"""Local YOLO11 segmentation inference through CPU ONNX; no runtime downloads."""

import ast
from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Instance:
    label: str
    confidence: float
    mask: np.ndarray
    box_xyxy: list


def suppress_contained_masks(instances, threshold=0.7):
    """Remove same-class fragments that duplicate a stronger instance mask."""
    retained = []
    for instance in sorted(instances, key=lambda item: item.confidence, reverse=True):
        duplicate = any(
            previous.label == instance.label
            and np.count_nonzero(previous.mask & instance.mask)
            / max(1, min(previous.mask.sum(), instance.mask.sum()))
            >= threshold
            for previous in retained
        )
        if not duplicate:
            retained.append(instance)
    return retained


def decode_outputs(
    prediction,
    prototypes,
    names,
    image_shape,
    input_shape,
    confidence=0.3,
    iou=0.5,
    max_detections=30,
):
    """Decode YOLO11-seg tensors and restore masks to registered RGB resolution."""
    nc = len(names)
    coefficients = prototypes.shape[1]
    values = prediction[0].T
    if values.shape[1] != 4 + nc + coefficients:
        raise ValueError(
            'Expected YOLO11 segmentation output: XYWH + class scores + mask coefficients'
        )
    scores = values[:, 4 : 4 + nc]
    labels = scores.argmax(axis=1)
    confidences = scores.max(axis=1)
    keep = np.flatnonzero(confidences >= confidence)
    if not len(keep):
        return []
    values, labels, confidences = values[keep], labels[keep], confidences[keep]
    height, width = image_shape
    ih, iw = input_shape
    ratio = min(iw / width, ih / height)
    resized_w, resized_h = round(width * ratio), round(height * ratio)
    left, top = round((iw - resized_w) / 2 - 0.1), round((ih - resized_h) / 2 - 0.1)
    boxes = values[:, :4].copy()
    boxes[:, :2] -= boxes[:, 2:] / 2
    selected = cv2.dnn.NMSBoxes(
        boxes.tolist(), confidences.tolist(), confidence, iou, top_k=max_detections * 10
    )
    selected = np.asarray(selected).reshape(-1)[:max_detections]
    instances = []
    for index in selected:
        x, y, bw, bh = boxes[index]
        xyxy = [
            (x - left) / ratio,
            (y - top) / ratio,
            (x + bw - left) / ratio,
            (y + bh - top) / ratio,
        ]
        x0, y0 = max(0, round(float(xyxy[0]))), max(0, round(float(xyxy[1])))
        x1, y1 = min(width, round(float(xyxy[2]))), min(height, round(float(xyxy[3])))
        if x1 <= x0 or y1 <= y0:
            continue
        logits = values[index, 4 + nc :] @ prototypes[0].reshape(coefficients, -1)
        logits = logits.reshape(prototypes.shape[2:])
        ph, pw = logits.shape
        gain = min(pw / width, ph / height)
        mask_left, mask_top = (pw - width * gain) / 2, (ph - height * gain) / 2
        logits = logits[
            round(mask_top - 0.1) : ph - round(mask_top + 0.1),
            round(mask_left - 0.1) : pw - round(mask_left + 0.1),
        ]
        logits = cv2.resize(logits, (width, height), interpolation=cv2.INTER_LINEAR)
        mask = np.zeros((height, width), dtype=bool)
        mask[y0:y1, x0:x1] = logits[y0:y1, x0:x1] > 0.0
        if not mask.any():
            continue
        instances.append(
            Instance(names[int(labels[index])], float(confidences[index]), mask, [x0, y0, x1, y1])
        )
    return instances


class YoloSegmenter:
    def __init__(self, model_path, confidence=0.3, iou=0.5, threads=2, deduplicate=True):
        path = Path(model_path).expanduser()
        if not path.is_absolute():
            path = ROOT / path
        if not path.is_file():
            raise ValueError(
                f'YOLO checkpoint not found: {path}. Run scripts/setup_perception_yolo.sh'
            )
        if not 0 < confidence < 1 or not 0 < iou < 1 or threads < 1:
            raise ValueError('Invalid YOLO confidence, IoU or thread count')
        runtime = ROOT / 'artifacts/perception/python'
        if str(runtime) not in sys.path:
            sys.path.append(str(runtime))
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise ValueError(
                'Missing ONNX runtime; run scripts/setup_perception_yolo.sh'
            ) from error
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(path), sess_options=options, providers=['CPUExecutionProvider']
        )
        self.input = self.session.get_inputs()[0]
        self.input_shape = tuple(self.input.shape[2:])
        if len(self.input.shape) != 4 or any(
            not isinstance(size, int) for size in self.input_shape
        ):
            raise ValueError('YOLO model must have static NCHW input dimensions')
        metadata = self.session.get_modelmeta().custom_metadata_map
        names = ast.literal_eval(metadata.get('names', '{}'))
        if isinstance(names, list):
            names = dict(enumerate(names))
        self.names = {int(key): value for key, value in names.items()}
        if set(self.names) != set(range(len(self.names))) or not self.names:
            raise ValueError('YOLO model requires contiguous class-name metadata')
        if metadata.get('task') != 'segment' or not {'table', 'box'} <= set(self.names.values()):
            raise ValueError(
                'Use a segmentation model with table and box classes; COCO weights lack box'
            )
        self.confidence, self.iou = confidence, iou
        self.deduplicate = deduplicate

    def predict(self, rgb):
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError('YOLO input must be RGB uint8')
        ih, iw = self.input_shape
        height, width = rgb.shape[:2]
        ratio = min(iw / width, ih / height)
        resized_w, resized_h = round(width * ratio), round(height * ratio)
        left, top = round((iw - resized_w) / 2 - 0.1), round((ih - resized_h) / 2 - 0.1)
        resized = cv2.resize(rgb, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        padded = np.full((ih, iw, 3), 114, dtype=np.uint8)
        padded[top : top + resized_h, left : left + resized_w] = resized
        tensor = np.ascontiguousarray(padded.transpose(2, 0, 1)[None], dtype=np.float32) / 255
        outputs = self.session.run(None, {self.input.name: tensor})
        if len(outputs) != 2 or outputs[1].ndim != 4:
            raise ValueError('Expected YOLO11-seg detection and prototype outputs')
        instances = decode_outputs(
            outputs[0],
            outputs[1],
            self.names,
            (height, width),
            self.input_shape,
            self.confidence,
            self.iou,
        )
        return suppress_contained_masks(instances) if self.deduplicate else instances


def overlay(rgb, instances):
    result = rgb.copy()
    colors = {
        'table': (0, 220, 0),
        'box': (255, 0, 0),
        'cylinder': (255, 180, 0),
        'sphere': (50, 100, 255),
    }
    for instance in instances:
        color = np.array(colors.get(instance.label, (200, 200, 200)), dtype=np.uint8)
        result[instance.mask] = (0.55 * result[instance.mask] + 0.45 * color).astype(np.uint8)
        x0, y0, x1, y1 = instance.box_xyxy
        cv2.rectangle(result, (x0, y0), (x1, y1), tuple(int(x) for x in color), 1)
        cv2.putText(
            result,
            f'{instance.label} {instance.confidence:.2f}',
            (x0, max(12, y0 - 3)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            tuple(int(x) for x in color),
            1,
        )
    return result
