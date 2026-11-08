"""Resident ONNX input/output buffers and CUDA mask reconstruction.

Input resize and NMS preserve the OpenCV contract. Prototypes, mask products,
mask resizing and thresholding stay on device; only selected masks leave it.
"""

import cv2
import numpy as np


class YoloPipeline:
    def __init__(self, segmenter, cp):
        self.segmenter, self.cp = segmenter, cp
        height, width = segmenter.input_shape
        self.image = cp.empty((height, width, 3), dtype=cp.uint8)
        self.input = cp.empty((1, 3, height, width), dtype=cp.float32)
        self.outputs = []
        for output in segmenter.session.get_outputs():
            if output.type != 'tensor(float)' or not all(
                isinstance(size, int) and size > 0 for size in output.shape
            ):
                raise RuntimeError('Resident YOLO buffers require static float32 outputs')
            self.outputs.append(cp.empty(output.shape, dtype=cp.float32))
        self.binding = segmenter.session.io_binding()
        self.binding.bind_input(
            segmenter.input.name, 'cuda', 0, np.float32, self.input.shape, self.input.data.ptr
        )
        for meta, array in zip(segmenter.session.get_outputs(), self.outputs):
            self.binding.bind_output(meta.name, 'cuda', 0, np.float32, array.shape, array.data.ptr)

    def predict(self, rgb):
        from ..perception_yolo import Instance

        cp, s = self.cp, self.segmenter
        ih, iw = s.input_shape
        height, width = rgb.shape[:2]
        ratio = min(iw / width, ih / height)
        rw, rh = round(width * ratio), round(height * ratio)
        left, top = round((iw - rw) / 2 - 0.1), round((ih - rh) / 2 - 0.1)
        # Match OpenCV INTER_LINEAR exactly; no change in the model input contract.
        resized = cv2.resize(rgb, (rw, rh), interpolation=cv2.INTER_LINEAR)
        self.image.fill(114)
        self.image[top : top + rh, left : left + rw] = cp.asarray(resized)
        self.input[0] = self.image.transpose(2, 0, 1).astype(cp.float32) / 255
        cp.cuda.get_current_stream().synchronize()
        s.session.session.run_with_iobinding(self.binding)
        self.binding.synchronize_outputs()
        prediction, proto = self.outputs
        if len(self.outputs) != 2 or proto.ndim != 4:
            raise ValueError('Expected YOLO11-seg detection and prototype outputs')
        nc, coefficients = len(s.names), proto.shape[1]
        values = prediction[0].T
        if values.shape[1] != 4 + nc + coefficients:
            raise ValueError('Unexpected YOLO segmentation output shape')
        scores = values[:, 4 : 4 + nc]
        labels = scores.argmax(axis=1)
        confidence = scores.max(axis=1)
        keep = cp.flatnonzero(confidence >= s.confidence)
        if not len(keep):
            return []
        values, labels, confidence = values[keep], labels[keep], confidence[keep]
        boxes = cp.asnumpy(values[:, :4])
        boxes[:, :2] -= boxes[:, 2:] / 2
        confidence, labels = cp.asnumpy(confidence), cp.asnumpy(labels)
        selected = np.asarray(
            cv2.dnn.NMSBoxes(boxes.tolist(), confidence.tolist(), s.confidence, s.iou, top_k=300)
        ).reshape(-1)[:30]
        if not len(selected):
            return []
        logits = values[cp.asarray(selected), 4 + nc :] @ proto[0].reshape(coefficients, -1)
        from .masks import resize_masks

        logits = logits.reshape(-1, *proto.shape[2:])
        bounds = []
        for index in selected:
            x, y, bw, bh = boxes[index]
            bounds.append(
                [
                    max(0, round(float((x - left) / ratio))),
                    max(0, round(float((y - top) / ratio))),
                    min(width, round(float((x + bw - left) / ratio))),
                    min(height, round(float((y + bh - top) / ratio))),
                ]
            )
        masks = resize_masks(cp, logits, bounds, (height, width))
        instances = []
        for index, mask, box in zip(selected, masks, bounds):
            if mask.any():
                instances.append(
                    Instance(s.names[int(labels[index])], float(confidence[index]), mask, box)
                )
        return instances
