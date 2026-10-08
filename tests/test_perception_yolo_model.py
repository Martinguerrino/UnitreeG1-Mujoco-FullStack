"""Optional installed-model parity check; never download weights during tests."""

import importlib.util
import sys

import numpy as np
import pytest

from g1_backend.perception_yolo import ROOT, YoloSegmenter

CHECKPOINT = ROOT / 'artifacts/perception/g1_objects_yolo11n_seg.pt'
ONNX = CHECKPOINT.with_suffix('.onnx')
SNAPSHOT = ROOT / 'results/perception/maxwell_evaluation/d0.6_a0.0_ideal.npz'


@pytest.mark.skipif(
    not (ONNX.is_file() and SNAPSHOT.is_file()),
    reason='Installed YOLO model and Maxwell RGB-D required',
)
def test_onnx_matches_native_yolo_classes_scores_and_masks(monkeypatch):
    monkeypatch.setenv('YOLO_CONFIG_DIR', str(ROOT / 'artifacts/perception/ultralytics_config'))
    monkeypatch.setenv('YOLO_AUTOINSTALL', 'false')
    sys.path.append(str(ROOT / 'artifacts/perception/python'))
    if importlib.util.find_spec('torch') is None:
        pytest.skip('Native-model parity uses the Conda PyTorch interpreter')
    import torch

    torch.set_num_threads(2)
    from ultralytics import YOLO

    with np.load(SNAPSHOT) as data:
        rgb = data['d435i_rgb']
    predictions = YoloSegmenter(ONNX, deduplicate=False).predict(rgb)
    native = YOLO(str(CHECKPOINT)).predict(
        rgb[:, :, ::-1].copy(),
        imgsz=640,
        rect=False,
        retina_masks=True,
        conf=0.3,
        iou=0.5,
        agnostic_nms=True,
        max_det=30,
        device='cpu',
        verbose=False,
    )[0]
    assert predictions and native.masks is not None
    assert len(predictions) == len(native.boxes)
    for predicted, label, score, mask in zip(
        predictions,
        native.boxes.cls.cpu().numpy(),
        native.boxes.conf.cpu().numpy(),
        native.masks.data.cpu().numpy(),
        strict=True,
    ):
        assert predicted.label == native.names[int(label)]
        assert predicted.confidence == pytest.approx(float(score), abs=0.015)
        expected = mask.astype(bool)
        iou = np.count_nonzero(predicted.mask & expected) / np.count_nonzero(
            predicted.mask | expected
        )
        assert iou > 0.90
