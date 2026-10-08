#!/usr/bin/env python3
"""Install the bundled trained ONNX model, verifying its SHA-256."""

import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def install_model(root=ROOT):
    source = Path(root) / 'assets/perception'
    manifest = json.loads((source / 'model_manifest.json').read_text())
    model = source / 'g1_objects_yolo11n_seg.onnx'
    if hashlib.sha256(model.read_bytes()).hexdigest() != manifest['onnx_sha256']:
        raise ValueError('El modelo YOLO incluido no coincide con su SHA-256')
    destination = Path(root) / 'artifacts/perception'
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(model, destination / model.name)
    shutil.copy2(source / 'model_manifest.json', destination / 'model_manifest.json')
    print(f'Modelo YOLO listo: {destination / model.name}')


if __name__ == '__main__':
    install_model()
