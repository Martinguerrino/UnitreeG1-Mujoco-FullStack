"""Report actual optional backend capabilities; no dependency/model downloads."""

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.acceleration import enabled, cupy, torch_device, preload_libraries


def capabilities():
    report = {
        'enabled': enabled(),
        'torch_device': torch_device(),
        'cupy': False,
        'onnx_cuda': False,
        'open3d_cuda': False,
        'opencv_cuda': False,
    }
    cp = cupy()
    if cp is not None:
        try:
            report['cupy'] = int(cp.arange(4).sum().get()) == 6
            report['device'] = cp.cuda.runtime.getDeviceProperties(0)['name'].decode()
        except Exception as error:  # noqa: BLE001 - diagnostics report optional failures
            report['cupy_error'] = str(error)
    try:
        import onnxruntime as ort

        preload_libraries()
        report['onnx_cuda'] = 'CUDAExecutionProvider' in ort.get_available_providers()
        report['onnx_note'] = 'Listed provider; model initialization must still succeed'
    except ImportError:
        pass
    try:
        import open3d as o3d

        report['open3d_cuda'] = o3d.core.cuda.is_available()
    except ImportError:
        pass
    try:
        import cv2

        report['opencv_cuda'] = cv2.cuda.getCudaEnabledDeviceCount() > 0
    except (ImportError, AttributeError):
        pass
    return report


if __name__ == '__main__':
    print(json.dumps(capabilities(), indent=2))
