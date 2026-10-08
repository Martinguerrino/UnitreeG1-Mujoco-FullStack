#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/python_env.sh"
TASK_PYTHON="${1:-$G1_PYTHON}"
"$TASK_PYTHON" -c 'import sys,mujoco,numpy,scipy,yaml,cv2; assert sys.version_info[:2] == (3,12); assert mujoco.__version__ == "3.3.6"; print("Python/MuJoCo:",sys.executable,mujoco.__version__)'
"$TASK_PYTHON" - "$ROOT_DIR" <<'PY'
import sys, os, json, hashlib
from pathlib import Path
root=Path(sys.argv[1])
sys.path.insert(0,str(root))
from g1_backend.config import runtime_path
runtime_path()
import cyclonedds, unitree_sdk2py, onnxruntime
from g1_backend.perception_yolo import YoloSegmenter
YoloSegmenter(root/'artifacts/perception/g1_objects_yolo11n_seg.onnx')
assert any((root/'artifacts/dex1').glob('*.STL')), 'Faltan assets Dex1'
lock=json.loads((root/'configs/sources.lock.json').read_text())
for name,entry in lock['files'].items():
    path=root/name
    assert path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256'], f'Asset DDS faltante o alterado: {name}'
if os.environ.get('G1_REQUIRE_EXPERIMENTS')=='true':
    import torch, torchvision, lerobot
    from experiments.okura.checkpoint import validate_checkpoint
    validate_checkpoint(root/'artifacts/okura/checkpoint')
print('DDS, Dex1 y modelo YOLO: listos')
PY
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/jazzy/setup.bash
[[ ! -f "$ROOT_DIR/install/setup.bash" ]] || source "$ROOT_DIR/install/setup.bash"
[[ ! -d "$ROOT_DIR/artifacts/slam" ]] || source "$ROOT_DIR/scripts/slam_env.sh"
set -u
for package in nav2_bringup slam_toolbox rviz2 moveit_ros_move_group mujoco_ros2_control; do
  ros2 pkg prefix "$package" >/dev/null
done
/usr/bin/python3 -c 'import rclpy,cv2,scipy; print("ROS y RGB-D: listos")'
if [[ "${G1_REQUIRE_SLAM3D:-false}" == true ]]; then
  for package in fast_lio rtabmap_slam rtabmap_odom; do ros2 pkg prefix "$package" >/dev/null; done
  "$ROOT_DIR/artifacts/slam/tsdf_venv/bin/python" -c 'import open3d'
  [[ -x "$ROOT_DIR/artifacts/slam/bin/g1_cloud_mesh" ]]
fi
if [[ "${G1_REQUIRE_BUILD:-false}" == true ]]; then
  for package in g1_bringup g1_manipulation g1_mujoco; do ros2 pkg prefix "$package" >/dev/null; done
fi
