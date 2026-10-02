"""Start the DDS simulator in its dedicated Conda interpreter."""

import argparse
import signal
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.config import ROOT, load_config
from g1_backend.runner import run_experiment


def request_shutdown(signum, frame):
    # ROS launch and the terminal can both send SIGINT. Let the first one
    # stop the experiment; further signals must not interrupt child cleanup.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


if __name__ == '__main__':
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', choices=['true', 'false'], default='true')
    parser.add_argument('--ideal', choices=['true', 'false'], default='false')
    parser.add_argument('--duration', type=float, default=3600.0)
    parser.add_argument('--dexterity', choices=['claw', 'hand'], default='hand')
    parser.add_argument('--camera-fps', type=float, default=10.0)
    parser.add_argument('--external-turn-assist', choices=['true', 'false'], default='false')
    parser.add_argument('--initial-x', type=float, default=0.0)
    parser.add_argument('--initial-y', type=float, default=0.0)
    parser.add_argument('--initial-yaw', type=float, default=0.0)
    parser.add_argument('--scene', default='')
    args = parser.parse_args()
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['robot']['dexterity'] = args.dexterity
    config['robot']['initial_position'][:2] = [args.initial_x, args.initial_y]
    config['robot']['initial_yaw'] = args.initial_yaw
    if args.scene:
        scene = Path(args.scene).expanduser().resolve()
        if not scene.is_file():
            parser.error('--scene must name an existing MuJoCo XML scene')
        config['robot']['scene'] = str(scene)
    if args.external_turn_assist == 'true':
        # ROS applies assistance before collision monitoring. The final command
        # must reach the policy unchanged, including an emergency zero.
        config['navigation']['turn_assist']['enabled'] = False
    if args.camera_fps <= 0:
        parser.error('--camera-fps must be positive')
    config['camera']['fps'] = args.camera_fps
    if args.ideal == 'true':
        # Original lightweight navigation setup: ideal planar scan only.
        config['camera']['enabled'] = False
        config['camera']['record'] = False
        config['navigation']['lidar']['enabled'] = False
    config['viewer'] = {'enabled': args.viewer == 'true'}
    config['duration'] = args.duration
    report = run_experiment(config)
    print(report)
    sys.exit(0 if report['status'] == 'completed' else 2)
