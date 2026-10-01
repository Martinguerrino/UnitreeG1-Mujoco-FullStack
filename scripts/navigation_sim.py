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
    args = parser.parse_args()
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['robot']['dexterity'] = args.dexterity
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
