"""Start the DDS simulator in its dedicated Conda interpreter."""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.config import ROOT, load_config
from g1_backend.runner import run_experiment


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', choices=['true', 'false'], default='true')
    parser.add_argument('--duration', type=float, default=3600.0)
    args = parser.parse_args()
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['viewer'] = {'enabled': args.viewer == 'true'}
    config['duration'] = args.duration
    report = run_experiment(config)
    print(report)
    sys.exit(0 if report['status'] == 'completed' else 2)
