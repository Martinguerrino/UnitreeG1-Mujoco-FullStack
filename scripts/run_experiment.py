"""Run the independent DDS backend from a reproducible YAML configuration."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend import run_experiment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument(
        '--report', type=Path, help='Write machine-readable result separately from SDK logs'
    )
    args = parser.parse_args()
    result = run_experiment(args.config)
    if args.report:
        from g1_backend.evaluation import write_json

        write_json(args.report, result)
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'completed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
