"""Run configured cases/repetitions and persist per-trial logs and aggregate metrics."""

import argparse
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.config import ROOT
from g1_backend.evaluation import run_benchmark, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', default=str(ROOT / 'configs/benchmarks/locomotion.yaml'))
    parser.add_argument('--config', help='Override the base experiment YAML')
    parser.add_argument('--repetitions', type=int)
    parser.add_argument('--output', help='Also write summary JSON to this path')
    args = parser.parse_args()
    specification = yaml.safe_load(Path(args.matrix).read_text())
    if args.config:
        specification['experiment'] = args.config
    if args.repetitions is not None:
        specification['repetitions'] = args.repetitions
    summary = run_benchmark(specification)
    if args.output:
        write_json(args.output, summary)
    print(f'Summary: {summary["results_dir"]}/summary.json')
    return 0 if summary['aggregate']['success_rate'] == 1 else 2


if __name__ == '__main__':
    raise SystemExit(main())
