"""Validate an ordered route through real Nav2 goals; stop at the first failure."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, action='append', required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    legs = []
    for index, (x, y, yaw) in enumerate(args.goal):
        output = args.output / f'goal_{index + 1}.json'
        result = subprocess.run([
            sys.executable, str(Path(__file__).with_name('check_navigation_goal.py')),
            '--x', str(x), '--y', str(y), '--yaw', str(yaw),
            '--timeout', str(args.timeout), '--output', str(output),
        ], check=False)
        legs.append({'goal': [x, y, yaw], 'returncode': result.returncode,
                     'report': json.loads(output.read_text()) if output.exists() else None})
        report = {'goals': legs, 'planned_goals': len(args.goal),
                  'passed': len(legs) == len(args.goal) and all(
                      leg['returncode'] == 0 and leg['report']['passed'] for leg in legs)}
        (args.output / 'route.json').write_text(json.dumps(report, indent=2))
        if result.returncode:
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
