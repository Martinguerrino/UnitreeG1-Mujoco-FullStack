"""Run a recorded experiment and export synchronized head and observer MP4 files."""

import argparse
import json
from pathlib import Path
import re
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend import run_experiment
from g1_backend.recording import encode_video

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--name', required=True, help='Subfolder name under videos/')
    parser.add_argument('--fps', type=float, default=20)
    parser.add_argument('--viewer', action='store_true', help='Open the live MuJoCo viewer')
    parser.add_argument('--no-record', action='store_true', help='Do not record or export MP4')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.name):
        parser.error('--name must contain only letters, numbers, underscores and hyphens')

    config = yaml.safe_load(args.config.read_text())
    config['results_dir'] = f'videos/{args.name}'
    config['viewer'] = {'enabled': args.viewer}
    camera_source = config.get('camera')
    if isinstance(camera_source, str):
        config['camera'] = yaml.safe_load((ROOT / camera_source).read_text())
    if not args.no_record and not config['camera'].get('views'):
        raise ValueError('Video tests require a camera configuration with named views')
    config['camera']['enabled'] = not args.no_record
    config['camera']['record'] = not args.no_record

    result = run_experiment(config)
    experiment = Path(result['results_dir'])
    videos = (
        {}
        if args.no_record
        else {
            view: encode_video(experiment, experiment / f'{view}.mp4', args.fps, view)
            for view in ('head_camera', 'observer_camera')
        }
    )
    print(json.dumps({'experiment': result, 'videos': videos}, indent=2))
    return 0 if result['status'] == 'completed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
