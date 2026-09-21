"""Export recorded RGB observations as MP4 without rerunning physics or policies."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.recording import encode_video


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--view', help='Named camera view, for example head_camera')
    args = parser.parse_args()
    print(json.dumps(encode_video(args.experiment, args.output, args.fps, args.view), indent=2))


if __name__ == '__main__':
    main()
