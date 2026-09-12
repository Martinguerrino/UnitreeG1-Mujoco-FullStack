"""Download a pinned checkpoint and validate its tensor contract without PyTorch."""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import urllib.request


REPO_ID = 'sotata/act-okura-pick-06102026'
REVISION = '220fc42eeff7585136ef29cf2bd5a82de8239092'
WEIGHTS_SHA256 = 'e15b0dfdbadfc93fa12d20caee0f69d2bc39b32be1e52ed5749326f1648708fc'
DEFAULT_CHECKPOINT = Path(__file__).resolve().parents[2] / 'artifacts/okura/checkpoint'
IMAGE_KEY = 'observation.images.cam_left_high'
FILES = (
    'config.json',
    'model.safetensors',
    'policy_preprocessor.json',
    'policy_postprocessor.json',
    'policy_preprocessor_step_3_normalizer_processor.safetensors',
    'policy_postprocessor_step_0_unnormalizer_processor.safetensors',
    'train_config.json',
    'README.md',
)
ARM_SUFFIXES = (
    'shoulder_pitch',
    'shoulder_roll',
    'shoulder_yaw',
    'elbow',
    'wrist_roll',
    'wrist_pitch',
    'wrist_yaw',
)
STATE_NAMES = tuple(
    f'{side}_{suffix}_joint' for side in ('left', 'right') for suffix in ARM_SUFFIXES
) + ('left_gripper', 'right_gripper')


def tensor_header(path: Path) -> dict:
    """Read bounded safetensors metadata, rejecting incomplete downloads."""
    with path.open('rb') as stream:
        prefix = stream.read(8)
        if len(prefix) != 8:
            raise ValueError(f'Incomplete safetensors: {path}')
        length = struct.unpack('<Q', prefix)[0]
        if not 2 <= length <= 16 * 1024 * 1024:
            raise ValueError(f'Invalid safetensors header size: {length}')
        header = json.loads(stream.read(length))
    payload_size = path.stat().st_size - 8 - length
    for name, tensor in header.items():
        if name != '__metadata__':
            start, end = tensor['data_offsets']
            if not 0 <= start <= end <= payload_size:
                raise ValueError(f'Truncated tensor {name} in {path}')
    return header


def validate_checkpoint(path: Path) -> dict:
    config = json.loads((path / 'config.json').read_text())
    if config['type'] != 'act':
        raise ValueError('Expected an ACT checkpoint')
    expected = {'observation.state': [16], IMAGE_KEY: [3, 480, 640]}
    if {k: v['shape'] for k, v in config['input_features'].items()} != expected:
        raise ValueError('Unexpected observation layout; refusing implicit remapping')
    if config['output_features']['action']['shape'] != [16]:
        raise ValueError('Expected 16 actions')
    weights = tensor_header(path / 'model.safetensors')
    for key, shape in {
        'model.action_head.weight': [16, 512],
        'model.encoder_robot_state_input_proj.weight': [512, 16],
    }.items():
        if weights[key]['shape'] != shape:
            raise ValueError(f'Weights disagree with configuration: {key}')
    for filename in FILES:
        if not (path / filename).is_file():
            raise FileNotFoundError(path / filename)
        if filename.endswith('processor.safetensors'):
            header = tensor_header(path / filename)
            for feature in ('action', 'observation.state'):
                for statistic in ('mean', 'std'):
                    if header[f'{feature}.{statistic}']['shape'] != [16]:
                        raise ValueError(f'Wrong normalization shape: {filename}')
    with (path / 'model.safetensors').open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if digest != WEIGHTS_SHA256:
        raise ValueError('Weights do not match the pinned Okura checkpoint')
    return {
        'repo_id': REPO_ID,
        'revision_expected': REVISION,
        'weights_sha256': digest,
        'state_names_from_model_card': STATE_NAMES,
        'chunk_size': config['chunk_size'],
        'checkpoint_contract_valid': True,
        'physical_grasp_tested': False,
        'warning': 'Public dataset currently declares 8 dimensions; this checkpoint declares 16. '
        'Do not use that dataset for replay or calibration without reconciliation.',
    }


def download(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for filename in FILES:
        destination = path / filename
        if destination.exists():
            continue
        url = f'https://huggingface.co/{REPO_ID}/resolve/{REVISION}/{filename}'
        temporary = destination.with_suffix(destination.suffix + '.partial')
        print(f'Downloading {filename}', flush=True)
        with urllib.request.urlopen(url, timeout=120) as response, temporary.open('wb') as output:
            while block := response.read(1024 * 1024):
                output.write(block)
        temporary.replace(destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument('--download', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.download:
        download(args.checkpoint)
    report = json.dumps(validate_checkpoint(args.checkpoint), indent=2)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report + '\n')
    print(report)


if __name__ == '__main__':
    main()
