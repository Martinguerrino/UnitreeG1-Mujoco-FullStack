"""Run original ACT weights offline. Produces actions, never sends motor commands."""

import argparse
import json
from pathlib import Path
import time

import numpy as np

from .checkpoint import DEFAULT_CHECKPOINT, IMAGE_KEY, validate_checkpoint


class OkuraPolicy:
    def __init__(self, checkpoint: Path, threads: int = 4):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.processor import PolicyProcessorPipeline
        from lerobot.processor.converters import (
            policy_action_to_transition,
            transition_to_policy_action,
        )
        from safetensors.torch import load_file

        self.contract = validate_checkpoint(checkpoint)
        torch.set_num_threads(threads)
        torch.manual_seed(0)
        config = PreTrainedConfig.from_pretrained(str(checkpoint), local_files_only=True)
        config.device = 'cpu'
        # All backbone weights are in model.safetensors. Avoid downloading ImageNet weights.
        config.pretrained_backbone_weights = None
        self.policy = ACTPolicy(config)
        self.policy.load_state_dict(load_file(str(checkpoint / 'model.safetensors')), strict=True)
        self.policy.eval()
        self.pre = PolicyProcessorPipeline.from_pretrained(
            str(checkpoint),
            config_filename='policy_preprocessor.json',
            overrides={'device_processor': {'device': 'cpu'}},
            local_files_only=True,
        )
        self.post = PolicyProcessorPipeline.from_pretrained(
            str(checkpoint),
            config_filename='policy_postprocessor.json',
            to_transition=policy_action_to_transition,
            to_output=transition_to_policy_action,
            local_files_only=True,
        )

    def chunk(self, state: np.ndarray, image: np.ndarray) -> np.ndarray:
        import torch

        if state.shape != (16,) or not np.isfinite(state).all():
            raise ValueError('State must contain 16 finite values in checkpoint order')
        if image.shape != (480, 640, 3) or image.dtype != np.uint8:
            raise ValueError('Camera must be RGB uint8 HWC (480,640,3)')
        self.policy.reset()
        with torch.inference_mode():
            observation = self.pre(
                {
                    'observation.state': torch.from_numpy(state.astype(np.float32)),
                    IMAGE_KEY: torch.from_numpy(image.copy()).permute(2, 0, 1).float() / 255,
                }
            )
            normalized = self.policy.predict_action_chunk(observation)
            # Keep each output in the official postprocessor's (batch, action) layout.
            actions = (
                torch.stack(
                    [self.post(normalized[:, index])[0] for index in range(normalized.shape[1])]
                )
                .cpu()
                .numpy()
            )
        if actions.shape != (100, 16) or not np.isfinite(actions).all():
            raise ValueError('Invalid action chunk')
        return actions


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument('--output', type=Path, default=DEFAULT_CHECKPOINT.parent / 'inference')
    parser.add_argument(
        '--observation', type=Path, help='NPZ with state(16) and RGB image(480,640,3)'
    )
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.threads <= 64:
        parser.error('--threads must be between 1 and 64')
    args.output.mkdir(parents=True, exist_ok=True)
    # Invalidate any previous successful report before a new attempt starts.
    (args.output / 'report.json').write_text(
        json.dumps(
            {
                'inference_passed': False,
                'status': 'started; incomplete until replaced',
                'physical_grasp_tested': False,
                'box_transport_tested': False,
            },
            indent=2,
        )
        + '\n'
    )
    policy = OkuraPolicy(args.checkpoint, args.threads)
    if args.observation:
        with np.load(args.observation, allow_pickle=False) as observation:
            state, image = observation['state'], observation['image']
        observation_source = str(args.observation)
    else:
        from safetensors.numpy import load_file
        from .scene import make_scene, render_head, set_initial_arm_pose

        stats = load_file(
            str(args.checkpoint / 'policy_preprocessor_step_3_normalizer_processor.safetensors')
        )
        state = stats['observation.state.mean'].copy()
        model, data = make_scene()
        set_initial_arm_pose(model, data, state)
        image = render_head(model, data)
        observation_source = (
            'MuJoCo visual fixture; rigid hands, hypothetical gripper state, uncalibrated camera'
        )
    from PIL import Image

    Image.fromarray(image).save(args.output / 'head_camera.png')
    np.savez_compressed(args.output / 'observation.npz', state=state, image=image)
    start = time.perf_counter()
    actions = policy.chunk(state, image)
    elapsed = time.perf_counter() - start
    np.save(args.output / 'actions.npy', actions)
    report = {
        **policy.contract,
        'inference_passed': True,
        'action_shape': list(actions.shape),
        'inference_seconds': elapsed,
        'device': 'cpu',
        'observation_source': observation_source,
        'first_action': actions[0].tolist(),
        'motor_commands_sent': False,
        'physical_grasp_tested': False,
        'box_transport_tested': False,
        'note': 'Inference is not grasp validation. No Dex1 geometry or camera calibration is verified yet.',
    }
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
