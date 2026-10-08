"""Render the actual Maxwell scene across distances, bearings and sensor noise."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile

# Configure software EGL before MuJoCo is imported, matching the renderer check.
from check_perception_render import initialize_renderer
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.config import ROOT, load_config
from g1_backend.perception import Settings, backproject, detect, transform_points
from g1_backend.perception_evaluation import aggregate, score, stability
from g1_backend.perception_tracking import Tracker
from g1_backend.perception_rgbd import detect_instances, validated_instances
from g1_backend.perception_views import ViewAccumulator
from g1_backend.perception_yolo import YoloSegmenter, overlay
from g1_backend.sensor_effects import SensorEffects
from g1_backend.simulator import model_xml


def reference(model, data, geom_id, name, surface=False):
    rotation = data.geom_xmat[geom_id].reshape(3, 3).copy()
    center = data.geom_xpos[geom_id].copy()
    dimensions = model.geom_size[geom_id].copy() * 2
    if surface:
        center += rotation[:, 2] * dimensions[2] / 2
        dimensions[2] = 0
    if dimensions[1] > dimensions[0]:
        dimensions[[0, 1]] = dimensions[[1, 0]]
        rotation = rotation @ Rotation.from_euler('z', np.pi / 2).as_matrix()
    return {
        'id': name,
        'center': center.tolist(),
        'dimensions': dimensions.tolist(),
        'orientation_xyzw': Rotation.from_matrix(rotation).as_quat().tolist(),
    }


def visible_reference(model, data, geom_id, segmentation):
    # Segmentation is used exclusively by this evaluator to distinguish an
    # object outside the field of view from a visible object the detector misses.
    return int(
        np.count_nonzero(
            (segmentation[:, :, 0] == geom_id)
            & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
        )
    )


def evaluate_view_sequence(cases, output, segmenter, settings):
    """Replay independent held-out Maxwell views through one tracker per noise profile."""
    if segmenter is None:
        return {}
    sequences = {}
    closest = min(case['distance_m'] for case in cases)
    for noise in ('ideal', 'real_profile'):
        chosen = sorted(
            [case for case in cases if case['distance_m'] == closest and case['noise'] == noise],
            key=lambda case: case['bearing_deg'],
        )
        tracker, accumulator = Tracker(), ViewAccumulator()
        records = []
        for index, case in enumerate(chosen):
            path = output / f'd{closest}_a{case["bearing_deg"]}_{noise}.npz'
            with np.load(path) as snapshot:
                rgb, depth, k = snapshot['d435i_rgb'], snapshot['d435i_depth'], snapshot['k']
                camera = snapshot['camera_to_processing']
            instances = segmenter.predict(rgb)
            raw = detect_instances(depth, rgb, k, instances, camera, settings=settings)
            tracked = tracker.update(raw, index * 0.5)
            filtered = score(tracked, case['references']['tables'], 0.6)
            fused = accumulator.update(tracked, camera, index * 0.5)
            boxes = [
                obj
                for table in fused
                for obj in table['objects']
                if obj['kind'] == 'box_candidate'
            ]
            records.append(
                {
                    'bearing_deg': case['bearing_deg'],
                    'filtered_tables': filtered,
                    'fused_tables': score(fused, case['references']['tables'], 0.6),
                    'fused_boxes': score(boxes, case['references']['boxes']),
                    'views': [
                        {'id': item['id'], 'quality': item['quality']}
                        for table in fused
                        for item in [table, *table['objects']]
                    ],
                }
            )
        sequences[noise] = {
            'frames': records,
            'filtered_tables': aggregate([row['filtered_tables'] for row in records]),
            'fused_tables': aggregate([row['fused_tables'] for row in records]),
        }
    return sequences


def run(args):
    initialize_renderer()
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['camera']['width'], config['camera']['height'] = args.width, args.height
    sources = model_xml(config)
    deployment = yaml.safe_load((ROOT / config['locomotion']['deployment']).read_text())
    noise_profile = yaml.safe_load((ROOT / 'configs/navigation/sensors_real.yaml').read_text())
    settings = Settings(
        **{
            key: value
            for key, value in yaml.safe_load((ROOT / 'configs/perception.yaml').read_text())[
                'g1_perception'
            ]['ros__parameters'].items()
            if key in Settings.__dataclass_fields__
        }
    )
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    segmenter = YoloSegmenter(args.model, args.confidence) if args.backend == 'yolo' else None
    with tempfile.TemporaryDirectory(prefix='g1_perception_eval_') as directory:
        path = Path(directory)
        (path / 'g1_29dof.xml').write_text(sources[0])
        (path / 'scene.xml').write_text(sources[1])
        model = mujoco.MjModel.from_xml_path(str(path / 'scene.xml'))
    data = mujoco.MjData(model)
    for actuator, joint_position in zip(
        deployment['joint_ids_map'], deployment['default_joint_pos'], strict=True
    ):
        joint_id = model.actuator_trnid[actuator, 0]
        data.qpos[model.jnt_qposadr[joint_id]] = joint_position
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435i')
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
    root_joint = model.body_jntadr[pelvis]
    root_address = model.jnt_qposadr[root_joint]
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'desk_top')
    box_ids = {}
    for name in ('red_cube', 'yellow_box'):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        box_ids[name] = int(model.body_geomadr[body_id])
    focal = args.height / (2 * np.tan(np.deg2rad(model.cam_fovy[camera_id]) / 2))
    k = [focal, 0, (args.width - 1) / 2, 0, focal, (args.height - 1) / 2, 0, 0, 1]
    cases = []
    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        for distance in args.distances:
            for bearing in args.bearings:
                theta = np.deg2rad(bearing)
                position = [4.5 + distance * np.sin(theta), -5.0 - distance * np.cos(theta), 0.793]
                yaw = np.pi / 2 + theta
                data.qpos[root_address : root_address + 3] = position
                data.qpos[root_address + 3 : root_address + 7] = [
                    np.cos(yaw / 2),
                    0,
                    0,
                    np.sin(yaw / 2),
                ]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera_id)
                renderer.disable_depth_rendering()
                rgb = renderer.render().copy()
                renderer.enable_depth_rendering()
                depth = renderer.render().copy()
                renderer.disable_depth_rendering()
                renderer.enable_segmentation_rendering()
                segmentation = renderer.render().copy()
                renderer.disable_segmentation_rendering()
                extrinsic = np.eye(4)
                extrinsic[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag(
                    [1.0, -1.0, -1.0]
                )
                extrinsic[:3, 3] = data.cam_xpos[camera_id]
                table_truth = [reference(model, data, table_id, 'desk_top', True)]
                box_truth = [reference(model, data, gid, name) for name, gid in box_ids.items()]
                visibility = {
                    name: visible_reference(model, data, gid, segmentation)
                    for name, gid in {'desk_top': table_id, **box_ids}.items()
                }
                for noise in ('ideal', 'real_profile'):
                    tracker = Tracker()
                    views = ViewAccumulator()
                    effects = SensorEffects(noise_profile, args.seed)
                    frames = []
                    for index in range(args.frames):
                        sensors = {'d435i_rgb': rgb.copy(), 'd435i_depth': depth.copy()}
                        if noise == 'real_profile':
                            sensors = effects.camera(sensors)
                        if segmenter is None:
                            points, colors = backproject(
                                sensors['d435i_depth'], sensors['d435i_rgb'], k, settings
                            )
                            raw = detect(transform_points(points, extrinsic), colors, settings)
                        else:
                            instances = segmenter.predict(sensors['d435i_rgb'])
                            raw = detect_instances(
                                sensors['d435i_depth'],
                                sensors['d435i_rgb'],
                                k,
                                instances,
                                extrinsic,
                                settings=settings,
                            )
                            validated = validated_instances(
                                sensors['d435i_depth'],
                                sensors['d435i_rgb'],
                                k,
                                instances,
                                raw,
                                extrinsic,
                                settings,
                            )
                        tracked = views.update(
                            tracker.update(raw, index * 0.5), extrinsic, index * 0.5
                        )
                        raw_boxes = [
                            obj
                            for table in raw
                            for obj in table['objects']
                            if obj['kind'] == 'box_candidate'
                        ]
                        boxes = [
                            obj
                            for table in tracked
                            for obj in table['objects']
                            if obj['kind'] == 'box_candidate'
                        ]
                        frames.append(
                            {
                                'frame': index,
                                'raw_tables': score(raw, table_truth, 0.6),
                                'raw_boxes': score(raw_boxes, box_truth),
                                'filtered_tables': score(tracked, table_truth, 0.6),
                                'filtered_boxes': score(boxes, box_truth),
                                'visible_boxes': score(
                                    boxes,
                                    [ref for ref in box_truth if visibility[ref['id']] >= 20],
                                ),
                                'track_ids': [obj['id'] for obj in boxes],
                            }
                        )
                    case = {
                        'distance_m': distance,
                        'bearing_deg': bearing,
                        'noise': noise,
                        'visible_pixels': visibility,
                        'references': {'tables': table_truth, 'boxes': box_truth},
                        'frames': frames,
                        'box_stability': stability(frames),
                        'table_stability': stability(frames, 'raw_tables', 'filtered_tables'),
                    }
                    cases.append(case)
                    print(
                        f'{distance:.2f} m {bearing:+.0f} deg {noise}: '
                        f'{len(raw)} surfaces, {len(boxes)} boxes',
                        flush=True,
                    )
                    np.savez_compressed(
                        output / f'd{distance}_a{bearing}_{noise}.npz',
                        **sensors,
                        k=k,
                        camera_to_processing=extrinsic,
                    )
                    if segmenter is not None:
                        import cv2

                        cv2.imwrite(
                            str(output / f'd{distance}_a{bearing}_{noise}_overlay.png'),
                            cv2.cvtColor(
                                overlay(sensors['d435i_rgb'], validated), cv2.COLOR_RGB2BGR
                            ),
                        )
                        cv2.imwrite(
                            str(output / f'd{distance}_a{bearing}_{noise}_raw_overlay.png'),
                            cv2.cvtColor(
                                overlay(sensors['d435i_rgb'], instances), cv2.COLOR_RGB2BGR
                            ),
                        )
    summary = {
        key: aggregate([frame[key] for case in cases for frame in case['frames']])
        for key in (
            'raw_tables',
            'raw_boxes',
            'filtered_tables',
            'filtered_boxes',
            'visible_boxes',
        )
    }
    scene_path = ROOT / config['robot']['scene']
    report = {
        'scene': str(scene_path.relative_to(ROOT)),
        'scene_sha256': hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        'seed': args.seed,
        'resolution': [args.width, args.height],
        'noise_profile': noise_profile,
        'sensor': 'robot-mounted d435i',
        'backend': args.backend,
        'model': args.model if segmenter is not None else None,
        'cases': cases,
        'summary': summary,
        'settings': vars(settings),
        'view_sequences': evaluate_view_sequence(cases, output, segmenter, settings),
        'limits': [
            'Static camera poses; no locomotion or physical grasp evaluated.',
            'Image noise only; delivery latency and IMU effects are not evaluated.',
            'Ground truth and segmentation are evaluator-only inputs.',
            'Dimension errors compare visible estimates with full reference dimensions.',
            'Orientation is scored modulo the rectangle symmetry.',
        ],
    }
    (output / 'evaluation.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(summary, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['geometry', 'yolo'], default='yolo')
    parser.add_argument('--model', default='artifacts/perception/g1_objects_yolo11n_seg.onnx')
    parser.add_argument('--confidence', type=float, default=0.3)
    parser.add_argument('--output', default='results/perception/maxwell_evaluation')
    parser.add_argument('--distances', nargs='+', type=float, default=[0.6, 1.0, 1.5])
    parser.add_argument('--bearings', nargs='+', type=float, default=[-20.0, 0.0, 20.0])
    parser.add_argument('--frames', type=int, default=5)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--width', type=int, default=424)
    parser.add_argument('--height', type=int, default=240)
    args = parser.parse_args()
    if args.frames < 1 or min(args.distances) <= 0 or min(args.width, args.height) < 1:
        parser.error('frames, distances and resolution must be positive')
    run(args)


if __name__ == '__main__':
    main()
