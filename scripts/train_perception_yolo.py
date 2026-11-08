"""Prepare a synthetic segmentation dataset and train a four-class YOLO11n model."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.cuda.rendering import initialize_renderer
import cv2
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'artifacts/perception/python'))
CLASSES = {0: 'table', 1: 'box', 2: 'cylinder', 3: 'sphere'}


def make_scene(rng, include_boxes=True):
    height = rng.uniform(0.68, 0.9)
    sx, sy = rng.uniform(0.65, 1.05), rng.uniform(0.35, 0.65)
    background = rng.uniform(0.18, 0.85, 3)
    wood = rng.uniform(0.12, 0.75, 3)
    rgb = lambda values: ' '.join(map(str, values)) + ' 1'
    geoms = [
        (
            f'<geom name="table" type="box" pos="0 0 {height - 0.03}" '
            f'size="{sx} {sy} .03" rgba="{rgb(wood)}"/>'
        )
    ]
    objects = []
    occupied = []
    for index in range(int(rng.integers(2, 6))):
        kind = int(rng.integers(1 if include_boxes else 2, 4))
        extent = rng.uniform(0.025, 0.065)
        if kind == 1:
            dx, dy, dz = extent, rng.uniform(0.025, 0.065), rng.uniform(0.025, 0.07)
            if rng.random() < 0.4:
                dx = dy = dz = extent
            shape = f'type="box" size="{dx} {dy} {dz}"'
        elif kind == 2:
            dz = rng.uniform(0.025, 0.065)
            shape = f'type="cylinder" size="{extent} {dz}"'
        else:
            dz = extent
            shape = f'type="sphere" size="{extent}"'
        for _ in range(30):
            x, y = rng.uniform(-sx + 0.12, sx - 0.12), rng.uniform(-sy + 0.1, sy - 0.1)
            if all(np.hypot(x - a, y - b) > 0.2 for a, b in occupied):
                break
        occupied.append((x, y))
        color = rng.uniform(0.02, 0.95, 3)
        if rng.random() < 0.5:
            color = np.array(
                [[0.9, 0.02, 0.02], [0.95, 0.8, 0.02], [0.03, 0.8, 0.05], [0.02, 0.08, 0.9]]
            )[int(rng.integers(4))]
        geoms.append(
            f'<geom name="object_{index}" {shape} pos="{x} {y} {height + dz}" '
            f'euler="0 0 {rng.uniform(-180, 180)}" rgba="{rgb(color)}"/>'
        )
        objects.append((f'object_{index}', kind))
    bearing = rng.uniform(-65, 65) * np.pi / 180
    distance = rng.uniform(0.75, 1.7)
    eye = np.array(
        [np.sin(bearing) * distance, -sy - np.cos(bearing) * distance, rng.uniform(1.1, 1.65)]
    )
    target = np.array([rng.uniform(-0.25, 0.25), rng.uniform(-0.15, 0.15), height])
    back = eye - target
    back /= np.linalg.norm(back)
    right = np.cross([0, 0, 1], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    xyaxes = ' '.join(map(str, np.r_[right, up]))
    xml = f'''<mujoco><visual><global offwidth="424" offheight="320"/>
      <headlight ambient=".4 .4 .4"/></visual><worldbody>
      <light pos="{rng.uniform(-1, 1)} -1 3" diffuse=".8 .8 .8"/>
      <camera name="rgbd" pos="{' '.join(map(str, eye))}" xyaxes="{xyaxes}" fovy="58"/>
      <geom type="plane" size="5 5 .1" rgba="{rgb(background)}"/>
      {''.join(geoms)}</worldbody></mujoco>'''
    return xml, [('table', 0), *objects]


def make_background_scene(rng):
    """Floor, wall and furniture panels; no tabletop target objects."""
    grey = ' '.join(map(str, rng.uniform(0.2, 0.8, 3))) + ' 1'
    xml = f'''<mujoco><visual><global offwidth="424" offheight="320"/></visual>
      <worldbody><light pos="0 -1 3"/><camera name="rgbd" pos="0 -2 1.3"
      xyaxes="1 0 0 0 .8 .6" fovy="58"/>
      <geom type="plane" size="5 5 .1" rgba="{grey}"/>
      <geom type="box" pos="0 .8 1" size="2 .1 1" rgba="{grey}"/>
      <geom type="box" pos="-.5 0 .6" size=".3 .2 .6" rgba="{grey}"/>
      <geom type="box" pos=".5 .2 .4" size=".06 .3 .4" rgba="{grey}"/>
      </worldbody></mujoco>'''
    return xml, []


def generate(directory, count, seed, hard_negative_fraction=0.0):
    initialize_renderer()
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    entries = []
    for index in range(count):
        split = 'val' if index % 5 == 0 else 'train'
        image_dir, label_dir = directory / 'images' / split, directory / 'labels' / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        hard = index < int(count * hard_negative_fraction)
        background = hard and index % 2 == 0
        xml, targets = (
            make_background_scene(rng) if background else make_scene(rng, include_boxes=not hard)
        )
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        with mujoco.Renderer(model, height=288, width=384) as renderer:
            renderer.update_scene(data, camera='rgbd')
            image = renderer.render().copy()
            renderer.enable_depth_rendering()
            depth = renderer.render().copy()
            renderer.disable_depth_rendering()
            renderer.enable_segmentation_rendering()
            segmentation = renderer.render().copy()
        labels = []
        for name, class_id in targets:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            mask = (
                (segmentation[:, :, 0] == geom_id)
                & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
            ).astype('uint8')
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < 12:
                continue
            epsilon = 0.002 * cv2.arcLength(contour, True)
            polygon = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2).astype(float)
            if len(polygon) < 3:
                continue
            polygon /= [image.shape[1], image.shape[0]]
            labels.append(
                str(class_id) + ' ' + ' '.join(f'{value:.6f}' for value in polygon.ravel())
            )
        filename = f'{index:05d}'
        if hard:
            capture_dir = directory / 'captures'
            capture_dir.mkdir(exist_ok=True)
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'rgbd')
            transform = np.eye(4)
            transform[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
            transform[:3, 3] = data.cam_xpos[camera_id]
            focal = image.shape[0] / (2 * np.tan(np.deg2rad(model.cam_fovy[camera_id]) / 2))
            k = [focal, 0, (image.shape[1] - 1) / 2, 0, focal, (image.shape[0] - 1) / 2, 0, 0, 1]
            np.savez_compressed(
                capture_dir / (filename + '.npz'),
                rgb=image,
                depth=depth,
                k=k,
                camera_to_processing=transform,
            )
        cv2.imwrite(str(image_dir / (filename + '.png')), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        (label_dir / (filename + '.txt')).write_text('\n'.join(labels) + '\n')
        entries.append(
            {
                'id': filename,
                'split': split,
                'classes': [int(line.split()[0]) for line in labels],
                'hard_negative': hard,
                'mode': 'background'
                if background
                else ('round_objects_only' if hard else 'mixed_objects'),
            }
        )
        if index % 40 == 0:
            print(f'Rendered {index}/{count}', flush=True)
    (directory / 'dataset.yaml').write_text(
        yaml.safe_dump(
            {
                'path': str(directory.resolve()),
                'train': 'images/train',
                'val': 'images/val',
                'names': CLASSES,
            }
        )
    )
    (directory / 'manifest.json').write_text(
        json.dumps(
            {
                'seed': seed,
                'count': count,
                'hard_negative_fraction': hard_negative_fraction,
                'source': 'Randomized standalone MuJoCo scenes, not Maxwell evaluation views',
                'annotation': 'MuJoCo geom segmentation, used for training labels only',
                'entries': entries,
            },
            indent=2,
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=240)
    parser.add_argument('--epochs', type=int, default=35)
    parser.add_argument('--seed', type=int, default=73)
    parser.add_argument('--dataset-only', action='store_true')
    parser.add_argument('--reuse-dataset', action='store_true')
    parser.add_argument('--hard-negative-fraction', type=float, default=0.25)
    parser.add_argument('--output', default='artifacts/perception')
    args = parser.parse_args()
    if args.samples < 5 or not 0 <= args.hard_negative_fraction <= 1:
        parser.error('samples must be >= 5 and hard-negative-fraction in [0,1]')
    output = ROOT / args.output
    dataset = output / 'dataset'
    if not args.reuse_dataset:
        generate(dataset, args.samples, args.seed, args.hard_negative_fraction)
    if args.dataset_only:
        return
    dataset_manifest = json.loads((dataset / 'manifest.json').read_text())
    # Disable automatic package/config updates and direct caches to workspace.
    import os

    os.environ['YOLO_CONFIG_DIR'] = str(output / 'ultralytics_config')
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    import torch

    torch.set_num_threads(4)
    from ultralytics import YOLO

    pretrained = output / 'yolo11n-seg.pt'
    from g1_backend.cuda.training import train_yolo

    _, device = train_yolo(
        YOLO,
        pretrained,
        {
            'data': str(dataset / 'dataset.yaml'),
            'epochs': args.epochs,
            'imgsz': 416,
            'batch': 8,
            'workers': 0,
            'seed': args.seed,
            'deterministic': True,
            'project': str(output / 'training'),
            'name': 'synthetic',
            'exist_ok': True,
            'plots': False,
            'cache': True,
            'amp': False,
            'patience': 12,
            'close_mosaic': 5,
            'mosaic': 0.3,
            'scale': 0.3,
            'translate': 0.05,
            'degrees': 10,
            'fliplr': 0.5,
        },
    )
    best = output / 'training/synthetic/weights/best.pt'
    import shutil

    shutil.copy2(best, output / 'g1_objects_yolo11n_seg.pt')
    exported = YOLO(str(output / 'g1_objects_yolo11n_seg.pt')).export(
        format='onnx', imgsz=640, dynamic=False, simplify=False, opset=17
    )
    manifest = {
        'architecture': 'YOLO11n-seg',
        'classes': CLASSES,
        'checkpoint_sha256': hashlib.sha256(
            (output / 'g1_objects_yolo11n_seg.pt').read_bytes()
        ).hexdigest(),
        'onnx_sha256': hashlib.sha256(Path(exported).read_bytes()).hexdigest(),
        'pretrained_sha256': hashlib.sha256(pretrained.read_bytes()).hexdigest(),
        'samples': dataset_manifest['count'],
        'hard_negative_fraction': dataset_manifest.get('hard_negative_fraction', 0.0),
        'epochs_requested': args.epochs,
        'seed': args.seed,
        'ultralytics': __import__('ultralytics').__version__,
        'torch': torch.__version__,
        'device': device,
        'domain': 'synthetic objects; no hardware validation',
    }
    (output / 'model_manifest.json').write_text(json.dumps(manifest, indent=2))
    print('Prepared model:', output / 'g1_objects_yolo11n_seg.pt', flush=True)


if __name__ == '__main__':
    main()
