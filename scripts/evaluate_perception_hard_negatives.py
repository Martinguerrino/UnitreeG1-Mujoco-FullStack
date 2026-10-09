"""Score YOLO and RGB-D rejection on independent cases without target boxes."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.perception_rgbd import RefinementSettings, detect_instances
from g1_backend.perception_yolo import YoloSegmenter


def run(args):
    model = YoloSegmenter(args.model, 0.3)
    records = []
    for path in sorted((Path(args.dataset) / 'captures').glob('*.npz')):
        with np.load(path) as data:
            instances = model.predict(data['rgb'])
            common = [
                data['depth'],
                data['rgb'],
                data['k'],
                instances,
                data['camera_to_processing'],
            ]
            previous = detect_instances(
                *common,
                refinement=RefinementSettings(
                    box_shape_validation=False,
                    min_supported_fraction=1e-6,
                    min_component_fraction=1e-6,
                    duplicate_point_fraction=1,
                ),
            )
            current = detect_instances(*common)

            def boxes(tables):
                return sum(
                    obj['kind'] == 'box_candidate' for table in tables for obj in table['objects']
                )

            records.append(
                {
                    'case': path.stem,
                    'visual_boxes': sum(i.label == 'box' for i in instances),
                    'baseline_boxes': boxes(previous),
                    'refined_boxes': boxes(current),
                }
            )
    if not records:
        raise ValueError('No hard-negative captures; generate the dataset first')
    report = {
        'model': args.model,
        'cases': len(records),
        'visual_boxes': sum(r['visual_boxes'] for r in records),
        'baseline_boxes': sum(r['baseline_boxes'] for r in records),
        'refined_boxes': sum(r['refined_boxes'] for r in records),
        'limits': [
            'Standalone synthetic cases without target boxes; not real-world precision.',
            'Bundled YOLO weights unchanged.',
        ],
        'records': records,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', default='artifacts/perception/hard_examples/dataset')
    parser.add_argument('--model', default='assets/perception/g1_objects_yolo11n_seg.onnx')
    parser.add_argument(
        '--output', default='results/perception/improvements_20261009/hard_negatives.json'
    )
    run(parser.parse_args())
