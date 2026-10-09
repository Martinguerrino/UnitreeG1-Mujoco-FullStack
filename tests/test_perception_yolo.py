"""YOLO mask decoding and registered-depth geometry without model downloads."""

import numpy as np
import pytest

from g1_backend.perception import Settings
from g1_backend.perception_rgbd import RefinementSettings, detect_instances
from g1_backend.perception_yolo import Instance, decode_outputs


def test_decoding_restores_letterbox_masks_and_suppresses_duplicates():
    # One class, one prototype, 3 candidate anchors. Input is 8x8 with a
    # 4x8 image centered in its padded rows; two candidates are duplicates.
    prediction = np.array(
        [
            [
                [4.0, 4.0, 4.0],
                [4.0, 4.0, 4.0],
                [4.0, 4.0, 4.0],
                [2.0, 2.0, 2.0],
                [0.9, 0.8, 0.1],
                [10.0, 10.0, 10.0],
            ]
        ],
        dtype='float32',
    )
    proto = np.ones((1, 1, 2, 2), dtype='float32')
    result = decode_outputs(prediction, proto, {0: 'box'}, (4, 8), (8, 8), 0.3)
    assert len(result) == 1 and result[0].label == 'box'
    assert result[0].box_xyxy == [2, 1, 6, 3]
    assert result[0].mask.shape == (4, 8) and result[0].mask.sum() == 8
    with pytest.raises(ValueError):
        decode_outputs(prediction[:, :5], proto, {0: 'box'}, (4, 8), (8, 8))


def rgbd():
    h, w, f = 120, 160, 100.0
    v, u = np.mgrid[:h, :w]
    x, y = (u - (w - 1) / 2) * 0.75 / f, -(v - (h - 1) / 2) * 0.75 / f
    mask = (abs(x - 0.1) < 0.08) & (abs(y - 0.05) < 0.04)
    depth = np.full((h, w), 0.85)
    depth[mask] = 0.75
    rgb = np.full((h, w, 3), 100, dtype='uint8')
    rgb[mask] = [255, 0, 0]
    k = [f, 0, (w - 1) / 2, 0, f, (h - 1) / 2, 0, 0, 1]
    extrinsic = np.diag([1.0, -1.0, -1.0, 1.0])
    extrinsic[2, 3] = 1.6
    yy, xx = np.where(mask)
    bounds = [int(xx.min()), int(yy.min()), int(xx.max() + 1), int(yy.max() + 1)]
    instances = [Instance('table', 0.95, ~mask, [0, 0, w, h]), Instance('box', 0.9, mask, bounds)]
    return depth, rgb, k, extrinsic, instances


def test_semantic_masks_limit_geometry_and_preserve_class_and_quality():
    depth, rgb, k, extrinsic, instances = rgbd()
    tables = detect_instances(
        depth,
        rgb,
        k,
        instances,
        extrinsic,
        settings=Settings(min_table_points=50),
        refinement=RefinementSettings(erosion_pixels=0),
    )
    assert len(tables) == 1 and len(tables[0]['objects']) == 1
    box = tables[0]['objects'][0]
    assert box['semantic_class'] == 'box' and box['source'] == 'yolo_rgbd'
    assert box['quality']['segmentation_confidence'] == 0.9
    assert box['quality']['valid_depth_fraction'] == 1
    np.testing.assert_allclose(box['center'], [0.1, 0.05, 0.8], atol=0.012)
    np.testing.assert_allclose(box['dimensions'], [0.16, 0.08, 0.1], atol=0.015)
    assert box['_points'].shape[1] == 3
    # Depth geometry cannot invent a semantic box when YOLO supplied no mask.
    only_table = detect_instances(
        depth, rgb, k, instances[:1], extrinsic, settings=Settings(min_table_points=50)
    )
    assert only_table[0]['objects'] == []


def test_invalid_depth_is_reported_and_wrong_mask_resolution_rejected():
    depth, rgb, k, extrinsic, instances = rgbd()
    depth[instances[1].mask] = np.nan
    tables = detect_instances(
        depth, rgb, k, instances, extrinsic, settings=Settings(min_table_points=50)
    )
    assert tables[0]['objects'] == []
    instances[0].mask = instances[0].mask[:3]
    with pytest.raises(ValueError):
        detect_instances(depth, rgb, k, instances, extrinsic)


def test_mask_containment_removes_fragments_but_preserves_neighbor_and_other_class():
    from g1_backend.perception_yolo import suppress_contained_masks

    outer = np.zeros((20, 30), dtype=bool)
    outer[2:12, 2:12] = True
    inner = np.zeros_like(outer)
    inner[3:7, 3:7] = True
    neighbor = np.zeros_like(outer)
    neighbor[2:12, 15:25] = True
    instances = [
        Instance('box', 0.9, outer, [2, 2, 12, 12]),
        Instance('box', 0.7, inner, [3, 3, 7, 7]),
        Instance('box', 0.8, neighbor, [15, 2, 25, 12]),
        Instance('sphere', 0.6, inner, [3, 3, 7, 7]),
    ]
    result = suppress_contained_masks(instances)
    assert len(result) == 3
    assert [item.label for item in result] == ['box', 'box', 'sphere']


def test_raised_tray_is_not_published_as_another_table():
    depth, rgb, k, extrinsic, _ = rgbd()
    depth[:] = 0.85
    v, u = np.mgrid[: depth.shape[0], : depth.shape[1]]
    x, y = (u - 79.5) * 0.75 / 100, -(v - 59.5) * 0.75 / 100
    tray = (abs(x) < 0.16) & (abs(y) < 0.16)
    depth[tray] = 0.75
    instances = [
        Instance('table', 0.95, ~tray, [0, 0, 160, 120]),
        Instance('table', 0.8, tray, [55, 35, 105, 85]),
    ]
    result = detect_instances(
        depth,
        rgb,
        k,
        instances,
        extrinsic,
        settings=Settings(min_table_points=50),
        refinement=RefinementSettings(erosion_pixels=0),
    )
    assert len(result) == 1
    assert result[0]['center'][2] == pytest.approx(0.75)


def test_floor_table_proposal_is_absent_from_validated_overlay():
    from g1_backend.perception_rgbd import validated_instances
    from g1_backend.perception_yolo import overlay

    depth, rgb, k, extrinsic, _ = rgbd()
    depth[:] = 1.6  # Optical depth -> z=0 floor in the processing frame.
    instance = Instance('table', 0.65, np.ones(depth.shape, dtype=bool), [0, 0, 160, 120])
    tables = detect_instances(depth, rgb, k, [instance], extrinsic)
    assert tables == []
    validated = validated_instances(depth, rgb, k, [instance], tables, extrinsic)
    assert validated == []
    np.testing.assert_array_equal(overlay(rgb, validated), rgb)
    assert not np.array_equal(
        overlay(rgb, [instance]), rgb
    )  # Diagnostic proposals remain visible.


def test_mixed_floor_and_table_mask_is_clipped_to_valid_surface_and_bounds():
    from g1_backend.perception_rgbd import validated_instances

    depth, rgb, k, extrinsic, _ = rgbd()
    depth[:] = 1.6
    region = np.zeros(depth.shape, dtype=bool)
    region[30:90, 35:125] = True
    depth[region] = 0.85
    instance = Instance('table', 0.65, np.ones(depth.shape, dtype=bool), [0, 0, 160, 120])
    tables = detect_instances(
        depth, rgb, k, [instance], extrinsic, settings=Settings(min_table_points=50)
    )
    validated = validated_instances(depth, rgb, k, [instance], tables, extrinsic)
    assert len(validated) == 1 and validated[0].confidence == 0.65
    assert not np.any(validated[0].mask & ~region)
    assert validated[0].mask.sum() >= region.sum() * 0.9
    assert validated[0].box_xyxy == [35, 30, 125, 90]


def test_validated_overlay_uses_output_frame_and_keeps_real_box():
    from g1_backend.perception_rgbd import validated_instances

    depth, rgb, k, extrinsic, instances = rgbd()
    output_transform = np.eye(4)
    output_transform[:3, 3] = [2.0, -1.0, -0.8]
    tables = detect_instances(
        depth,
        rgb,
        k,
        instances,
        extrinsic,
        output_transform,
        settings=Settings(min_table_points=50),
    )
    validated = validated_instances(depth, rgb, k, instances, tables, output_transform @ extrinsic)
    assert {instance.label for instance in validated} == {'table', 'box'}
    box = next(instance for instance in validated if instance.label == 'box')
    assert not np.any(box.mask & ~instances[1].mask)


def test_round_top_mislabeled_as_box_is_rejected_but_cylinder_is_retained():
    depth, rgb, k, extrinsic, _instances = rgbd()
    v, u = np.mgrid[: depth.shape[0], : depth.shape[1]]
    round_mask = (u - 90) ** 2 + (v - 55) ** 2 < 7**2
    depth[:] = 0.85
    depth[round_mask] = 0.75
    rgb[:] = 100
    proposals = [
        Instance('table', 0.95, ~round_mask, [0, 0, 160, 120]),
        Instance('box', 0.9, round_mask, [83, 48, 98, 63]),
    ]
    settings = Settings(min_table_points=50)
    refined = detect_instances(depth, rgb, k, proposals, extrinsic, settings=settings)
    assert refined and not refined[0]['objects']
    proposals[1].label = 'cylinder'
    refined = detect_instances(depth, rgb, k, proposals, extrinsic, settings=settings)
    assert any(
        obj['semantic_class'] == 'cylinder' for table in refined for obj in table['objects']
    )


def test_depth_space_duplicates_across_visual_classes_are_suppressed():
    depth, rgb, k, extrinsic, instances = rgbd()
    mask = instances[1].mask.copy()
    instances.append(Instance('cylinder', 0.4, mask, instances[1].box_xyxy))
    refined = detect_instances(
        depth, rgb, k, instances, extrinsic, settings=Settings(min_table_points=50)
    )
    objects = [obj for table in refined for obj in table['objects']]
    assert len(objects) == 1 and objects[0]['semantic_class'] == 'box'
