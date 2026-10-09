import numpy as np

from g1_backend.depth_preview import depth_preview


def test_far_background_does_not_flatten_valid_depth_and_invalid_is_distinct():
    depth = np.array([[1, 2, 400], [0, np.nan, np.inf]], np.float32)
    original = depth.copy()
    preview = depth_preview(depth, annotate=False)
    assert preview.dtype == np.uint8 and preview.shape == (2, 3, 3)
    assert not np.array_equal(preview[0, 0], preview[0, 1])
    np.testing.assert_array_equal(preview[0, 2], [100, 0, 100])
    np.testing.assert_array_equal(preview[1], np.tile([100, 0, 100], (3, 1)))
    np.testing.assert_array_equal(depth, original)


def test_empty_readings_still_have_visible_invalid_color():
    preview = depth_preview(np.zeros((24, 32), np.float32), annotate=False)
    assert (preview[:, :, 0] == 100).all()
    assert (preview[:, :, 2] == 100).all()
