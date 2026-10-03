from pathlib import Path
import numpy as np
import yaml
from g1_backend.sensor_effects import SensorEffects, DelayedFrames

PROFILE = yaml.safe_load((Path(__file__).resolve().parents[1] / 'configs/navigation/sensors_real.yaml').read_text())


def test_lidar_seeded_radial_noise_and_range_limits():
    points = np.tile([3., 0., 0.], (10000, 1)).astype(np.float32)
    a = SensorEffects(PROFILE, 1).lidar(points, 20)
    b = SensorEffects(PROFILE, 1).lidar(points, 20)
    assert np.array_equal(a, b)
    assert 9500 < len(a) < len(points)
    assert .009 < np.std(a[:, 0]) < .014
    assert np.all(a[:, 1:] == 0)
    assert np.all(points[:, 0] == 3)


def test_depth_noise_grows_with_distance_and_invalid_values_remain_invalid():
    effect = SensorEffects(PROFILE, 2)
    near = np.ones((100, 100), dtype=np.float32)
    far = near * 5
    output = effect.camera({'depth': near})['depth']
    distant = effect.camera({'depth': far})['depth']
    assert np.std(distant[distant > 0]) > 8 * np.std(output[output > 0])
    assert np.count_nonzero(output == 0) > 0
    invalid = np.array([[np.nan, np.inf, 0., 10.]], dtype=np.float32)
    assert np.all(effect.camera({'depth': invalid})['depth'] == 0)
    assert np.all(near == 1)


def test_rgb_and_imu_preserve_shape_type_and_capture_time():
    effect = SensorEffects(PROFILE, 3)
    image = np.full((20, 20, 3), 128, dtype=np.uint8)
    result = effect.camera({'d435i_rgb': image})['d435i_rgb']
    assert result.dtype == np.uint8 and result.shape == image.shape
    assert not np.array_equal(image, result)
    sample = {'time': 1., 'gyro': [0., 0., 0.], 'acc': [0., 0., 9.81]}
    imu = effect.imu(sample)
    assert imu['time'] == 1 and imu['gyro'] != sample['gyro']
    assert sample['gyro'] == [0., 0., 0.]


def test_delayed_frames_preserve_timestamps_and_bound_memory():
    delivery = DelayedFrames(.03)
    frame = {'time': 1.}
    assert delivery.receive(frame, 1.) is None
    assert delivery.receive(None, 1.02) is None
    assert delivery.receive(None, 1.04) is frame
    assert frame['time'] == 1.
    for i in range(100):
        delivery.receive({'time': float(i)}, -1)
    assert len(delivery.pending) == 8
