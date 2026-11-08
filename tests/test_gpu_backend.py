"""Fallback contracts run everywhere; numerical parity runs on an optional GPU."""

from types import SimpleNamespace
import importlib

import numpy as np
import pytest

from g1_backend import acceleration
from g1_backend.cuda import geometry


@pytest.fixture(autouse=True)
def reset_failures():
    saved = acceleration._DISABLED.copy()
    acceleration._DISABLED.clear()
    yield
    acceleration._DISABLED.clear()
    acceleration._DISABLED.update(saved)


def test_cpu_override_never_imports_optional_cuda(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'off')

    def forbidden(*args):
        raise AssertionError('CPU mode imported a CUDA dependency')

    monkeypatch.setattr(acceleration.importlib, 'import_module', forbidden)
    assert acceleration.cupy() is None
    assert acceleration.torch_device() == 'cpu'
    assert geometry.transform(np.ones((40000, 3)), np.eye(4)) is None


def test_missing_cupy_is_cached_and_cpu_remains_available(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'auto')
    calls = []

    def missing(name):
        calls.append(name)
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(acceleration.importlib, 'import_module', missing)
    assert acceleration.cupy() is None
    assert acceleration.cupy() is None
    assert calls == ['cupy']


def test_kernel_failure_requests_original_cpu_path(monkeypatch):
    monkeypatch.setattr(geometry, 'cupy', lambda: object())

    def failed(cp):
        raise RuntimeError('out of device memory')

    assert geometry.attempt('test_kernel', 100000, failed) is None
    assert 'test_kernel' in acceleration._DISABLED
    assert geometry.attempt('test_kernel', 100000, lambda cp: 1) is None


class FakeOrt:
    def __init__(self, failure=None):
        self.created = []
        self.failure = failure

    def get_available_providers(self):
        return ['CUDAExecutionProvider', 'CPUExecutionProvider']

    def InferenceSession(self, path, sess_options, providers):
        self.created.append(providers)
        if self.failure == 'init' and providers[0] == 'CUDAExecutionProvider':
            raise RuntimeError('missing cuDNN')
        owner = self

        class Session:
            def get_providers(self):
                return providers

            def run(self, outputs, inputs):
                if owner.failure == 'run' and providers[0] == 'CUDAExecutionProvider':
                    raise RuntimeError('CUDA failure')
                return inputs['obs'] + 1

        return Session()


@pytest.mark.parametrize('failure', ['init', 'run'])
def test_onnx_failure_retries_original_cpu_session(monkeypatch, failure):
    monkeypatch.setenv('G1_GPU', 'auto')
    ort = FakeOrt(failure)
    session = acceleration.OnnxSession(ort, 'model.onnx', object())
    np.testing.assert_array_equal(session.run(None, {'obs': np.array([2])}), [3])
    assert session.backend == 'cpu'
    assert ort.created[-1] == ['CPUExecutionProvider']


def test_disable_gpu_after_session_creation(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'auto')
    session = acceleration.OnnxSession(FakeOrt(), 'model.onnx', object())
    assert session.backend == 'cuda'
    monkeypatch.setenv('G1_GPU', 'off')
    session.run(None, {'obs': np.array([1])})
    assert session.backend == 'cpu'


def test_training_failure_restarts_from_fresh_source_weights(monkeypatch):
    from g1_backend.cuda import training

    monkeypatch.setattr(training, 'torch_device', lambda: 'cuda')
    created, devices = [], []

    class Model:
        def __init__(self, path):
            created.append(self)

        def train(self, **kwargs):
            devices.append(kwargs['device'])
            if kwargs['device'] == 'cuda':
                raise RuntimeError('out of memory')

    _, device = training.train_yolo(Model, 'weights.pt', {'seed': 7})
    assert device == 'cpu' and devices == ['cuda', 'cpu']
    assert len(created) == 2 and created[0] is not created[1]


def test_act_chunk_failure_retries_after_cpu_migration(monkeypatch):
    from g1_backend.cuda import act

    monkeypatch.setenv('G1_GPU', 'auto')
    monkeypatch.setattr(act, 'torch_device', lambda: 'cuda')
    controller = act.ActDevice('checkpoint')
    monkeypatch.setattr(controller, 'preprocessor', lambda: 'cpu_pre')
    moved = []

    def chunk(state, image):
        if controller.device == 'cuda':
            raise RuntimeError('out of memory')
        return np.zeros((100, 16))

    owner = SimpleNamespace(policy=SimpleNamespace(to=moved.append), _chunk=chunk)
    assert controller.chunk(owner, None, None).shape == (100, 16)
    assert moved == ['cpu'] and owner.pre == 'cpu_pre'


@pytest.fixture
def gpu(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'auto')
    if importlib.util.find_spec('cupy') is None:
        pytest.skip('Optional CuPy is not installed')
    cp = acceleration.cupy()
    if cp is None:
        pytest.skip('CUDA device/runtime unavailable')
    # Once the runtime is selected, kernel errors must fail rather than skip.
    assert int(cp.arange(4).sum().get()) == 6
    return cp


def test_cuda_transform_and_voxel_match_cpu(gpu):
    from g1_backend.slam_lidar_graph import voxel_cloud

    rng = np.random.default_rng(4)
    points = rng.normal(size=(40000, 3))
    matrix = np.eye(4)
    matrix[:3, 3] = [1, 2, 3]
    result = geometry.transform(points, matrix)
    assert result is not None
    np.testing.assert_allclose(result, points + [1, 2, 3], atol=1e-12)
    expected = np.unique(np.floor(points / 0.08).astype(np.int64), axis=0, return_index=True)[1]
    np.testing.assert_array_equal(geometry.voxel_indices(points, 0.08), expected)
    assert len(voxel_cloud(points, maximum=100)) == 100


def test_cuda_ransac_matches_seeded_cpu(gpu):
    rng = np.random.default_rng(8)
    cloud = rng.uniform(-1, 1, (10000, 3))
    cloud[:8000, 2] = 0.72
    samples = np.asarray([rng.choice(len(cloud), 3, replace=False) for _ in range(180)])
    best = np.empty(0, dtype=int)
    for sample in samples:
        a, b, c = cloud[sample]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if abs(normal[2]) < 0.97:
            continue
        indices = np.flatnonzero(abs((cloud - a) @ normal) < 0.008)
        if len(indices) > len(best):
            best = indices
    result = geometry.plane_inliers(cloud, samples, 0.008, 0.97)
    assert result is not None
    np.testing.assert_array_equal(result, best)


def test_cuda_backprojection_and_scan_projection_match_cpu(gpu, monkeypatch):
    from g1_backend.perception import Settings, backproject
    from g1_backend.navigation import project_scan

    depth = np.full((480, 640), 2, dtype=np.float32)
    depth[0, 0] = np.nan
    rgb = np.full((480, 640, 3), 80, dtype=np.uint8)
    k = np.array([[400.0, 0, 319.5], [0, 400, 239.5], [0, 0, 1]])
    settings = Settings()
    result = geometry.backproject(depth, rgb, k, settings)
    assert result is not None
    monkeypatch.setenv('G1_GPU', 'off')
    expected = backproject(depth, rgb, k, settings)
    np.testing.assert_allclose(result[0], expected[0], atol=1e-12)
    np.testing.assert_array_equal(result[1], expected[1])
    points = np.random.default_rng(1).uniform(-3, 3, (12000, 3))
    config = {'horizontal_samples': 360, 'min_range': 0.1, 'max_range': 20}
    expected = project_scan(points, np.zeros(3), 0.2, config)
    monkeypatch.setenv('G1_GPU', 'auto')
    result = geometry.project_scan(points, np.zeros(3), 0.2, config)
    assert result is not None
    for actual, reference in zip(result, expected):
        if reference is None:
            assert actual is None
        else:
            assert actual == pytest.approx(reference, abs=1e-12)


def test_cuda_global_map_preserves_cpu_voxel_representatives(gpu, monkeypatch):
    from g1_backend.slam_lidar_graph import optimized_cloud

    scans = {1: np.random.default_rng(6).normal(size=(40000, 3)).astype(np.float32)}
    poses = {1: np.eye(4)}
    poses[1][:3, 3] = [1, 2, 3]
    actual = geometry.optimized_cloud(scans, poses, 0.08, 400000)
    assert actual is not None
    monkeypatch.setenv('G1_GPU', 'off')
    np.testing.assert_array_equal(actual, optimized_cloud(scans, poses))


@pytest.mark.parametrize(
    'kind,size',
    [
        ('plane', '3 3 .1'),
        ('sphere', '.5'),
        ('box', '.4 .5 .6'),
        ('capsule', '.3 .5'),
        ('cylinder', '.3 .5'),
        ('ellipsoid', '.3 .4 .5'),
    ],
)
def test_cuda_rays_match_mujoco_primitives(gpu, kind, size):
    import mujoco
    from g1_backend.cuda.lidar import Raycaster

    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><worldbody><geom type="{kind}" size="{size}" '
        'pos="0 0 0" euler="10 20 30" group="2"/></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    origin = np.array([0.0, -2.0, 0.5])
    directions = np.random.default_rng(4).normal(size=(4000, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    groups = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
    actual, ids = Raycaster(model, groups, gpu).cast(data, origin, directions, 20)
    expected, expected_ids = np.empty(4000), np.empty(4000, dtype=np.int32)
    mujoco.mj_multiRay(
        model,
        data,
        origin,
        directions.ravel(),
        groups,
        1,
        -1,
        expected_ids,
        expected,
        len(directions),
        20,
    )
    np.testing.assert_allclose(actual, expected, atol=1e-9)
    np.testing.assert_array_equal(ids, expected_ids)


def test_unsupported_lidar_geometry_returns_cpu_path(gpu):
    import mujoco
    from g1_backend.cuda.lidar import multi_ray

    model = mujoco.MjModel.from_xml_string(
        '<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 .1"/>'
        '</asset><worldbody><geom type="hfield" hfield="h" group="2"/></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert (
        multi_ray(
            model,
            data,
            np.array([0.0, 0.0, 2.0]),
            np.array([[0.0, 0.0, -1.0]]),
            np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8),
            20,
        )
        is None
    )


def test_cuda_spatial_components_and_counts_match_cpu(gpu, monkeypatch):
    from scipy.spatial import cKDTree
    from g1_backend.cuda.spatial import components, radius_counts
    from g1_backend.perception import clusters

    rng = np.random.default_rng(20)
    cloud = np.vstack(
        [rng.uniform(-0.2, 0.2, (2200, 3)), rng.uniform(-0.2, 0.2, (2200, 3)) + [2, 0, 0]]
    )
    expected_counts = cKDTree(cloud).query_ball_point(cloud, 0.08, return_length=True)
    actual_counts = radius_counts(cloud, 0.08)
    assert actual_counts is not None
    np.testing.assert_array_equal(actual_counts, expected_counts)
    actual = components(cloud, 0.08)
    assert actual is not None
    monkeypatch.setenv('G1_GPU', 'off')
    expected = list(clusters(cloud, 0.08))
    assert len(actual) == len(expected)
    for a, b in zip(actual, expected):
        np.testing.assert_array_equal(np.sort(a), np.sort(b))


def test_cuda_sensor_noise_preserves_seed_and_rng_state(gpu, monkeypatch):
    from g1_backend.sensor_effects import SensorEffects
    from g1_backend.config import ROOT
    import yaml

    profile = yaml.safe_load((ROOT / 'configs/navigation/sensors_real.yaml').read_text())
    # Profiles are nested under sensor_effects in the navigation config.
    profile = profile.get('sensor_effects', profile)
    sensors = {
        'rgb': np.full((480, 640, 3), 100, dtype=np.uint8),
        'depth': np.full((480, 640), 2, dtype=np.float32),
    }
    accelerated = SensorEffects(profile, seed=7)
    actual = accelerated.camera(sensors)
    assert not getattr(accelerated, '_cuda_failed', False)
    monkeypatch.setenv('G1_GPU', 'off')
    original = SensorEffects(profile, seed=7)
    expected = original.camera(sensors)
    np.testing.assert_array_equal(actual['rgb'], expected['rgb'])
    np.testing.assert_array_equal(actual['depth'], expected['depth'])
    assert accelerated.rng.bit_generator.state == original.rng.bit_generator.state


def test_workspace_matches_cpu_and_clears_on_exception(gpu, monkeypatch):
    from g1_backend.cuda.workspace import frame_scope, active
    from g1_backend.perception import Settings, backproject

    depth = np.full((480, 640), 1.5, dtype=np.float32)
    rgb = np.full((480, 640, 3), 80, dtype=np.uint8)
    k = np.array([[400.0, 0, 319.5], [0, 400.0, 239.5], [0, 0, 1]])
    with pytest.raises(RuntimeError, match='test'), frame_scope(depth, rgb) as workspace:
        assert workspace is not None and active() is workspace
        actual = backproject(depth, rgb, k, Settings())
        matrix = np.eye(4)
        matrix[:3, 3] = [1, 2, 3]
        transformed, _ = workspace.backproject(depth, rgb, k, Settings(), transform=matrix)
        np.testing.assert_allclose(transformed, actual[0] + [1, 2, 3], atol=1e-12)
        raise RuntimeError('test')
    assert active() is None
    monkeypatch.setenv('G1_GPU', 'off')
    expected = backproject(depth, rgb, k, Settings())
    np.testing.assert_allclose(actual[0], expected[0], atol=1e-12)
    np.testing.assert_array_equal(actual[1], expected[1])


def test_nvenc_failure_restarts_export_on_cpu(monkeypatch):
    from g1_backend import recording
    from g1_backend.cuda import video

    monkeypatch.setattr(video, 'codec', lambda: 'h264_nvenc')
    calls = []

    def encode(directory, output, fps, view, encoder):
        calls.append(encoder)
        if encoder == 'h264_nvenc':
            raise RuntimeError('encoder failed')
        return {'encoder': encoder}

    monkeypatch.setattr(recording, '_encode_video', encode)
    assert recording.encode_video('recording')['encoder'] == 'libx264'
    assert calls == ['h264_nvenc', 'libx264']


def test_cuda_yolo_matches_cpu_model(gpu, monkeypatch):
    from g1_backend.perception_yolo import ROOT, YoloSegmenter

    model = ROOT / 'assets/perception/g1_objects_yolo11n_seg.onnx'
    pytest.importorskip('onnxruntime')
    # A rendered frame exercises masks rather than only the no-detection path.
    import mujoco

    scene = mujoco.MjModel.from_xml_string("""<mujoco><visual>
      <global offwidth="640" offheight="480"/></visual><worldbody>
      <light pos="0 0 3"/><camera name="rgbd" pos="0 -1.1 1.4"
       xyaxes="1 0 0 0 .5087293 .8609265" fovy="58"/>
      <geom type="plane" size="2 2 .1" rgba=".3 .3 .3 1"/>
      <geom type="box" pos="0 0 .72" size=".5 .35 .03" rgba=".5 .35 .2 1"/>
      <geom type="box" pos=".1 .05 .8" size=".08 .04 .05" rgba="1 0 0 1"/>
      </worldbody></mujoco>""")
    data = mujoco.MjData(scene)
    mujoco.mj_forward(scene, data)
    with mujoco.Renderer(scene, height=480, width=640) as renderer:
        renderer.update_scene(data, camera='rgbd')
        rgb = renderer.render().copy()
    accelerated = YoloSegmenter(model)
    assert accelerated.session.backend == 'cuda'
    actual = accelerated.predict(rgb)
    assert accelerated.cuda_pipeline is not None and accelerated.session.backend == 'cuda'
    monkeypatch.setenv('G1_GPU', 'off')
    expected = YoloSegmenter(model).predict(rgb)
    assert actual and len(actual) == len(expected)
    for a, b in zip(actual, expected):
        assert a.label == b.label and a.box_xyxy == b.box_xyxy
        assert a.confidence == pytest.approx(b.confidence, abs=1e-4)
        overlap = np.count_nonzero(a.mask & b.mask) / np.count_nonzero(a.mask | b.mask)
        assert overlap > 0.99


def test_cuda_mesh_rays_and_transparency_match_mujoco(gpu):
    import mujoco
    from g1_backend.cuda.lidar import Raycaster

    model = mujoco.MjModel.from_xml_string("""<mujoco><asset>
      <mesh name="tetra" vertex="0 0 0 1 0 0 0 1 0 0 0 1"
       face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody>
      <geom type="mesh" mesh="tetra" group="2" euler="10 20 30"/>
      <geom type="sphere" pos="0 -1 0" size=".8" group="2" rgba="1 1 1 0"/>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    origin = np.array([0.0, -2.0, 0.3])
    directions = np.random.default_rng(5).normal(size=(2000, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    groups = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
    actual, ids = Raycaster(model, groups, gpu).cast(data, origin, directions, 20)
    expected, expected_ids = np.empty(2000), np.empty(2000, dtype=np.int32)
    mujoco.mj_multiRay(
        model,
        data,
        origin,
        directions.ravel(),
        groups,
        1,
        -1,
        expected_ids,
        expected,
        len(directions),
        20,
    )
    np.testing.assert_allclose(actual, expected, atol=1e-8)
    np.testing.assert_array_equal(ids, expected_ids)


def test_cuda_depth_preview_matches_cpu(gpu, monkeypatch):
    from g1_backend.depth_preview import depth_preview
    from g1_backend.cuda.images import depth_colors

    depth = np.random.default_rng(9).uniform(0.1, 7, (480, 640)).astype(np.float32)
    depth[0, 0] = np.nan
    actual = depth_colors(depth, 0.2, 6.0)
    assert actual is not None
    monkeypatch.setenv('G1_GPU', 'off')
    expected = depth_preview(depth, annotate=False)
    np.testing.assert_array_equal(actual, expected)


def test_small_onnx_network_keeps_cpu_when_cuda_latency_is_worse(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'auto')
    ticks = iter([0.0, 1.0, 1.0, 3.0, 3.0, 4.0, 4.0, 6.0, 6.0, 7.0, 7.0, 9.0])
    monkeypatch.setattr(acceleration.time, 'perf_counter', lambda: next(ticks))
    session = acceleration.OnnxSession(FakeOrt(), 'model.onnx', object(), calibrate=True)
    np.testing.assert_array_equal(session.run(None, {'obs': np.array([1])}), [2])
    assert session.calibrated and session.backend == 'cpu'


def test_sensor_gpu_failure_restores_rng_before_cpu_retry(monkeypatch):
    from g1_backend.cuda import sensors as gpu_sensors
    from g1_backend.sensor_effects import SensorEffects
    from g1_backend.config import ROOT
    import yaml

    monkeypatch.setenv('G1_GPU', 'auto')

    def failed(*args, **kwargs):
        raise RuntimeError('device memory unavailable')

    monkeypatch.setattr(gpu_sensors, 'cupy', lambda: SimpleNamespace(asarray=failed))
    profile = yaml.safe_load((ROOT / 'configs/navigation/sensors_real.yaml').read_text())
    frame = {'rgb': np.full((480, 640, 3), 100, dtype=np.uint8)}
    owner = SensorEffects(profile, seed=4)
    actual = owner.camera(frame)
    assert owner._cuda_failed
    monkeypatch.setenv('G1_GPU', 'off')
    reference = SensorEffects(profile, seed=4)
    expected = reference.camera(frame)
    np.testing.assert_array_equal(actual['rgb'], expected['rgb'])
    assert owner.rng.bit_generator.state == reference.rng.bit_generator.state


def test_runtime_prefers_gpu_package_in_project_environment(monkeypatch, tmp_path):
    import sys
    import importlib.metadata
    from g1_backend.config import runtime_path

    monkeypatch.setenv('G1_GPU', 'auto')
    monkeypatch.delenv('G1_GPU_PYTHON_PATH', raising=False)
    monkeypatch.setattr(sys, 'path', list(sys.path))
    monkeypatch.setattr(
        importlib.metadata,
        'distribution',
        lambda name: SimpleNamespace(locate_file=lambda path: tmp_path),
    )
    runtime_path()
    assert sys.path[0] == str(tmp_path)


def test_cpu_runtime_does_not_auto_select_gpu_package(monkeypatch):
    import sys
    import importlib.metadata
    from g1_backend.config import runtime_path

    monkeypatch.setenv('G1_GPU', 'off')
    monkeypatch.delenv('G1_GPU_PYTHON_PATH', raising=False)
    monkeypatch.setattr(sys, 'path', list(sys.path))

    def forbidden(name):
        raise AssertionError('CPU override must not select the GPU package path')

    monkeypatch.setattr(importlib.metadata, 'distribution', forbidden)
    runtime_path()


def test_large_geometry_neighbors_never_try_quadratic_cuda(monkeypatch):
    from g1_backend.cuda import neighbors

    def forbidden():
        raise AssertionError('Large cloud attempted dense CUDA nearest neighbors')

    monkeypatch.setattr(neighbors, 'cupy', forbidden)
    reference = np.random.default_rng(2).normal(size=(6000, 3))
    index = neighbors.NeighborIndex(reference)
    distances, ids = index.query(reference[:1000])
    np.testing.assert_allclose(distances, 0)
    np.testing.assert_array_equal(ids, np.arange(1000))


def test_rejected_neighbor_cuda_choice_is_reused_across_frames(monkeypatch):
    from g1_backend.cuda import neighbors

    reference = np.random.default_rng(3).normal(size=(1500, 3))
    key = (len(reference).bit_length(), len(reference).bit_length(), 1)
    monkeypatch.setitem(neighbors._CHOICES, key, False)

    def forbidden():
        raise AssertionError('Repeated rejected CUDA calibration')

    monkeypatch.setattr(neighbors, 'cupy', forbidden)
    for _ in range(2):
        distances, _ = neighbors.NeighborIndex(reference).query(reference)
        np.testing.assert_allclose(distances, 0)
