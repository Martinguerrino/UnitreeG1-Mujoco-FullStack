"""Skill deadlines, benchmark failure accounting and time-preserving video export."""

from pathlib import Path

import numpy as np
import pytest

from g1_backend.config import ROOT, load_config
from g1_backend.evaluation import run_benchmark, summarize, write_json
from g1_backend.recording import SensorRecorder, encode_video
from g1_backend.skills import SkillTimeout, TaskExecutor


def test_skill_requires_continuous_settling_and_records_error():
    task = TaskExecutor([{'skill': 'goto', 'settle_duration': 1.0}])
    task.start(0)
    assert not task.ready_to_complete(True, 1)
    assert not task.ready_to_complete(False, 1.8)
    assert not task.ready_to_complete(True, 2)
    assert task.ready_to_complete(True, 3)
    task.complete_skill(3, position_error=0.05)
    assert task.current is None
    assert task.events[-1]['duration'] == 3
    assert task.events[-1]['position_error'] == 0.05


def test_timeout_retries_are_bounded_and_do_not_advance_skill():
    task = TaskExecutor([{'skill': 'goto', 'timeout': 2, 'max_retries': 1}])
    assert not task.check_deadline(10)
    assert task.check_deadline(12)
    assert task.index == 0
    assert task.retries == 1
    with pytest.raises(SkillTimeout, match='Skill 0'):
        task.check_deadline(14)
    assert [event['event'] for event in task.events] == [
        'started',
        'timeout',
        'started',
        'timeout',
    ]


@pytest.mark.parametrize(
    'field,value', [('timeout', float('nan')), ('max_retries', -1), ('settle_duration', 30)]
)
def test_rejects_invalid_skill_timing(field, value):
    with pytest.raises(ValueError):
        TaskExecutor([{'skill': 'goto', field: value}])


@pytest.mark.parametrize(
    'field,value',
    [('duration', float('inf')), ('max_speed', float('nan')), ('warmup', -1), ('seed', -1)],
)
def test_config_rejects_non_reproducible_numbers(field, value):
    config = load_config(ROOT / 'configs/experiments/locomotion.yaml')
    config[field] = value
    with pytest.raises(ValueError):
        load_config(config)


def test_summary_counts_blocked_and_missing_separately():
    report = summarize(
        [
            {'status': 'completed', 'success': True, 'final_position_error': 0.1},
            {'status': 'failed', 'success': False, 'final_position_error': 0.3},
            {'status': 'blocked', 'success': False},
        ]
    )
    assert report['success_rate'] == 1 / 3
    assert report['statuses']['blocked'] == 1
    assert report['metrics']['final_position_error']['mean'] == pytest.approx(0.2)
    assert report['metrics']['final_position_error']['measured'] == 2
    assert report['metrics']['falls']['mean'] is None


def test_benchmark_preserves_missing_native_report_and_blocked_trial(tmp_path, monkeypatch):
    calls = []

    def process(arguments, log_path, timeout):
        calls.append(arguments)
        log_path.write_text('SDK log, not JSON\n')
        if len(calls) == 1:
            write_json(Path(arguments[-1]), {'status': 'blocked', 'success': False})
            return 2
        return -6

    monkeypatch.setattr('g1_backend.evaluation.run_process', process)
    report = run_benchmark(
        {
            'experiment': 'configs/experiments/locomotion.yaml',
            'repetitions': 2,
            'seed': 11,
            'results_dir': str(tmp_path),
            'cases': [
                {
                    'name': 'case',
                    'initial_position': [0, 0, 0.793],
                    'initial_yaw': 0,
                    'target': [0.5, 0, 0],
                    'max_speed': 0.3,
                }
            ],
        }
    )
    assert report['aggregate']['statuses'] == {'blocked': 1, 'failed': 1}
    assert [trial['seed'] for trial in report['trials']] == [11, 12]
    assert 'code -6' in report['trials'][1]['reason']
    assert (Path(report['results_dir']) / 'summary.json').exists()


def test_sensor_recording_tracks_duplicates_missing_samples_and_limit(tmp_path):
    recorder = SensorRecorder(tmp_path, {'record': True, 'max_recorded_frames': 1})
    rgb = {'rgb': np.zeros((8, 8, 3), dtype=np.uint8)}
    recorder.capture(rgb, 0.1, 1)
    recorder.capture(rgb, 0.1, 1)
    recorder.capture(rgb, 0.3, 3)
    report = recorder.close()
    assert report == {
        'recorded_frames': 1,
        'observed_frames': 2,
        'unobserved_frames': 1,
        'recording_truncated': True,
    }
    with np.load(tmp_path / 'camera_latest.npz') as frame:
        assert float(frame['timestamp']) == 0.3
    assert len((tmp_path / 'trajectory/camera/index.jsonl').read_text().splitlines()) == 1


def test_video_roundtrip_preserves_time_gaps(tmp_path):
    av = pytest.importorskip('av')
    recorder = SensorRecorder(tmp_path, {'record': True})
    for index, timestamp in enumerate([1.0, 1.1, 1.4]):
        recorder.capture(
            {'rgb': np.full((16, 16, 3), index * 100, dtype=np.uint8)}, timestamp, index + 1
        )
    recorder.close()
    report = encode_video(tmp_path, fps=10)
    assert report['source_frames'] == 3
    assert report['encoded_frames'] == 5
    with av.open(report['video']) as video:
        frames = list(video.decode(video=0))
    assert len(frames) == 5
    # Missing times hold the previous sample, not the future sample.
    assert frames[2].to_ndarray(format='rgb24').mean() == pytest.approx(100, abs=5)
    assert frames[4].to_ndarray(format='rgb24').mean() == pytest.approx(200, abs=5)


def test_named_camera_view_video(tmp_path):
    pytest.importorskip('av')
    recorder = SensorRecorder(tmp_path, {'record': True})
    for index, timestamp in enumerate([0.0, 0.1]):
        recorder.capture(
            {
                'head_camera_rgb': np.full((16, 16, 3), index * 100, dtype=np.uint8),
                'observer_camera_rgb': np.full((16, 16, 3), 200, dtype=np.uint8),
            },
            timestamp,
            index + 1,
        )
    recorder.close()
    report = encode_video(
        tmp_path, tmp_path / 'head_camera.mp4', fps=10, view='head_camera'
    )
    assert report['view'] == 'head_camera'
    assert Path(report['video']).is_file()


def test_video_rejects_depth_only_without_creating_fake_rgb(tmp_path):
    pytest.importorskip('av')
    recorder = SensorRecorder(tmp_path, {'record': True})
    recorder.capture({'depth': np.ones((8, 8))}, 0, 1)
    recorder.close()
    with pytest.raises(ValueError, match='RGB recording'):
        encode_video(tmp_path)
    assert not (tmp_path / 'video.mp4').exists()


def test_camera_recorder_ignores_lidar_arriving_out_of_time_order(tmp_path):
    recorder = SensorRecorder(tmp_path, {'record': True})
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    points = np.ones((4, 3), dtype=np.float32)
    recorder.capture_telemetry({
        'sensors': {'d435i_rgb': rgb, 'mid360_points': points},
        'sensor_time': .4, 'camera_time': .2, 'camera_frame_id': 1,
        'sensor_times': {'d435i_rgb': .2, 'mid360_points': .4},
    })
    # A later-arriving LiDAR capture is older than the last camera capture.
    recorder.capture_telemetry({
        'sensors': {'mid360_points': points}, 'sensor_time': .1,
        'camera_time': .2, 'camera_frame_id': 1,
    })
    recorder.capture_telemetry({
        'sensors': {'d435i_rgb': rgb, 'mid360_points': points},
        'sensor_time': .1, 'camera_time': .3, 'camera_frame_id': 2,
    })
    assert recorder.close()['observed_frames'] == 2
    with np.load(tmp_path / 'camera_latest.npz') as frame:
        assert frame['timestamp'] == .3
        assert 'd435i_rgb' in frame
        assert 'mid360_points' not in frame


def test_camera_recorder_uses_modality_time_for_legacy_telemetry(tmp_path):
    recorder = SensorRecorder(tmp_path, {'record': False})
    recorder.capture_telemetry({
        'sensors': {'rgb': np.zeros((2, 2, 3), dtype=np.uint8)},
        'sensor_time': 5.0, 'sensor_times': {'rgb': 1.0}, 'camera_frame_id': 1,
    })
    assert recorder.last_timestamp == 1.0
