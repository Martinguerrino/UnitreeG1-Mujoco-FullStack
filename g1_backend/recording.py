"""Record actual sensor samples; encode video offline using simulation timestamps."""

from fractions import Fraction
import json
import math
from pathlib import Path

import numpy as np

from .evaluation import write_json


class SensorRecorder:
    def __init__(self, directory, config):
        self.directory = Path(directory)
        self.enabled = config.get('record', False)
        self.limit = config.get('max_recorded_frames', 10000)
        self.last_timestamp = None
        self.latest = None
        self.saved = 0
        self.observed = 0
        self.last_frame_id = 0
        self.missing = 0
        self.index = None
        if self.enabled:
            (self.directory / 'trajectory/camera').mkdir(parents=True)
            self.index = (self.directory / 'trajectory/camera/index.jsonl').open('w')

    def capture(self, sensors, timestamp, frame_id):
        if not sensors or timestamp == self.last_timestamp:
            return
        if not math.isfinite(timestamp) or (
            self.last_timestamp is not None and timestamp < self.last_timestamp
        ):
            raise ValueError('Non-monotonic camera timestamps')
        self.last_timestamp = timestamp
        self.missing += max(0, frame_id - self.last_frame_id - 1)
        self.last_frame_id = frame_id
        self.latest = sensors
        self.observed += 1
        if self.enabled and self.saved < self.limit:
            relative = f'trajectory/camera/{self.saved:06d}.npz'
            np.savez(self.directory / relative, timestamp=timestamp, **sensors)
            self.index.write(
                json.dumps({'file': relative, 'simulation_time': timestamp, 'frame_id': frame_id})
                + '\n'
            )
            self.saved += 1

    def close(self):
        if self.index:
            self.index.close()
        if self.latest:
            np.savez(
                self.directory / 'camera_latest.npz', timestamp=self.last_timestamp, **self.latest
            )
        result = {
            'recorded_frames': self.saved,
            'observed_frames': self.observed,
            'unobserved_frames': self.missing,
            'recording_truncated': self.enabled and self.observed > self.saved,
        }
        if self.enabled:
            write_json(self.directory / 'recording.json', result)
        return result


def encode_video(directory, output=None, fps=30, view=None):
    """Zero-order hold of recorded RGB samples, preserving simulation-time spacing.

    Video is an export of observations, never a new physics replay or success test.
    """
    import av

    directory = Path(directory).resolve()
    if not math.isfinite(fps) or not 0 < fps <= 240:
        raise ValueError('Video fps must be in (0, 240]')
    rows = [
        json.loads(line)
        for line in (directory / 'trajectory/camera/index.jsonl').read_text().splitlines()
    ]
    if not rows:
        raise ValueError('No recorded camera frames')
    timestamps = np.array([row['simulation_time'] for row in rows])
    if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError('Recorded timestamps must be finite and strictly increasing')

    def read(row):
        path = (directory / row['file']).resolve()
        if not path.is_relative_to(directory):
            raise ValueError('Frame path is outside the experiment directory')
        with np.load(path, allow_pickle=False) as data:
            key = f'{view}_rgb' if view else 'rgb'
            if key not in data:
                raise ValueError(
                    f'RGB recording requires key {key!r}; depth is not converted implicitly'
                )
            frame = data[key]
            if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[-1] != 3:
                raise ValueError('Expected uint8 HxWx3 RGB')
            if float(data['timestamp']) != row['simulation_time']:
                raise ValueError('Frame timestamp disagrees with index')
            return frame

    first = read(rows[0])
    height, width, _ = first.shape
    if width % 2 or height % 2:
        raise ValueError('MP4 yuv420p export requires even width and height')
    output = Path(output) if output else directory / 'video.mp4'
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix('.partial.mp4')
    rate = Fraction(str(fps)).limit_denominator(1000)
    total = max(1, math.ceil((timestamps[-1] - timestamps[0]) * float(rate)) + 1)
    source_index = -1
    current = None
    try:
        with av.open(str(temporary), mode='w') as container:
            stream = container.add_stream('libx264', rate=rate)
            stream.width, stream.height = width, height
            stream.pix_fmt = 'yuv420p'
            for index in range(total):
                sample_time = timestamps[0] + index / float(rate)
                selected = int(np.searchsorted(timestamps, sample_time, side='right') - 1)
                selected = min(max(selected, 0), len(rows) - 1)
                if selected != source_index:
                    current = read(rows[selected])
                    if current.shape != first.shape:
                        raise ValueError('Camera resolution changed during recording')
                    source_index = selected
                frame = av.VideoFrame.from_ndarray(current, format='rgb24')
                frame.pts = index
                frame.time_base = 1 / rate
                for packet in stream.encode(frame):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
        temporary.replace(output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    report = {
        'video': str(output),
        'view': view,
        'source_frames': len(rows),
        'encoded_frames': total,
        'fps': float(rate),
        'simulation_start': timestamps[0],
        'simulation_end': timestamps[-1],
        'duration': total / float(rate),
        'sampling': 'hold last recorded RGB sample; simulation time',
        'pyav_version': av.__version__,
    }
    write_json(output.with_suffix('.json'), report)
    return report
