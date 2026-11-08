"""Probe NVENC with an actual frame before choosing it over libx264."""

import io
from functools import lru_cache
from ..acceleration import enabled, _DISABLED


@lru_cache(maxsize=1)
def _nvenc_works():
    try:
        import av

        with av.open(io.BytesIO(), mode='w', format='mp4') as container:
            stream = container.add_stream('h264_nvenc', rate=30)
            stream.width, stream.height, stream.pix_fmt = 64, 64, 'yuv420p'
            frame = av.VideoFrame(64, 64, 'yuv420p')
            for plane in frame.planes:
                plane.update(bytes(plane.buffer_size))
            for packet in stream.encode(frame):
                container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
        return True
    except Exception:  # noqa: BLE001 - codecs differ between FFmpeg builds
        return False


def codec():
    return 'h264_nvenc' if enabled() and 'video' not in _DISABLED and _nvenc_works() else 'libx264'
