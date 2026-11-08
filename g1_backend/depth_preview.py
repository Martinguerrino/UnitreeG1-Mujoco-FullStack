"""Fixed-scale RGB depth preview; raw metric depth remains untouched."""

import cv2
import numpy as np


def depth_preview(depth, minimum=0.2, maximum=6.0, annotate=True):
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2 or not np.isfinite([minimum, maximum]).all() or not 0 < minimum < maximum:
        raise ValueError('Expected a depth image and positive ordered display limits')
    valid = np.isfinite(values) & (values >= minimum) & (values <= maximum)
    from .cuda.images import depth_colors

    rgb = depth_colors(values, minimum, maximum)
    if rgb is None:
        safe = np.where(valid, values, minimum)
        scaled = np.rint(np.clip((safe - minimum) / (maximum - minimum), 0, 1) * 255).astype(
            np.uint8
        )
        rgb = cv2.cvtColor(cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
        rgb[~valid] = [100, 0, 100]
    if annotate and values.shape[0] >= 40:
        cv2.rectangle(
            rgb, (0, values.shape[0] - 28), (values.shape[1], values.shape[0]), (20, 20, 20), -1
        )
        text = f'{minimum:g}-{maximum:g} m | sin lectura: {1 - valid.mean():.0%}'
        cv2.putText(
            rgb,
            text,
            (5, values.shape[0] - 9),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return rgb
