"""Optional placement velocity measurement from physical position history."""

from collections import deque

import numpy as np

from .transfer import TransferController


class PositionSpeedWindow:
    def __init__(self, duration):
        if not np.isfinite(duration) or not 0 < duration <= .5:
            raise ValueError('Position window must be in (0, 0.5] seconds')
        self.duration = duration
        self.history = deque()

    def measure(self, now, position, fallback):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('Position requires three finite coordinates')
        if not self.history or now > self.history[-1][0]:
            self.history.append((now, position.copy()))
        while len(self.history) > 1 and now - self.history[1][0] >= self.duration:
            self.history.popleft()
        then, previous = self.history[0]
        elapsed = now - then
        if elapsed < self.duration - 1e-8:
            return float(fallback)
        return float(np.linalg.norm(position - previous) / elapsed)


class WindowedTransferController(TransferController):
    """Experimental alternative; original TransferController remains unchanged."""

    def __init__(self, model, data, config, *, window=.1):
        super().__init__(model, data, config)
        self.speed_window = PositionSpeedWindow(window)

    def observe(self, state, grippers):
        observation = super().observe(state, grippers)
        observation['instantaneous_object_speed'] = observation['object_speed']
        observation['object_speed'] = self.speed_window.measure(
            float(self.data.time), self.data.xpos[self.body], observation['object_speed']
        )
        self.last_observation = observation
        # TransferTask accepts the original fields only.
        return {key: value for key, value in observation.items()
                if key != 'instantaneous_object_speed'}
