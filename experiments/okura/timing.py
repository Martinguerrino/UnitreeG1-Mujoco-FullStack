"""Experimental timestamped ACT chunks; legacy scheduling remains the default."""

from collections import deque

import numpy as np


class ChunkTimeline:
    def __init__(self, rate=30.0, ensemble=False, decay=0.01):
        self.rate, self.ensemble, self.decay = rate, ensemble, decay
        self.chunks = deque(maxlen=8)

    def add(self, actions, observed_at):
        actions = np.asarray(actions)
        if actions.shape != (100, 16) or not np.isfinite(actions).all():
            raise ValueError('Expected a finite 100x16 ACT chunk')
        if not np.isfinite(observed_at):
            raise ValueError('Observation time must be finite')
        self.chunks.append((float(observed_at), actions.copy()))

    def action(self, now):
        candidates = []
        for observed_at, actions in self.chunks:
            age = (now - observed_at) * self.rate
            index = int(np.floor(age + 1e-7))
            if 0 <= index < len(actions):
                candidates.append((actions[index], age))
        if not candidates:
            return None
        if not self.ensemble:
            return candidates[-1][0].copy()
        weights = np.exp(-self.decay * np.array([age for _, age in candidates]))
        return np.average(np.stack([a for a, _ in candidates]), axis=0, weights=weights)
