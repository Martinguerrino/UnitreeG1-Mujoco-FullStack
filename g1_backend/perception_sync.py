"""Bounded latest-ready RGB-D queue; TF readiness is supplied by the ROS node."""

from dataclasses import dataclass
import math


@dataclass
class Capture:
    stamp: float
    received: float
    pair: object
    ready: bool = False


class CaptureQueue:
    def __init__(self, capacity=8, max_age=0.5, tf_wait=0.3):
        if capacity < 1 or not all(math.isfinite(v) and v > 0 for v in [max_age, tf_wait]):
            raise ValueError('Capture queue limits must be positive and finite')
        self.capacity, self.max_age, self.tf_wait = capacity, max_age, tf_wait
        self.queue = []
        self.last_processed = -math.inf
        self.last_now = None
        self.dropped = {'capacity': 0, 'stale': 0, 'tf_timeout': 0, 'superseded': 0}

    def push(self, pair, stamp, wall):
        if not math.isfinite(stamp):
            raise ValueError('Capture timestamp must be finite')
        self.queue = [item for item in self.queue if item.stamp != stamp]
        self.queue.append(Capture(stamp, wall, pair))
        self.queue.sort(key=lambda item: item.stamp)
        if len(self.queue) > self.capacity:
            self.dropped['capacity'] += len(self.queue) - self.capacity
            self.queue = self.queue[-self.capacity :]

    def select(self, now, wall, ready, consume=True):
        if self.last_now is not None and now < self.last_now:
            self.queue.clear()
            self.last_processed = -math.inf
        self.last_now = now
        retained = []
        for capture in self.queue:
            if capture.stamp <= self.last_processed:
                self.dropped['superseded'] += 1
            elif now - capture.stamp > self.max_age:
                self.dropped['stale'] += 1
            elif not capture.ready and wall - capture.received > self.tf_wait:
                self.dropped['tf_timeout'] += 1
            else:
                if not capture.ready and capture.stamp <= now + 0.02:
                    capture.ready = ready(capture.pair, capture.stamp)
                retained.append(capture)
        self.queue = retained
        chosen = next(
            (c for c in reversed(retained) if c.stamp <= now + 0.02 and c.ready),
            None,
        )
        if chosen is None or not consume:
            return chosen
        self.dropped['superseded'] += sum(c.stamp < chosen.stamp for c in retained)
        self.queue = [c for c in retained if c.stamp > chosen.stamp]
        self.last_processed = chosen.stamp
        return chosen
