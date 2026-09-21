"""Small deterministic skills, independent of inference implementations."""

import math

import numpy as np


def yaw_from_quaternion(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class GoTo:
    def __init__(self, target, speed=0.3, position_tolerance=0.12, orientation_tolerance=0.15):
        self.target = np.asarray(target, dtype=float)
        if (
            self.target.shape != (3,)
            or not np.isfinite(self.target).all()
            or not math.isfinite(speed)
            or speed <= 0
        ):
            raise ValueError('GoTo expects finite world x, y, yaw and positive speed')
        self.speed = speed
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance

    def errors(self, telemetry):
        position = np.asarray(telemetry['position'][:2])
        yaw = yaw_from_quaternion(telemetry['quaternion'])
        return float(np.linalg.norm(self.target[:2] - position)), abs(
            wrap_angle(self.target[2] - yaw)
        )

    def is_done(self, telemetry):
        position, orientation = self.errors(telemetry)
        return position <= self.position_tolerance and orientation <= self.orientation_tolerance

    def velocity(self, telemetry):
        delta = self.target[:2] - telemetry['position'][:2]
        yaw = yaw_from_quaternion(telemetry['quaternion'])
        c, s = math.cos(yaw), math.sin(yaw)
        body_delta = np.array([[c, s], [-s, c]]) @ delta
        norm = np.linalg.norm(body_delta)
        # Keep the requested walking speed until entering the tolerance. Scaling
        # it down with distance can strand this policy below its walking deadband.
        velocity = body_delta * self.speed / max(norm, 1e-9)
        if norm <= self.position_tolerance:
            velocity[:] = 0
        return np.array([*velocity, np.clip(wrap_angle(self.target[2] - yaw), -0.2, 0.2)])


class Manipulate:
    def __init__(self, policy, request):
        self.policy = policy
        self.request = request
        self.policy.reset()

    def tick(self, state, sensors):
        observation = self.policy.observe({'images': sensors, 'request': self.request}, state)
        self.policy.infer(observation)
        return self.policy.get_action()


class SkillTimeout(TimeoutError):
    pass


class TaskExecutor:
    """Sequential skills with deadlines, settling, bounded retries and event history."""

    def __init__(self, steps, defaults=None):
        if not steps or any(step.get('skill') not in ('goto', 'pick', 'place') for step in steps):
            raise ValueError('Task requires goto/pick/place steps')
        defaults = defaults or {}
        self.steps = [{**defaults, **step} for step in steps]
        for step in self.steps:
            timeout = step.get('timeout', 20.0)
            settling = step.get('settle_duration', 1.0 if step['skill'] == 'goto' else 0.0)
            retries = step.get('max_retries', 0)
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError('Skill timeout must be finite and positive')
            if not math.isfinite(settling) or settling < 0 or settling >= timeout:
                raise ValueError('Settling must be nonnegative and shorter than skill timeout')
            if type(retries) is not int or retries < 0:
                raise ValueError('max_retries must be a nonnegative integer')
            step.update(timeout=timeout, settle_duration=settling, max_retries=retries)
        self.index = 0
        self.retries = 0
        self.attempt = 0
        self.started_at = None
        self.settled_since = None
        self.events = []

    @property
    def current(self):
        return None if self.index == len(self.steps) else self.steps[self.index]

    def _event(self, event, now, **details):
        self.events.append(
            {
                'event': event,
                'time': now,
                'index': self.index,
                'skill': self.current['skill'],
                'attempt': self.attempt,
                **details,
            }
        )

    def start(self, now):
        if self.current is not None and self.started_at is None:
            self.started_at = now
            self._event('started', now)

    def check_deadline(self, now):
        """Return True on retry; callers recreate/reset that skill, never the robot."""
        self.start(now)
        if self.current is None or now - self.started_at < self.current['timeout']:
            return False
        self._event('timeout', now, duration=now - self.started_at)
        if self.attempt >= self.current['max_retries']:
            raise SkillTimeout(f'Skill {self.index} ({self.current["skill"]}) timed out')
        self.retries += 1
        self.attempt += 1
        self.started_at = None
        self.settled_since = None
        self.start(now)
        return True

    def ready_to_complete(self, reached, now):
        if not reached:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = now
        return now - self.settled_since >= self.current['settle_duration']

    def complete_skill(self, now=None, **details):
        if self.current is not None:
            duration = None if now is None or self.started_at is None else now - self.started_at
            self._event('completed', now, duration=duration, **details)
            self.index += 1
            self.attempt = 0
            self.started_at = self.settled_since = None

    def abort(self, now, reason):
        if self.current is not None:
            self._event('failed', now, reason=reason)
