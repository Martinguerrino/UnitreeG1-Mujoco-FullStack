"""Guarded ACT pick -> closed-gripper carry -> table release sequencing.

The observer supplies physical contacts; task completion never attaches the object.
"""

import numpy as np


class TransferTask:
    def __init__(self, pick_timeout=25.0, carry_timeout=30.0):
        if not all(np.isfinite(t) and t > 0 for t in (pick_timeout, carry_timeout)):
            raise ValueError('Task deadlines must be positive and finite')
        self.phase = 'pick'
        self.started = 0.0
        self.stable_since = None
        self.lost_since = None
        self.arm_hold = None
        self.events = []
        self.failure = None
        self.timeouts = {
            'pick': pick_timeout,
            'hold': 3.0,
            'stand': 6.0,
            'carry': carry_timeout,
            'lower': 12.0,
            'release': 5.0,
        }

    @property
    def finished(self):
        return self.phase in ('done', 'failed')

    @property
    def act_enabled(self):
        return self.phase == 'pick'

    @property
    def gripper_override(self):
        if self.phase == 'pick':
            return None
        return 5.4 if self.phase in ('release', 'done') else 0.0

    def _transition(self, phase, now, reason=None):
        self.events.append({'from': self.phase, 'to': phase, 'time': now, 'reason': reason})
        self.phase, self.started = phase, now
        self.stable_since = self.lost_since = None

    def fail(self, now, reason):
        if not self.finished:
            self.failure = reason
            self._transition('failed', now, reason)

    def _stable(self, condition, now, duration):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= duration

    def update(
        self,
        now,
        *,
        arms,
        bilateral,
        lift,
        fallen=False,
        arrived=False,
        table_contact=False,
        over_table=False,
        release_ready=False,
        object_speed=0.0,
        hand_contact=None,
        gripper_open=False,
    ):
        if self.finished:
            return
        if fallen:
            self.fail(now, 'robot_fell')
            return
        if now - self.started >= self.timeouts[self.phase]:
            self.fail(now, f'{self.phase}_timeout')
            return
        if self.phase in ('hold', 'stand', 'carry', 'lower'):
            supported = self.phase == 'lower' and table_contact and over_table
            if not bilateral and not supported:
                if self.lost_since is None:
                    self.lost_since = now
                if now - self.lost_since >= 0.15:
                    self.fail(now, 'object_lost')
                    return
            else:
                self.lost_since = None
        if self.phase == 'pick':
            if self._stable(bilateral and lift >= 0.03, now, 0.3):
                self.arm_hold = np.asarray(arms, dtype=float).copy()
                if self.arm_hold.shape != (14,) or not np.isfinite(self.arm_hold).all():
                    raise ValueError('Expected 14 finite arm positions')
                self._transition('hold', now)
        elif self.phase == 'hold':
            if self._stable(bilateral, now, 1.0):
                self._transition('stand', now)
        elif self.phase == 'stand':
            if bilateral and now - self.started >= 2.0:
                self._transition('carry', now)
        elif self.phase == 'carry':
            if self._stable(arrived, now, 1.0):
                self._transition('lower', now)
        elif self.phase == 'lower':
            if over_table and release_ready:
                self._transition('release', now)
        elif self.phase == 'release':
            touching = bilateral if hand_contact is None else hand_contact
            if self._stable(
                table_contact
                and over_table
                and not touching
                and gripper_open
                and object_speed < 0.05,
                now,
                0.5,
            ):
                self._transition('done', now)
