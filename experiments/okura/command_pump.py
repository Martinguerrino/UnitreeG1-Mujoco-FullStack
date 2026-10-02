"""Optional single-writer command publication independent of ACT/render latency."""

import threading

import numpy as np

from g1_backend.contracts import JointCommand


class CommandPump:
    def __init__(self, body_send, hands_send, period=.01):
        self.body_send, self.hands_send = body_send, hands_send
        self.period = period
        self.lock = threading.Lock()
        self.latest = None
        self.enabled = True
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def update(self, command, grippers, *, enabled=True):
        command.validate()
        grippers = np.asarray(grippers)
        if grippers.shape != (2,) or not np.isfinite(grippers).all():
            raise ValueError('Expected two finite gripper commands')
        snapshot = JointCommand(command.q.copy(), command.kp.copy(), command.kd.copy())
        with self.lock:
            self.latest = snapshot, grippers.copy()
            self.enabled = enabled

    def publish_once(self):
        with self.lock:
            latest = self.latest if self.enabled else None
        if latest is not None:
            self.body_send(latest[0])
            self.hands_send(latest[1])

    def _run(self):
        try:
            while not self.stop.is_set():
                self.publish_once()
                self.stop.wait(self.period)
        except Exception as error:  # noqa: BLE001 - surface publisher failure to controller
            self.error = error

    def start(self):
        self.thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError('Asynchronous command publisher failed') from self.error

    def close(self):
        self.stop.set()
        if self.thread.ident is not None:
            self.thread.join()
