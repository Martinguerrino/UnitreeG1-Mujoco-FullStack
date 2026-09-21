"""Physical observer and command composition for the experimental Okura transfer.

ACT owns the pick. After a verified lift, arm targets are latched and the right
Dex1 stays closed. The existing locomotion network controls legs/waist; carrying
with held arms is an experimental composition, not a trained carrying policy.
"""

from pathlib import Path
import json

import mujoco
import numpy as np
import yaml

from g1_backend.config import ROOT
from g1_backend.contracts import JointCommand
from g1_backend.policies.locomotion import UnitreeVelocityPolicy, projected_gravity
from g1_backend.skills import GoTo
from g1_backend.skills.transfer import TransferTask
from .checkpoint import STATE_NAMES


def load_transfer_config(path):
    config = yaml.safe_load(Path(path).read_text())
    if config['object_kind'] != 'okra':
        raise ValueError('The transfer task reproduces the Okura object')
    for name, length in [('object_position', 3), ('carry_target', 3), ('initial_right_state', 8)]:
        values = np.asarray(config[name])
        if values.shape != (length,) or not np.isfinite(values).all():
            raise ValueError(f'Invalid {name}')
    if not 0 < config['carry_speed'] <= 0.3:
        raise ValueError('Experimental carrying speed must be in (0, .3]')
    waypoints = np.asarray(config.get('carry_waypoints', [config['carry_target']]))
    if (
        waypoints.ndim != 2
        or waypoints.shape[1] != 3
        or not len(waypoints)
        or not np.isfinite(waypoints).all()
    ):
        raise ValueError('Expected finite x/y/yaw carrying waypoints')
    if not np.allclose(waypoints[-1], config['carry_target']):
        raise ValueError('Last waypoint must match the table approach goal')
    TransferTask(config['pick_timeout'], config['carry_timeout'])
    return config


class TransferController:
    def __init__(self, model, data, config):
        self.config = config
        self.model, self.data = model, data
        self.task = TransferTask(config['pick_timeout'], config['carry_timeout'])
        self.locomotion = UnitreeVelocityPolicy(
            yaml.safe_load((ROOT / 'configs/locomotion.yaml').read_text())
        )
        self.waypoints = config.get('carry_waypoints', [config['carry_target']])
        self.waypoint_index = 0
        self.goto = GoTo(self.waypoints[0], speed=config['carry_speed'])
        self.arms = np.array(
            [
                next(i for i in range(29) if model.joint(model.actuator_trnid[i, 0]).name == name)
                for name in STATE_NAMES[:14]
            ]
        )
        self.arm_dofs = np.array([model.joint(name).dofadr[0] for name in STATE_NAMES[7:14]])
        self.arm_limits = np.array([model.joint(name).range for name in STATE_NAMES[7:14]])
        self.fixture = model.equality('pelvis_fixture').id
        self.body = model.body('box').id
        self.object_geoms = set(np.flatnonzero(model.geom_bodyid == self.body))
        self.table = model.geom('table_b').id
        self.initial_z = float(data.xpos[self.body, 2])
        self.next_policy = 0.0
        self.released_fixture = False
        self.navigation_started = False
        self.last_observation = None
        self.phases_executed = set()
        self.arm_target = None
        self.lower_target = None
        self.command = None
        self.wrist = model.body('right_wrist_yaw_link').id
        self.free_dof = model.joint('box_free').dofadr[0]

    def initialize_body(self):
        q = self.locomotion.get_action().q
        for i in range(15):
            joint = self.model.actuator_trnid[i, 0]
            self.data.qpos[self.model.jnt_qposadr[joint]] = q[i]
        # Reset only; all subsequent motion is integrated through ctrl.
        mujoco.mj_forward(self.model, self.data)

    def observe(self, state, grippers):
        data, model = self.data, self.model
        fingers = set()
        supported = False
        for i, contact in enumerate(data.contact):
            pair = {int(contact.geom1), int(contact.geom2)}
            if not pair & self.object_geoms or contact.efc_address < 0:
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            if abs(force[0]) < 0.01:
                continue
            supported |= self.table in pair
            for geom in pair - self.object_geoms:
                name = model.body(model.geom_bodyid[geom]).name
                if name.startswith('right_dex1_finger_link_'):
                    fingers.add(name)
        position = data.xpos[self.body]
        table_pos = data.geom_xpos[self.table]
        over = bool(
            np.all(np.abs(position[:2] - table_pos[:2]) < model.geom_size[self.table, :2] - 0.06)
        )
        top = table_pos[2] + model.geom_size[self.table, 2]
        geom = model.geom('box_collision').id
        rotation = data.geom_xmat[geom].reshape(3, 3)
        vertical_radius = np.sqrt(np.sum((rotation[2] * model.geom_size[geom]) ** 2))
        bottom = position[2] - vertical_radius
        telemetry = {'position': data.qpos[:3].copy(), 'quaternion': data.qpos[3:7].copy()}
        reached = self.goto.is_done(telemetry)
        if (
            self.task.phase == 'carry'
            and reached
            and self.waypoint_index < len(self.waypoints) - 1
        ):
            self.waypoint_index += 1
            self.goto = GoTo(self.waypoints[self.waypoint_index], speed=self.config['carry_speed'])
            reached = False
        arrived = (
            reached
            and self.waypoint_index == len(self.waypoints) - 1
            and np.linalg.norm(data.qvel[:3]) < 0.08
        )
        observation = {
            'arms': state.q[self.arms],
            'bilateral': len(fingers) == 2,
            'hand_contact': bool(fingers),
            'lift': float(position[2] - self.initial_z),
            'fallen': bool(data.qpos[2] < 0.45 or projected_gravity(state.quaternion)[2] > -0.54),
            'arrived': bool(arrived),
            'table_contact': bool(supported),
            'over_table': over,
            'release_ready': bool(over and (supported or top - 0.005 <= bottom <= top + 0.008)),
            'object_speed': float(np.linalg.norm(data.qvel[self.free_dof : self.free_dof + 3])),
            'gripper_open': bool(grippers[1] > 4.5),
        }
        self.last_observation = observation
        return observation

    def update(self, state, grippers):
        self.task.update(float(self.data.time), **self.observe(state, grippers))
        if (
            self.task.phase != 'pick'
            and self.arm_target is None
            and self.task.arm_hold is not None
        ):
            self.arm_target = self.task.arm_hold.copy()
        self.phases_executed.add(self.task.phase)

    def compose(self, state, body_command, grippers):
        phase, now = self.task.phase, float(self.data.time)
        if phase == 'pick':
            return body_command, grippers
        grippers = grippers.copy()
        grippers[1] = self.task.gripper_override
        if phase == 'failed':
            return JointCommand(state.q.copy(), body_command.kp, body_command.kd), grippers
        if self.arm_target is None:
            raise RuntimeError('Carry has no verified grasp pose')
        body_command.q[self.arms] = self.arm_target
        if phase == 'hold':
            return body_command, grippers
        if phase in ('stand', 'carry', 'lower', 'release', 'done'):
            if not self.released_fixture:
                self.data.eq_active[self.fixture] = 0
                self.released_fixture = True
                self.locomotion.reset()
            if now >= self.next_policy:
                telemetry = {
                    'position': self.data.qpos[:3].copy(),
                    'quaternion': self.data.qpos[3:7].copy(),
                }
                velocity = self.goto.velocity(telemetry) if phase == 'carry' else np.zeros(3)
                self.navigation_started |= bool(np.linalg.norm(velocity) > 0)
                self.locomotion.infer(self.locomotion.get_observation(state, velocity))
                self.command = self.locomotion.get_action()
                self.next_policy = now + self.locomotion.period
            command = JointCommand(
                self.command.q.copy(), self.command.kp.copy(), self.command.kd.copy()
            )
            if phase == 'lower':
                self._lower_arm()
            command.q[self.arms] = self.arm_target
            command.kp[self.arms] = body_command.kp[self.arms]
            command.kd[self.arms] = body_command.kd[self.arms]
            return command, grippers
        return body_command, grippers

    def _lower_arm(self):
        # Cartesian target lowers at 2 cm/s; Jacobian updates targets, never qpos.
        if self.lower_target is None:
            self.lower_target = self.data.xpos[self.wrist].copy()
        if self.last_observation['over_table'] and not self.last_observation['release_ready']:
            self.lower_target[2] -= 0.02 * self.model.opt.timestep
        jac = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jac, None, self.wrist)
        J = jac[:, self.arm_dofs]
        error = np.clip(self.lower_target - self.data.xpos[self.wrist], -0.02, 0.02)
        change = J.T @ np.linalg.solve(J @ J.T + 0.01 * np.eye(3), error)
        self.arm_target[7:] = np.clip(
            self.arm_target[7:] + np.clip(change, -0.003, 0.003),
            self.arm_limits[:, 0],
            self.arm_limits[:, 1],
        )

    def report(self):
        return {
            'phase': self.task.phase,
            'success': self.task.phase == 'done',
            'failure': self.task.failure,
            'events': self.task.events,
            'fixture_released': self.released_fixture,
            'navigation_started': self.navigation_started,
            'waypoint_index': self.waypoint_index,
            'phases_executed': sorted(self.phases_executed),
            'carry_composition_validated': False,
            'uses_privileged_object_and_base_pose': True,
            'configuration': json.loads(json.dumps(self.config)),
        }
