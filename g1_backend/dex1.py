"""Official Dex1 URDF geometry and separate SDK2 motor topics.

The prismatic model is a local MJCF conversion, not an official MJCF release.
Motor radians follow Unitree's convention (0 closed, 5.4 open). The official
URDF slide axes have the opposite opening direction to the Isaac Lab model.
"""

import hashlib
import json
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

from .config import ROOT

ASSETS = ROOT / 'artifacts/dex1'
SLOPE = 0.044 / 5.4


def motor_to_slide(q):
    if not np.isfinite(q).all():
        raise ValueError('Nonfinite Dex1 command')
    return -0.02 + SLOPE * np.clip(q, 0, 5.4)


def slide_to_motor(q):
    # Continuous inverse: unlike upstream display rounding, retain sensor precision.
    return np.clip((np.asarray(q) + 0.02) / SLOPE, 0, 5.4)


def verify_assets():
    manifest = json.loads((ROOT / 'configs/dex1.sources.json').read_text())
    for name, entry in manifest.items():
        if hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError(f'Dex1 asset checksum mismatch: {name}')


def add_dex1(root):
    verify_assets()
    # Reduce soft-contact friction drift in grasping; retain physical contacts.
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('cone', 'elliptic')
    option.set('impratio', '10')
    urdf = ET.parse(ASSETS / 'g1_29dof_mode_15_with_dex1_1.urdf').getroot()
    assets = root.find('asset')
    for name in (
        'Dex1_base_link.STL',
        'Dex1_finger_link_1.STL',
        'Dex1_finger_link_2.STL',
        'dex1_col_1.stl',
        'dex1_col_2.stl',
    ):
        ET.SubElement(assets, 'mesh', name=name, file=str(ASSETS / name))
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root, 'contact')
    for side in ('left', 'right'):
        wrist = root.find(f".//body[@name='{side}_wrist_yaw_link']")
        for geom in list(wrist.findall('geom')):
            if geom.get('mesh') == f'{side}_rubber_hand':
                wrist.remove(geom)
        base = None
        for suffix in ('base', 'finger_link_1', 'finger_link_2'):
            name = f'{side}_dex1_' + ('base_link' if suffix == 'base' else suffix)
            link = urdf.find(f"link[@name='{name}']")
            uj = next(j for j in urdf.findall('joint') if j.find('child').get('link') == name)
            origin = uj.find('origin')
            body = ET.SubElement(
                wrist if suffix == 'base' else base,
                'body',
                name=name,
                pos=origin.get('xyz'),
                euler=origin.get('rpy'),
            )
            inertial = link.find('inertial')
            inertia = inertial.find('inertia')
            ET.SubElement(
                body,
                'inertial',
                pos=inertial.find('origin').get('xyz'),
                mass=inertial.find('mass').get('value'),
                fullinertia=' '.join(
                    inertia.get(k) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')
                ),
            )
            for kind in ('visual', 'collision'):
                mesh = link.find(f'{kind}/geometry/mesh').get('filename').split('/')[-1]
                ET.SubElement(
                    body,
                    'geom',
                    name=f'{name}_{kind}',
                    type='mesh',
                    mesh=mesh,
                    contype='0' if kind == 'visual' else '1',
                    conaffinity='0' if kind == 'visual' else '1',
                    group='1' if kind == 'visual' else '3',
                    density='0',
                    rgba='.65 .68 .72 1',
                    friction='1 .01 .001',
                    condim='4',
                )
            if suffix == 'base':
                base = body
            else:
                limit = uj.find('limit')
                ET.SubElement(
                    body,
                    'joint',
                    name=uj.get('name'),
                    type='slide',
                    axis=uj.find('axis').get('xyz'),
                    range=f'{limit.get("lower")} {limit.get("upper")}',
                    damping='2',
                    armature='.005',
                )
                ET.SubElement(
                    root.find('actuator'),
                    'motor',
                    name=uj.get('name'),
                    joint=uj.get('name'),
                    ctrllimited='true',
                    ctrlrange='-20 20',
                )
                # Adjacent meshes overlap mechanically at the guide; no finger-base contact.
                ET.SubElement(contact, 'exclude', body1=base.get('name'), body2=name)
    # Body actuators and sensor order stay unchanged; hands have their own DDS interface.


class GripperDds:
    """Client or simulation server. ChannelFactory must already be initialized."""

    def __init__(self, *, server=False, timeout=0.25):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

        self.server, self.timeout = server, timeout
        self.lock = threading.Lock()
        self.values = [None, None]
        self.received = [0.0, 0.0]
        self.invalid = 0
        self.publishers, self.subscribers = [], []
        for i, side in enumerate(('left', 'right')):
            topic = f'rt/dex1/{side}'
            pub = ChannelPublisher(
                topic + ('/state' if server else '/cmd'), MotorStates_ if server else MotorCmds_
            )
            pub.Init()
            sub = ChannelSubscriber(
                topic + ('/cmd' if server else '/state'), MotorCmds_ if server else MotorStates_
            )
            sub.Init(lambda msg, index=i: self._receive(index, msg), 10)
            self.publishers.append(pub)
            self.subscribers.append(sub)

    def _receive(self, index, message):
        values = message.cmds if self.server else message.states
        with self.lock:
            if len(values) != 1 or not np.isfinite(values[0].q):
                self.invalid += 1
                return
            self.values[index] = float(values[0].q)
            self.received[index] = time.monotonic()

    def read(self):
        with self.lock:
            if any(v is None for v in self.values) or any(
                time.monotonic() - t > self.timeout for t in self.received
            ):
                raise TimeoutError('Dex1 command/state missing or stale')
            return np.array(self.values)

    def send(self, q):
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__MotorCmd_,
            unitree_go_msg_dds__MotorState_,
        )

        q = np.asarray(q)
        if q.shape != (2,) or not np.isfinite(q).all():
            raise ValueError('Expected two finite Dex1 positions')
        for i in range(2):
            if self.server:
                msg = MotorStates_()
                motor = unitree_go_msg_dds__MotorState_()
                motor.q = float(q[i])
                msg.states = [motor]
            else:
                msg = MotorCmds_()
                motor = unitree_go_msg_dds__MotorCmd_()
                motor.q = float(np.clip(q[i], 0, 5.4))
                motor.kp, motor.kd = 5.0, 0.05
                msg.cmds = [motor]
            self.publishers[i].Write(msg)

    def close(self):
        for channel in self.subscribers + self.publishers:
            channel.Close()


class GripperPlant:
    """Bounded force/velocity servo for the URDF's four prismatic finger joints."""

    def __init__(self, model):
        self.qadr, self.vadr, self.act = [], [], []
        for side in ('left', 'right'):
            for finger in (1, 2):
                name = f'{side}_dex1_finger_joint_{finger}'
                joint = model.joint(name)
                self.qadr.append(joint.qposadr[0])
                self.vadr.append(joint.dofadr[0])
                self.act.append(model.actuator(name).id)
        self.qadr, self.vadr, self.act = map(np.array, (self.qadr, self.vadr, self.act))

    def state(self, data):
        return slide_to_motor(data.qpos[self.qadr].reshape(2, 2).mean(axis=1))

    def encoder_state(self, data):
        # Match Unitree Isaac Lab's state publisher: round slide position to 1 mm,
        # then motor radians to 0.001. This also represents a closed stop as zero.
        slide = data.qpos[self.qadr].reshape(2, 2).mean(axis=1)
        return np.round(slide_to_motor(np.round(slide, 3)), 3)

    def advance(self, data, target):
        velocity = data.qvel[self.vadr]
        if target is None:
            force = -20 * velocity
        else:
            error = np.repeat(motor_to_slide(target), 2) - data.qpos[self.qadr]
            desired_velocity = np.clip(40 * error, -0.2, 0.2)
            force = 60 * (desired_velocity - velocity)
        data.ctrl[self.act] = np.clip(force, -20, 20)
