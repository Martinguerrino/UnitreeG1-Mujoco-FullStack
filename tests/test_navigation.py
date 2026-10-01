"""Navigation sensor geometry and loopback command boundary tests."""

import json
import math
import socket

import mujoco
import numpy as np

from g1_backend.config import ROOT, load_config
from g1_backend.navigation import NavigationTransport, laser_scan, lidar_scan
from g1_backend.simulator import model_xml


def test_scan_hits_wall_and_rotates_with_base():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base"><freejoint/><geom type="sphere" size="0.1"/></body>
      <geom type="box" group="2" pos="3 0 1" size="0.1 5 1"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scan = laser_scan(model, data, count=4)
    assert np.isclose(scan[2], 2.9)
    assert scan[0] is None
    data.qpos[3:7] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    mujoco.mj_forward(model, data)
    assert np.isclose(laser_scan(model, data, count=4)[1], 2.9)


def test_facility_scan_sees_workbench_top(tmp_path):
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['robot']['initial_position'] = [4.5, -6, 0.793]
    config['robot']['initial_yaw'] = math.pi / 2
    model_text, scene_text = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_text)
    (tmp_path / 'scene.xml').write_text(scene_text)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert np.isclose(laser_scan(model, data, count=4)[2], 0.95, atol=0.01)


def test_mid360_cloud_and_horizontal_projection_follow_mount(tmp_path):
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    model_text, scene_text = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_text)
    (tmp_path / 'scene.xml').write_text(scene_text)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    points, scan = lidar_scan(model, data, config['navigation']['lidar'])
    assert points.ndim == 2 and points.shape[1] == 3
    assert len(points) > 0
    assert len(scan) == config['navigation']['lidar']['horizontal_samples']
    assert any(distance is not None for distance in scan)
    assert np.isfinite(points).all()


def test_commands_are_bounded_reject_nan_and_expire():
    transport = NavigationTransport({'command_port': 0})
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        address = transport.socket.getsockname()
        sender.sendto(json.dumps({'velocity': [3, -3, 3]}).encode(), address)
        np.testing.assert_allclose(transport.velocity(), [0.55, -0.15, 1.0])
        transport.received -= 1
        sender.sendto(b'{"velocity": [NaN, 0, 0]}', address)
        np.testing.assert_array_equal(transport.velocity(), [0, 0, 0])
    finally:
        sender.close()
        transport.close()


def test_turn_assist_walks_sinusoidally_backwards_first_and_respects_lidar():
    transport = NavigationTransport({'command_port': 0, 'turn_assist': {
        'enabled': True, 'speed': 0.3, 'period': 1.5, 'acceleration': 0.4,
        'min_angular_speed': 0.1, 'max_linear_speed': 0.03,
        'obstacle_clearance': 0.8,
    }})
    # Nav2 commonly adds forward motion to a turn. The assist must still
    # replace it when rotation dominates the command.
    command = np.array([0.2, 0.0, 0.35])
    clear = {'time': 0.0, 'scan': [2.0] * 360}
    try:
        assert transport._assist_turn(command, clear)[0] == 0.0
        clear['time'] = 0.75
        assert np.isclose(transport._assist_turn(command, clear)[0], -0.3)
        clear['time'] = 1.5
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.0)
        clear['time'] = 2.25
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.3)
        blocked = [2.0] * 360
        blocked[180] = 0.5  # forward sector during the positive half-wave
        clear.update(time=2.3, scan=blocked)
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.0)
    finally:
        transport.close()


def test_slow_nav2_turn_keeps_phase_and_stops_on_zero():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        scan = [2.0] * 360
        outputs = []
        for tick in range(201):
            # Small angular commands observed in the user's navigation run,
            # including dips below the entry threshold.
            yaw = -0.005 if tick % 5 == 0 else -0.03
            outputs.append(transport._assist_turn(
                np.array([0.0, 0.0, yaw]), {'time': tick * 0.02, 'scan': scan}
            )[0])
        assert min(outputs) < -0.35
        assert max(outputs) > 0.35
        stopped = transport._assist_turn(np.zeros(3), {'time': 4.02, 'scan': scan})
        np.testing.assert_array_equal(stopped, np.zeros(3))
        assert transport.turn_started is None
    finally:
        transport.close()


def test_turn_boost_ramps_holds_and_reverses_with_nav2():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        def step(t, yaw):
            return transport._assist_turn(
                np.array([0.0, 0.0, yaw]), {'time': t, 'scan': [2.0] * 360}
            )[2]

        assert step(0.0, -0.03) == 0.0
        assert np.isclose(step(0.2, -0.03), -0.8)
        assert np.isclose(step(1.0, -0.03), -1.0)
        assert np.isclose(step(2.0, -0.002), -1.0)
        assert np.isclose(step(3.0, 0.03), 1.0)
        assert step(3.02, 0.0) == 0.0
    finally:
        transport.close()


def test_both_correction_phases_request_equal_turn_rate():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        samples = []
        for tick in range(201):
            t = tick * 0.02
            # Nav2 reduces its angular request during the forward half.
            command = np.array([0.0, 0.0, -0.03 if t < 2 else -0.002])
            out = transport._assist_turn(command, {'time': t, 'scan': [2.0] * 360})
            if tick in (50, 150):
                samples.append(out)
        assert samples[0][0] < 0 < samples[1][0]
        np.testing.assert_allclose(samples[0][2], samples[1][2])
        assert samples[0][2] == -1.0
    finally:
        transport.close()


def test_turn_assist_releases_when_nav2_starts_forward_path_following():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        telemetry = {'time': 0.0, 'scan': [2.0] * 360}
        transport._assist_turn(np.array([0.0, 0.0, 0.2]), telemetry)
        telemetry['time'] = 0.5
        command = np.array([0.1, 0.0, 0.2])
        np.testing.assert_allclose(
            transport._assist_turn(command, telemetry), [0.1, 0.0, 0.6]
        )
        assert transport.turn_started is None
        telemetry['time'] = 1.0
        np.testing.assert_allclose(
            transport._assist_turn(np.array([0.2, 0.0, 0.01]), telemetry),
            [0.2, 0.0, 0.03],
        )
        telemetry['time'] = 1.02
        np.testing.assert_array_equal(
            transport._assist_turn(np.zeros(3), telemetry), np.zeros(3)
        )
    finally:
        transport.close()


def test_sensor_mounts_match_urdf_and_camera_optical_axis(tmp_path):
    import xml.etree.ElementTree as ET

    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    urdf = ET.parse(ROOT / 'src/g1_description/urdf/g1_29dof.urdf.xacro')
    lidar = config['navigation']['lidar']
    camera = config['camera']['views'][0]
    for sensor, joint in ((lidar, 'mid360_joint'), (camera, 'd435_joint')):
        origin = urdf.find(f".//joint[@name='{joint}']/origin")
        np.testing.assert_allclose(sensor['position'], np.fromstring(origin.get('xyz'), sep=' '))
        if sensor is lidar:
            np.testing.assert_allclose(sensor['rpy'], np.fromstring(origin.get('rpy'), sep=' '))
    text, scene = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(text)
    (tmp_path / 'scene.xml').write_text(scene)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435i')
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso_link')
    pitch = float(urdf.find(".//joint[@name='d435_joint']/origin").get('rpy').split()[1])
    link_rotation = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                              [-math.sin(pitch), 0, math.cos(pitch)]])
    # ROS optical x=right, y=down, z=forward; MuJoCo x=right,y=up,z=back.
    optical_rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    expected = data.xmat[body_id].reshape(3, 3) @ link_rotation @ optical_rotation
    actual = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
    np.testing.assert_allclose(actual, expected, atol=1e-8)


def test_projection_bins_world_returns_and_rejects_ground():
    from g1_backend.navigation import project_scan

    cfg = {'horizontal_samples': 4, 'min_range': 0.05, 'max_range': 20,
               'scan_min_height': 0.15, 'scan_max_height': 1.5}
    cloud = np.array([[3, 0, 1], [2, 0, 0.4], [1, 0, 0], [0, 4, 1], [0, -1, 2]])
    assert project_scan(cloud, [0, 0, 0.8], 0, cfg) == [None, None, 2.0, 4.0]
    assert project_scan(cloud, [0, 0, 0.8], math.pi / 2, cfg) == [None, 2.0, 4.0, None]


def test_lidar_respects_self_occlusion_and_uses_planar_distance():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="torso"><freejoint/><geom type="sphere" size="0.1" pos="0 0 -1"/>
        <body name="arm"><geom type="box" size="0.1 0.3 0.3" pos="1 0 0"/></body>
      </body>
      <geom group="2" type="box" pos="3 0 1" size="0.1 5 2"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[2] = 1
    mujoco.mj_forward(model, data)
    cfg = {'body': 'torso', 'position': [0, 0, 0], 'rpy': [0, 0, 0],
               'horizontal_samples': 4, 'vertical_angles_deg': [0], 'min_range': 0.05, 'max_range': 20}
    _cloud, scan = lidar_scan(model, data, cfg)
    assert scan[2] is None  # Arm blocks the wall; it is not transparent.
    data.qpos[1] = 2  # Move arm and sensor together; remains occluded.
    mujoco.mj_forward(model, data)
    assert lidar_scan(model, data, cfg)[1][2] is None
    cfg['position'] = [0, 1, 0]
    cfg['rpy'] = [math.pi, 0.1, 0]  # Tilt / inversion must not reverse scan bins.
    _cloud, scan = lidar_scan(model, data, cfg)
    assert np.isclose(scan[2], math.hypot(2.9, 1.0), atol=1e-6)


def test_rgb_and_depth_render_same_frontal_surface():
    model = mujoco.MjModel.from_xml_string('''<mujoco><visual>
      <global offwidth="160" offheight="120"/></visual><worldbody>
      <camera name="rgbd" pos="0 0 1" quat="0.5 0.5 -0.5 -0.5" fovy="58"/>
      <light pos="0 0 3"/>
      <geom type="box" pos="3 0 1" size="0.1 0.5 0.5" rgba="1 0 0 1"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=120, width=160) as renderer:
        renderer.update_scene(data, camera='rgbd')
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
    red = (rgb[:, :, 0].astype(float) > 2*rgb[:, :, 1]+20)
    assert red.sum() > 500
    np.testing.assert_allclose(depth[red], 2.9, atol=1e-4)
    fy = 120/(2*math.tan(math.radians(58)/2))
    v, u = np.where(red)
    x = (u-79.5)*depth[red]/fy
    y = (v-59.5)*depth[red]/fy
    assert np.max(np.abs(x)) <= 0.52
    assert np.max(np.abs(y)) <= 0.52


def test_urdf_torso_chain_matches_mujoco_with_tilt_and_waist(tmp_path):
    import xml.etree.ElementTree as ET

    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    model_text, scene_text = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_text)
    (tmp_path / 'scene.xml').write_text(scene_text)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    data.qpos[:3] = [1, -2, 0.85]
    data.qpos[3:7] = [0.9, 0.1, -0.15, 0.3]
    data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
    for name, angle in [('waist_yaw_joint', 0.3), ('waist_roll_joint', -0.1),
                        ('waist_pitch_joint', 0.2)]:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = angle
    mujoco.mj_forward(model, data)
    urdf = ET.parse(ROOT / 'src/g1_description/urdf/g1_29dof.urdf.xacro')
    joints = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
    chain = []
    link = 'torso_link'
    while link != 'pelvis':
        chain.append(joints[link])
        link = joints[link].find('parent').get('link')
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
    transform = np.eye(4)
    transform[:3, :3] = data.xmat[pelvis].reshape(3, 3)
    transform[:3, 3] = data.xpos[pelvis]
    for joint in reversed(chain):
        origin = joint.find('origin')
        local = np.eye(4)
        local[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        angles = np.fromstring(origin.get('rpy'), sep=' ')
        rotation = np.eye(3)
        for axis, angle in reversed(list(zip(np.eye(3), angles))):
            quat = np.empty(4)
            matrix = np.empty(9)
            mujoco.mju_axisAngle2Quat(quat, axis, angle)
            mujoco.mju_quat2Mat(matrix, quat)
            rotation = rotation @ matrix.reshape(3, 3)
        local[:3, :3] = rotation
        transform = transform @ local
        if joint.get('type') != 'fixed':
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.get('name'))
            angle = data.qpos[model.jnt_qposadr[joint_id]]
            quat = np.empty(4)
            matrix = np.empty(9)
            mujoco.mju_axisAngle2Quat(quat, np.fromstring(joint.find('axis').get('xyz'), sep=' '), angle)
            mujoco.mju_quat2Mat(matrix, quat)
            transform[:3, :3] = transform[:3, :3] @ matrix.reshape(3, 3)
    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso_link')
    np.testing.assert_allclose(transform[:3, 3], data.xpos[torso], atol=1e-8)
    np.testing.assert_allclose(transform[:3, :3], data.xmat[torso].reshape(3, 3), atol=1e-8)


def test_cloud_transport_without_camera_keeps_acquisition_time():
    import io
    import struct

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(1)
    transport = NavigationTransport({'command_port': 0,
                                     'sensor_port': receiver.getsockname()[1]})
    row = {'position': [0, 0, 0.8], 'quaternion': [1, 0, 0, 0],
           'linear_velocity': [0, 0, 0], 'scan': None, 'scan_time': None,
           'torso_position_base': [0, 0, 0], 'torso_quaternion_base': [1, 0, 0, 0],
           'joint_names': [], 'joint_positions': [], 'camera_frame_id': 0,
           'sensors': {'mid360_points': np.ones((2, 3), np.float32)},
           'sensor_time': 9.0}
    sequences = []
    try:
        for t in (1.0, 1.1):
            row.update(time=t+0.02, sensor_times={'mid360_points': t})
            transport.publish(row)
            packet, _ = receiver.recvfrom(65535)
            sequence, index, count = struct.unpack('!IHH', packet[:8])
            sequences.append(sequence)
            assert (index, count) == (0, 1)
            with np.load(io.BytesIO(packet[8:]), allow_pickle=False) as values:
                assert float(values['time']) == t
                np.testing.assert_array_equal(values['mid360_points'], row['sensors']['mid360_points'])
        assert sequences[1] > sequences[0]
    finally:
        transport.close()
        receiver.close()
