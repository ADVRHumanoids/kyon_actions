#!/usr/bin/env python3
"""Replay saved joint motion through kyon_controller in MuJoCo without arms."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


from trajectory_io import JOINTS, joint_feedback, load_trajectory, package_directory


def payload_mass(value):
    mass = float(value)
    if not np.isfinite(mass) or mass < 0:
        raise argparse.ArgumentTypeError('Payload mass must be finite and nonnegative')
    return mass


def configure_payload(urdf, mass, position=None):
    if mass == 0:
        return None
    link = urdf.find("link[@name='payload_pelvis']")
    link.find('inertial/mass').set('value', str(mass))
    size = np.fromstring(link.find('visual/geometry/box').get('size'), sep=' ')
    inertia = mass / 12 * np.array([size[1]**2 + size[2]**2,
                                    size[0]**2 + size[2]**2,
                                    size[0]**2 + size[1]**2])
    for axis, value in zip(('ixx', 'iyy', 'izz'), inertia):
        link.find('inertial/inertia').set(axis, str(value))
    collision = ET.SubElement(link, 'collision')
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box',
                  size=' '.join(map(str, size)))
    origin = urdf.find("joint[@name='pelvis_to_payload']/origin")
    if position is None:
        position = list(map(float, origin.get('xyz').split()))
    origin.set('xyz', ' '.join(map(str, position)))
    return list(position)


def prepare(data, dt, output, payload_kg=0., payload_position=None):
    from ament_index_python.packages import get_package_share_directory as share
    from casadi_kin_dyn import pycasadi_kin_dyn

    model_dir = Path(share('kyon_urdf'))
    sim_dir = Path(share('kyon_mujoco'))
    args = ['upper_body:=false', 'wheels:=false', 'feet:=true',
            'ft_sensors:=true', 'varta:=true', f'payload:={str(payload_kg > 0).lower()}']
    urdf = ET.fromstring(subprocess.check_output(
        ['xacro', str(model_dir / 'urdf/kyon_robot.urdf.xacro'), *args]))
    urdf.find("joint[@name='reference']/origin").set('xyz', '0 0 0')
    payload_position = configure_payload(urdf, payload_kg, payload_position)
    urdf_text = ET.tostring(urdf, encoding='unicode')
    (output / 'robot.urdf').write_text(urdf_text)
    srdf = subprocess.check_output(['xacro', str(Path(share('kyon_srdf')) /
                                   'srdf/kyon.srdf.xacro'), *args]).decode()
    (output / 'robot.srdf').write_text(srdf)
    kd = pycasadi_kin_dyn.CasadiKinDyn(urdf_text)
    if kd.joint_names()[2:] != JOINTS:
        raise ValueError('Robot joint order does not match the saved leg trajectory')
    model_info = {'robot_mass_kg': float(kd.mass()),
                  'trajectory_robot_mass_kg': float(data['robot_mass'].item()),
                  'payload_kg': payload_kg, 'payload_position_m': payload_position,
                  'arms': False, 'feedforward_torques': False}
    (output / 'model.json').write_text(json.dumps(model_info, indent=2))
    print(f'Leg-only model: {kd.mass():.3f} kg. Full-body feedforward torques disabled.',
          flush=True)

    # The MAT positions describe contact centers; the collision spheres have radius 3 cm.
    foot_z = [float(kd.fk(f'contact_{i}')(q=data['q'][:, 0])['ee_pos'][2])
              for i in range(1, 5)]
    floor = float(np.mean(foot_z)) - 0.03
    step_x, step_z = (float(data[key].item()) for key in ('step_x', 'step_z'))
    world = ET.Element('mujoco')
    assets = ET.SubElement(world, 'asset')
    ET.SubElement(assets, 'texture', name='checker', type='2d', builtin='checker',
                  width='256', height='256', rgb1='.2 .25 .3', rgb2='.3 .35 .4')
    ET.SubElement(assets, 'material', name='floor', texture='checker', texrepeat='2 2')
    body = ET.SubElement(world, 'worldbody')
    ET.SubElement(body, 'light', pos='0 -3 5', dir='0 0 -1')
    ET.SubElement(body, 'geom', name='ground', type='plane', size='5 5 .05',
                  pos=f'0 0 {floor}', material='floor', friction='.8 .01 .001')
    ET.SubElement(body, 'geom', name='step', type='box', size=f'1.5 1.5 {step_z / 2}',
                  pos=f'{step_x + 1.5} 0 {floor + step_z / 2}',
                  rgba='.65 .65 .7 1', friction='.8 .01 .001')
    ET.SubElement(world, 'statistic', center='.7 0 .4', extent='2.5')
    ET.ElementTree(world).write(output / 'world.xml')

    with (output / 'prepare.log').open('w') as log:
        subprocess.run(['mujoco_simulator_wrapper.py', '--name', 'step_up',
                        '--urdf', str(output / 'robot.urdf'),
                        '--world', str(output / 'world.xml'),
                        '--simopt', str(sim_dir / 'config/options.xml'),
                        '--sites', str(sim_dir / 'config/sites.xml'),
                        '--output-dir', str(output / 'model'), '--skip-sim'],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    xml_path = output / 'model/step_up.xml'
    xml = ET.parse(xml_path)
    q0 = dict(zip(JOINTS, data['q'][7:, 0]))
    positions = []
    for joint in xml.getroot().find('worldbody').iter('joint'):
        if joint.get('type') == 'free':
            positions.extend(data['q'][:3, 0])
            positions.extend(data['q'][[6, 3, 4, 5], 0])  # MuJoCo uses wxyz.
        else:
            positions.append(q0[joint.get('name')])
    ET.SubElement(ET.SubElement(xml.getroot(), 'keyframe'), 'key', name='home',
                  qpos=' '.join(map(str, positions)))
    xml.write(xml_path)

    core = yaml.safe_load((Path(share('kyon_config')) / 'kyon_basic.yaml').read_text())
    core['XBotInterface'] = {'urdf': f'cat {output / "robot.urdf"}',
                            'srdf': f'cat {output / "robot.srdf"}'}
    core['xbotcore_threads']['rt_main'].update(sched='other', prio=0)
    core['xbotcore_plugins'] = {name: cfg for name, cfg in core['xbotcore_plugins'].items()
                               if name in ('ros_io', 'ros_control')}
    core['xbotcore_plugins']['ros_io']['params']['publish_sim_time'] = True
    (output / 'xbot.yaml').write_text(yaml.safe_dump(core))
    (output / 'mpc.yaml').write_text(yaml.safe_dump(
        {'dt': [dt], 'n_nodes': 1, 'model': {'fixed_joint_map': {}}}))
    controller = yaml.safe_load((package_directory() / 'config/step_up.yaml').read_text())
    (output / 'controller.yaml').write_text(yaml.safe_dump(controller))
    return urdf_text, srdf, sim_dir


def replay(data, dt, n, output, descriptions, children):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Trigger
    from kyon_controller.msg import WBTrajectory
    from xbot_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('saved_step_up_replay')
    logs = []
    samples = []
    telemetry = {key: [] for key in ('time', 'q', 'v', 'torque', 'q_ref', 'v_ref',
                                     'base_position', 'base_quaternion',
                                     'base_ref', 'base_quaternion_ref')}
    last_recorded_joints = None
    state = {'clock': None, 'joints': None, 'pose': None}

    def start(name, command):
        log = (output / f'{name}.log').open('w')
        logs.append(log)
        process = subprocess.Popen(['stdbuf', '-oL', '-eL', *command],
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)

    def wait_for(condition, timeout=20):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not condition():
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Child process exited with {child.returncode}; see {output}')
            if time.monotonic() > deadline:
                raise TimeoutError(f'Simulation did not become ready; see {output}')
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = [node.create_publisher(String, f'/xbotcore/{topic}', qos)
                      for topic in ('robot_description', 'robot_description_semantic')]
        for publisher, text in zip(publishers, descriptions[:2]):
            publisher.publish(String(data=text))
        subscriptions = [
            node.create_subscription(Clock, '/clock', lambda msg: state.update(
                clock=msg.clock.sec + msg.clock.nanosec * 1e-9), qos_profile_sensor_data),
            node.create_subscription(JointState, '/xbotcore/joint_states',
                                     lambda msg: state.update(joints=msg), qos_profile_sensor_data),
            node.create_subscription(PoseStamped, '/xbotcore/link_state/pelvis/pose',
                                     lambda msg: state.update(pose=msg), qos_profile_sensor_data),
        ]
        start('simulator', ['mujoco_simulator', str(output / 'model/step_up.xml'),
                            str(descriptions[2] / 'config/kyon.yaml')])
        wait_for(lambda: 'xbot2 bridge constructed' in
                 (output / 'simulator.log').read_text())
        start('xbot', ['xbot2-core', '-H', 'mj', '-C', str(output / 'xbot.yaml'), '-S'])
        wait_for(lambda: state['joints'] is not None and state['clock'] is not None)
        fast_filter = node.create_client(Trigger, '/xbotcore/set_filter_profile_fast')
        wait_for(lambda: fast_filter.service_is_ready())
        future = fast_filter.call_async(Trigger.Request())
        wait_for(future.done)
        if not future.result().success:
            raise RuntimeError('Could not enable the fast joint filter')
        start('controller', ['ros2', 'run', 'kyon_controller', 'controller', '--ros-args',
                              '-p', f'config:={output / "controller.yaml"}',
                              '-p', f'mpc_config:={output / "mpc.yaml"}', '-p', 'rate:=500',
                              '-r', 'robot_description:=/xbotcore/robot_description',
                              '-r', 'robot_description_semantic:=/xbotcore/robot_description_semantic'])
        publisher = node.create_publisher(WBTrajectory, '/mpc_solution', 1)
        wait_for(lambda: publisher.get_subscription_count() > 0 and state['pose'] is not None)
        print('Controller connected. Holding the initial pose for 2 simulation seconds.', flush=True)
        beginning = state['clock']
        last_clock = time.monotonic()
        previous_clock = beginning
        duration = n * dt
        outcome = 'completed'
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.005)
            now = state['clock']
            if now != previous_clock:
                last_clock, previous_clock = time.monotonic(), now
            if time.monotonic() - last_clock > 10:
                raise TimeoutError('Simulation clock stopped')
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Child process exited; see {output}')
            elapsed = max(0., now - beginning - 2.)
            coordinate = min(elapsed / dt, n)
            index = min(int(coordinate), n - 1)
            alpha = coordinate - index
            q = (1 - alpha) * data['q'][:, index] + alpha * data['q'][:, index + 1]
            q[3:7] /= np.linalg.norm(q[3:7])
            v = (1 - alpha) * data['v'][:, index] + alpha * data['v'][:, index + 1]
            msg = WBTrajectory()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.joint_names = JOINTS
            msg.q, msg.v = q.tolist(), v.tolist()
            msg.a = data['a'][:, index].tolist()
            # shortcut: replay joint references only; regenerate for payload-specific dynamics.
            msg.tau = [0.] * 18
            if elapsed >= duration:
                msg.v, msg.a = [0.] * 18, [0.] * 18
            publisher.publish(msg)
            actual = dict(zip(state['joints'].name, state['joints'].link_position))
            error = max(abs(actual[joint] - target) for joint, target in zip(JOINTS, q[7:]))
            effort = max(abs(value) for value in state['joints'].effort)
            pose = state['pose'].pose
            samples.append([elapsed, pose.position.x, pose.position.y, pose.position.z, error, effort])
            if state['joints'] is not last_recorded_joints:
                measured_q, measured_v, torque = joint_feedback(state['joints'])
                telemetry['time'].append(now - beginning - 2.)
                telemetry['q'].append(measured_q)
                telemetry['v'].append(measured_v)
                telemetry['torque'].append(torque)
                telemetry['q_ref'].append(q[7:].copy())
                telemetry['v_ref'].append(np.array(msg.v[6:]))
                telemetry['base_position'].append([pose.position.x, pose.position.y, pose.position.z])
                telemetry['base_quaternion'].append([pose.orientation.x, pose.orientation.y,
                                                      pose.orientation.z, pose.orientation.w])
                telemetry['base_ref'].append(q[:3].copy())
                telemetry['base_quaternion_ref'].append(q[3:7].copy())
                last_recorded_joints = state['joints']
            # Stop a fallen open-loop replay; an old trajectory cannot recover the robot.
            if pose.position.z < .2 or abs(pose.orientation.x) > .4 or abs(pose.orientation.y) > .4:
                outcome = 'stopped_after_fall'
                break
            if elapsed >= duration + 3:
                break
        np.savetxt(output / 'tracking.csv', samples, delimiter=',',
                   header='time,base_x,base_y,base_z,max_joint_error_rad,max_joint_effort_nm', comments='')
        np.savez_compressed(output / 'telemetry.npz', joint_names=JOINTS,
                            **{key: np.asarray(values) for key, values in telemetry.items()})
        summary = json.loads((output / 'model.json').read_text()) | {
            'outcome': outcome, 'trajectory_seconds': duration,
            'replayed_seconds': samples[-1][0], 'last_base_position': samples[-1][1:4],
            'max_joint_error_rad': max(row[4] for row in samples),
            'max_reported_joint_effort_nm': max(row[5] for row in samples)}
        (output / 'result.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        return outcome == 'completed'
    finally:
        for child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for child in reversed(children):
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-f', '--file', type=Path, default=package_directory() /
                        'mat_files/step_up_clearance.mat')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--payload-kg', type=payload_mass, default=0.)
    parser.add_argument('--payload-position', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Payload center in the pelvis frame, in meters')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if args.payload_position is not None:
        if args.payload_kg == 0 or not np.isfinite(args.payload_position).all():
            parser.error('--payload-position needs a positive payload mass and finite coordinates')
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        parser.error('Use ROS_DOMAIN_ID=77 in a dedicated simulation container')
    suffix = f'_payload_{args.payload_kg:g}kg' if args.payload_kg else ''
    output = (args.output or Path.cwd() / 'artifacts' /
              f'{args.file.stem}_legs_only{suffix}').resolve()
    output.mkdir(parents=True, exist_ok=True)
    for filename in ('result.json', 'tracking.csv', 'telemetry.npz', 'plots/report.pdf',
                     'plots/torques.png', 'plots/velocities.png', 'plots/positions.png',
                     'plots/tracking_errors.png', 'plots/base_motion.png',
                     'plots/mechanical_power.png'):
        (output / filename).unlink(missing_ok=True)
    data, dt, n = load_trajectory(args.file)
    descriptions = prepare(data, dt, output, args.payload_kg, args.payload_position)
    print(f'Prepared {n} intervals at {dt}s in {output}', flush=True)
    if not args.prepare_only:
        completed = replay(data, dt, n, output, descriptions, [])
        from plot_experiment import plot_experiment
        plot_experiment(output)
        return 0 if completed else 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
