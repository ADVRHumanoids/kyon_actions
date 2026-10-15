#!/usr/bin/env python3
"""Home the 12 leg joints or replay a MAT trajectory through the live ROS 2 stack."""
import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from replay_sim_ros2 import JOINTS, joint_feedback, load_trajectory, package_directory


def reference_at(data, dt, elapsed):
    n = data['a'].shape[1]
    coordinate = np.clip(elapsed / dt, 0, n)
    i = min(int(coordinate), n - 1)
    alpha = coordinate - i
    q = (1 - alpha) * data['q'][:, i] + alpha * data['q'][:, i + 1]
    q[3:7] /= np.linalg.norm(q[3:7])
    v = (1 - alpha) * data['v'][:, i] + alpha * data['v'][:, i + 1]
    a = data['a'][:, i].copy()
    if elapsed >= n * dt:
        v[:] = 0
        a[:] = 0
    return q, v, a


def homing_position(start, target, elapsed, duration):
    phase = np.clip(elapsed / duration, 0, 1)
    blend = phase**3 * (10 - 15 * phase + 6 * phase**2)
    return start + blend * (target - start)


def validate_model(urdf, data):
    from casadi_kin_dyn import pycasadi_kin_dyn
    kd = pycasadi_kin_dyn.CasadiKinDyn(urdf)
    if kd.nq() != 19 or kd.joint_names()[2:] != JOINTS:
        raise ValueError('Live model must contain only the floating base and 12 leg joints')
    tree = ET.fromstring(urdf)
    for index, name in enumerate(JOINTS):
        limit = tree.find(f"joint[@name='{name}']/limit")
        q = data['q'][index + 7]
        v = data['v'][index + 6]
        if (q.min() < float(limit.get('lower')) - 1e-6 or
                q.max() > float(limit.get('upper')) + 1e-6 or
                np.max(np.abs(v)) > float(limit.get('velocity')) + 1e-6):
            raise ValueError(f'Trajectory exceeds the live model limits for {name}')
    return kd


def validate_homing(urdf, start, target, duration):
    tree = ET.fromstring(urdf)
    for i, name in enumerate(JOINTS):
        limit = tree.find(f"joint[@name='{name}']/limit")
        if not float(limit.get('lower')) <= start[i] <= float(limit.get('upper')):
            raise ValueError(f'Measured position is outside the model limits for {name}')
        # The quintic blend has a peak normalized speed of 1.875.
        if 1.875 * abs(target[i] - start[i]) / duration > float(limit.get('velocity')):
            raise ValueError(f'Homing is too fast for {name}; increase --duration')


def run(args, data, dt, n):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from xbot_msgs.msg import JointState, JointCommand
    from kyon_controller.msg import WBTrajectory
    from rcl_interfaces.srv import GetParameters

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('kyon_mat_replayer')
    state = {'urdf': None, 'joints': None, 'received': None}
    publisher = None
    last_q = None
    sent = False
    records = []

    def feedback(message):
        state.update(joints=message, received=time.monotonic())

    subscriptions = [
        node.create_subscription(String, '/xbotcore/robot_description',
                                 lambda msg: state.update(urdf=msg.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(JointState, '/xbotcore/joint_states', feedback,
                                 qos_profile_sensor_data),
    ]

    def wait_for(condition, timeout=10.):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError('Live robot/controller did not become ready')
            rclpy.spin_once(node, timeout_sec=.01)

    def measured():
        if time.monotonic() - state['received'] > args.feedback_timeout:
            raise TimeoutError('Joint feedback is stale')
        values = joint_feedback(state['joints'])
        if not all(np.isfinite(value).all() for value in values):
            raise ValueError('Joint feedback contains non-finite values')
        return values

    def publish(q, v, a):
        nonlocal sent, last_q
        if args.mode == 'homing':
            msg = JointCommand()
            msg.name = JOINTS
            msg.position = q[7:].astype(float).tolist()
            # Position-only homing keeps zero velocity/effort references if the process is lost.
            msg.velocity, msg.effort = [0.] * 12, [0.] * 12
            msg.ctrl_mode = [7] * 12  # position | velocity | effort; preserve existing gains
        else:
            msg = WBTrajectory()
            msg.header.frame_id = 'world'
            msg.joint_names = JOINTS
            msg.q, msg.v, msg.a = q.tolist(), v.tolist(), a.tolist()
            msg.tau = [0.] * 18  # Saved full-body torques do not match the leg-only model.
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        last_q, sent = q.copy(), True

    try:
        wait_for(lambda: state['urdf'] is not None and state['joints'] is not None)
        kd = validate_model(state['urdf'], data)
        q_measured, v_measured, _ = measured()
        if set(state['joints'].name) != set(JOINTS):
            raise ValueError('Joint feedback must contain exactly the 12 leg joints')
        if np.max(np.abs(v_measured)) > .2:
            raise ValueError('Robot must be stationary before homing or replay')
        topic = '/xbotcore/command' if args.mode == 'homing' else '/mpc_solution'
        publisher = node.create_publisher(JointCommand if args.mode == 'homing' else WBTrajectory,
                                          topic, 1)
        wait_for(lambda: publisher.get_subscription_count() > 0)
        # Allow discovery to expose any competing command source before the first command.
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        if node.count_publishers(topic) != 1:
            raise RuntimeError(f'Another command publisher is active on {topic}')
        if args.mode == 'replay':
            client = node.create_client(GetParameters, '/controller_node/get_parameters')
            wait_for(client.service_is_ready)
            request = GetParameters.Request(names=['command_timeout'])
            future = client.call_async(request)
            wait_for(future.done)
            if not future.result().values or future.result().values[0].double_value <= 0:
                raise RuntimeError('Start controller_step_up.launch.py with a positive command_timeout')
        q_measured, _, _ = measured()
        if args.mode == 'homing':
            validate_homing(state['urdf'], q_measured, data['q'][7:, 0], args.duration)
        if args.mode == 'replay' and np.max(np.abs(q_measured - data['q'][7:, 0])) > args.position_tolerance:
            raise ValueError('Robot is not at the initial posture; run homing first')
        start_q = q_measured.copy()
        duration = args.duration if args.mode == 'homing' else n * dt
        print(f'{args.mode}: {duration:.2f}s, live model {kd.mass():.3f} kg, 12 leg joints', flush=True)
        beginning = time.monotonic()
        next_tick = beginning
        while True:
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            q_measured, v_measured, torque = measured()
            if node.count_publishers(topic) != 1 or publisher.get_subscription_count() == 0:
                raise RuntimeError('Command connection changed during motion')
            elapsed = now - beginning
            q, v, a = reference_at(data, dt, elapsed if args.mode == 'replay' else 0.)
            if args.mode == 'homing':
                q[7:] = homing_position(start_q, data['q'][7:, 0], elapsed, duration)
                v[:], a[:] = 0., 0.
            publish(q, v, a)
            records.append([elapsed, *q_measured, *v_measured, *torque, *q[7:], *v[6:]])
            if elapsed >= duration + 1.:
                if args.mode == 'homing' and np.max(np.abs(q_measured - data['q'][7:, 0])) > args.position_tolerance:
                    raise RuntimeError('Homing finished but measured joints did not reach the target tolerance')
                print(f'{args.mode} complete; final joint error '
                      f'{np.max(np.abs(q_measured - q[7:])):.4f} rad', flush=True)
                break
            next_tick = max(next_tick + 1. / args.rate, now)
            time.sleep(max(0., next_tick - time.monotonic()))
    finally:
        if sent and publisher is not None and rclpy.ok():
            # Leave a position hold with zero velocity, acceleration, and feedforward effort.
            for _ in range(10):
                publish(last_q, np.zeros(18), np.zeros(18))
                time.sleep(.01)
        if records:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            header = ['time'] + [f'{field}_{name}' for field in ('q', 'v', 'torque', 'q_ref', 'v_ref')
                                for name in JOINTS]
            np.savetxt(args.output, records, delimiter=',', header=','.join(header), comments='')
            print(f'Joint feedback saved to {args.output}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('info', 'homing', 'replay'))
    parser.add_argument('-f', '--file', type=Path,
                        default=package_directory() / 'mat_files/step_up_clearance.mat')
    parser.add_argument('--duration', type=float, default=5., help='Homing duration in seconds')
    parser.add_argument('--rate', type=float, default=200.)
    parser.add_argument('--position-tolerance', type=float, default=.05, help='Radians')
    parser.add_argument('--feedback-timeout', type=float, default=.5, help='Wall seconds')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if any(not np.isfinite(value) or value <= 0 for value in
           (args.duration, args.rate, args.position_tolerance, args.feedback_timeout)):
        parser.error('Duration, rate, position tolerance, and feedback timeout must be finite and positive')
    data, dt, n = load_trajectory(args.file)
    if args.mode == 'info':
        print(f'{args.file}: {n} intervals, dt={dt:g}s, duration={n * dt:g}s')
        print(dict(zip(JOINTS, data['q'][7:, 0])))
        return 0
    args.output = args.output or Path(f'{args.mode}_feedback.csv')
    try:
        run(args, data, dt, n)
    except KeyboardInterrupt:
        print('Interrupted; position hold requested.', flush=True)
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
