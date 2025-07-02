#!/usr/bin/python3
from geometry_msgs.msg import Vector3
from kyon_controller.msg import WBTrajectory
import rospy
import numpy as np
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
import scipy
import time
import argparse

class XBotHorizonReplayer:
    def __init__(self, solution, fixed_joint_map):

        self.urdf = rospy.get_param('/xbotcore/robot_description')
        self.srdf = rospy.get_param('/xbotcore/robot_description_semantic')

        self.solution_publisher = None
        self.sol_msgs = list()

        self.fixed_joint_map = fixed_joint_map

        self.kin_dyn = casadi_kin_dyn.CasadiKinDyn(self.urdf, fixed_joints=fixed_joint_map)
        self.solution = solution

        self.joint_names = [elem for elem in self.kin_dyn.joint_names() if elem not in ['universe', 'reference']]

        self.dim_q = self.solution['q'].shape[0]
        self.dim_v = self.solution['v'].shape[0]
        self.dim_a = self.solution['a'].shape[0]
        self.dim_tau = self.solution['tau'].shape[0]

        self.nodes_q = self.solution['q'].shape[1]
        self.nodes_v = self.solution['v'].shape[1]
        self.nodes_a = self.solution['a'].shape[1]
        self.nodes_tau = self.solution['tau'].shape[1]

        self.dt = self.solution['dt'][0][0]
        self.ns = self.solution['ns'][0][0]
        print(f'n nodes: {self.dt}')
        print(f'dt: {self.ns}')
        print(f'dim q: {self.dim_q}x{self.nodes_q}') # n nodes
        print(f'dim v: {self.dim_v}x{self.nodes_v}') # n nodes
        print(f'dim a: {self.dim_a}x{self.nodes_a}') # n nodes - 1
        print(f'dim tau: {self.dim_tau}x{self.nodes_tau}') # n nodes - 1


        self.init_xbot()
        self.init_publisher()
        self.msg_from_solution()
        self.init_rate()

    def init_publisher(self):

        self.solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)

    def init_xbot(self):

        rospy.init_node('xbot_replay')

        cfg = co.ConfigOptions()
        cfg.set_urdf(self.urdf)
        cfg.set_srdf(self.srdf)
        cfg.generate_jidmap()
        cfg.set_string_parameter('model_type', 'RBDL')
        cfg.set_string_parameter('framework', 'ROS')
        cfg.set_bool_parameter('is_model_floating_base', True)

        self.xbot_model = xbot.ModelInterface(cfg)
        self.xbot_robot = xbot.RobotInterface(cfg)

    def init_rate(self):

        dt = self.solution['dt'][0][0]
        self.rate = 1 / dt
        self.ros_rate = rospy.Rate(self.rate)

    def homing(self, duration, dt=0.001):

        """
            Interpolates joint values from dict1 to dict2 at a given rate (units/sec).

            Args:
                dict1 (dict): Starting joint values.
                dict2 (dict): Target joint values.
                rate (float): Rate of interpolation in units per second.
                callback (function): Called with current interpolated values at each step.
                update_interval (float): Time between updates in seconds.
            """

        bar_visual_length = 30

        robot_initial_joint_position = list()
        robot_initial_joint_temp = self.xbot_robot.getPositionReference()
        robot_initial_joint_dict = self.xbot_robot.eigenToMap(robot_initial_joint_temp)

        # add in list the initial values of joint position (without fixed joints)
        for joint in self.joint_names:
            robot_initial_joint_position.append(robot_initial_joint_dict[joint])

        solution_initial_joint_position = self.solution['q'][7:, 0]

        # add fixed joints
        for fixed_joint_name, fixed_joint_val in self.fixed_joint_map.items():
            print(f'  - adding fixed joint {fixed_joint_name}: {fixed_joint_val}')
            robot_initial_joint_position.append(robot_initial_joint_dict[fixed_joint_name])
            solution_initial_joint_position = np.append(solution_initial_joint_position, fixed_joint_val)


        if len(robot_initial_joint_position) != len(solution_initial_joint_position):
            raise ValueError(f"initial and goal joint position must be the same length: ({len(robot_initial_joint_position)}) != ({len(solution_initial_joint_position)})")


        start = np.array(robot_initial_joint_position, dtype=np.float32)
        end = np.array(solution_initial_joint_position, dtype=np.float32)
        total_steps = max(1, int(duration / dt))


        for step in range(total_steps + 1):
            t = step / total_steps  # normalized time [0, 1]
            current = (1 - t) * start + t * end

            self.xbot_robot.setPositionReference(dict(zip(self.joint_names + list(self.fixed_joint_map.keys()), current)))
            self.xbot_robot.move()

            # sol_msg = WBTrajectory()
            # sol_msg.header.frame_id = 'world'
            # sol_msg.header.stamp = rospy.Time.now()
            # sol_msg.joint_names = self.joint_names
            # sol_msg.q = current
            #
            # self.solution_publisher.publish(sol_msg)
            # self.ros_rate.sleep()

            # Progress bar and time info
            percent = int(t * 100)
            time_remaining = duration - (step * dt)
            filled = int(bar_visual_length * t)
            bar = "█" * filled + "-" * (bar_visual_length - filled)

            print(f"\r[{bar}] {percent:3d}%  Time remaining: {time_remaining:5.2f}s", end="")

            time.sleep(dt)

        print("\nHoming complete.")

        self.xbot_robot.sense()
        robot_initial_joint_position = list()
        robot_initial_joint_dict = self.xbot_robot.eigenToMap(self.xbot_robot.getPositionReference())

        for joint in self.joint_names:
            robot_initial_joint_position.append(robot_initial_joint_dict[joint])

        # add fixed joints
        for fixed_joint_name, fixed_joint_val in self.fixed_joint_map.items():
            robot_initial_joint_position.append(robot_initial_joint_dict[fixed_joint_name])

        error_matrix = np.array(robot_initial_joint_position) - solution_initial_joint_position

        print('joint position error: ')
        for joint_name, joint_error in dict(zip(self.joint_names + list(self.fixed_joint_map.keys()), error_matrix)).items():
            print(f'{joint_name}: {joint_error}')



    def msg_from_solution(self):

        for sample_i in range(self.nodes_a):

            self.sol_msg_i = WBTrajectory()
            self.sol_msg_i.header.frame_id = 'world'
            # self.sol_msg_i.header.stamp = rospy.Time.now()
            self.sol_msg_i.joint_names = self.joint_names

            self.sol_msg_i.q = self.solution['q'][:, sample_i].tolist()
            self.sol_msg_i.v = self.solution['v'][:, sample_i].tolist()
            self.sol_msg_i.a = self.solution['a'][:, sample_i].tolist()
            self.sol_msg_i.tau = self.solution['tau'][:, sample_i].tolist()

            self.sol_msgs.append(self.sol_msg_i)

    def replay(self):

        bar_length = 30
        total_steps = len(self.sol_msgs)

        self.xbot_robot.sense()

        for step, msg_i in enumerate(self.sol_msgs):

            # robot_current_joint_position = list()
            # robot_current_joint_dict = self.xbot_robot.getJointPositionMap()
            #
            # for joint in self.joint_names:
            #     robot_current_joint_position.append(robot_current_joint_dict[joint])
            #
            # difference = self.solution['q'][7:, step] - robot_current_joint_position
            # print(f"difference: {difference}")


            # self.xbot_robot.setPositionReference(dict(zip(self.joint_names, self.solution['q'][7:, step])))
            # self.xbot_robot.move()
            # time.sleep(1/self.rate)



            msg_i.header.stamp = rospy.Time.now()
            self.solution_publisher.publish(msg_i)
            self.ros_rate.sleep()

            t = step / (total_steps - 1) if total_steps > 1 else 1.0
            percent = int(t * 100)
            filled = int(bar_length * t)
            bar = "█" * filled + "-" * (bar_length - filled)

            print(f"\r[{bar}] {percent:3d}%  Publishing messages...", end="")

        print('Trajectory replay completed!')

    def main(self, mode):

        if mode == "replay":
            print("Replaying ...")
            self.replay()
        elif mode == "homing":
            print("Homing ...")
            self.homing(5.)


def retrieve_info_by_name(data, name):
    if name in data:
        print(f'{name}: {data[name][0][0]}')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Process a file with replay or homing mode.")

    parser.add_argument("-f", "--file", required=True, help="Name of the file to process")
    parser.add_argument("mode", choices=["replay", "homing", "info"], help="Mode to run: 'replay' or 'homing' or 'info'")

    args = parser.parse_args()

    print(f"Data from file: {args.file}")
    data = scipy.io.loadmat(args.file)

    fixed_joint_map = {'shoulder_yaw_1': 0.0,
                       'shoulder_pitch_1': np.pi / 2,
                       'elbow_pitch_1': 0.,
                       'wrist_pitch_1': 0.,
                       'wrist_yaw_1': 0.,
                       'shoulder_yaw_2': 0.0,
                       'shoulder_pitch_2': -np.pi / 2,
                       'elbow_pitch_2': 0.,
                       'wrist_pitch_2': 0.,
                       'wrist_yaw_2': 0.,
                       'dagana_1_clamp_joint': 0,
                       'dagana_2_clamp_joint': 0,
                       # 'ankle_yaw_1': 0.,
                       # 'ankle_yaw_2': 0.,
                       # 'ankle_yaw_3': 0.,
                       # 'ankle_yaw_4': 0.
                       }


    if args.mode == 'info':
        retrieve_info_by_name(data, 'ns')
        retrieve_info_by_name(data, 'dt')
        retrieve_info_by_name(data, 'tf')
        retrieve_info_by_name(data, 'initial_time')
        retrieve_info_by_name(data, 'final_time')
        retrieve_info_by_name(data, 'contact_flight_time')
        retrieve_info_by_name(data, 'double_stance_time')
        retrieve_info_by_name(data, 'n_cycles')
        retrieve_info_by_name(data, 'step_x')
        retrieve_info_by_name(data, 'step_z')
        retrieve_info_by_name(data, 'clearance')
        retrieve_info_by_name(data, 'displacement_x')
        retrieve_info_by_name(data, 'cycle_order')
        print(f'fixed_joint_map: \n{fixed_joint_map}')

    else:



        xbot_horizon_replayer = XBotHorizonReplayer(data, fixed_joint_map)
        xbot_horizon_replayer.main(args.mode)


    # data = scipy.io.loadmat('/tmp/kyon_codesign_mat_files/gap_bag.mat')
    # data = scipy.io.loadmat('/tmp/kyon_codesign_mat_files/paw_bag.mat')


