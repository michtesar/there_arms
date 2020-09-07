import argparse

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

from robot import KinovaGen3Lite
from robot import DeviceConnection
from robot import Pose
import threading

import numpy as np
import time
import math
from matplotlib import pyplot as plt


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications
    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def draw_velocity_vector(base, velocity_vector):

    command = Base_pb2.TwistCommand()
    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    for vel_xyz in velocity_vector:
        twist = command.twist
        twist.linear_x = vel_xyz[0]
        twist.linear_y = vel_xyz[1]
        twist.linear_z = vel_xyz[2]
        twist.angular_x = 0
        twist.angular_y = 0
        twist.angular_z = 0

        print("EE velocity (x,y,z) : ", vel_xyz)
        vel_norm = np.sqrt(vel_xyz[0]*vel_xyz[0] + vel_xyz[1]*vel_xyz[1] + vel_xyz[2]*vel_xyz[2])
        print("norm (m/s) : ", vel_norm)
        base.SendTwistCommand(command)
        time.sleep(10 * vel_norm)

    print("----- Cartesian velocity control : DONE ")
    print("----- Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def build_circle_velocity_vector(radius, angle_resolution):
    # delta
    delta = radius * math.tan(angle_resolution)
    # to be continued !!!



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', help='IP address of the robot', default='192.168.10')
    parser.add_argument('--username', help='Username to login')
    parser.add_argument('--password', help='IP address of the robot')
    args = parser.parse_args()

    kinova = KinovaGen3Lite(ip_address=args.ip)

    with DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # ---- move to retract
        # print(kinova.get_pose(base_cyclic=base_cyclic))
        # print(kinova.get_joints(base_cyclic=base_cyclic))
        kinova.move_named_pose(base, kinova.POSE_RETRACT)
        print(kinova.get_pose(base_cyclic=base_cyclic))
        # kinova.move_angular(base=base, configuration=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # task_1 = kinova.create_joint_sequence(base=base, joint_array=joints)
        # kinova.move_sequence(base=base, task_list=[task_1])

        default_pose = Pose(
            [0.15, 0.0, 0.20, 0.0, 180.0, 90.0]
        )

        # ---- move to default
        default = kinova.create_cartesian_action(base_cyclic=base_cyclic, pose=default_pose)
        kinova.move_sequence(base=base, task_list=[default])

        # ---- Draw square
        # p1 = Pose([0.25, 0.0, 0.20, 0.0, 180.0, 90.0])
        # p2 = Pose([0.25, 0.1, 0.20, 0.0, 180.0, 90.0])
        # p3 = Pose([0.15, 0.1, 0.20, 0.0, 180.0, 90.0])
        # p4 = Pose([0.15, 0.0, 0.20, 0.0, 180.0, 90.0])
        # tasks = [kinova.create_cartesian_action(base_cyclic, p) for p in [p1, p2, p3, p4]]
        # kinova.move_sequence(base=base, task_list=tasks)

        # Draw circle
        center_x = 0.15
        center_y = 0.15
        radius = 0.05

        # points = []
        # for theta in range(1, 360):
        #     points.append(
        #         Pose(
        #             [radius * np.sin(theta) + center_x, radius * np.cos(theta) + center_y,
        #              0.20, 0.0, 180.0, 90.0]
        #         ))
        # tasks = [kinova.create_cartesian_action(base_cyclic, p) for p in points]
        # kinova.move_sequence(base=base, task_list=tasks)

        velocity_vector = [[0.01, 0.02, 0.01],
                           [0.01, -0.02, -0.01],
                           [-0.01, 0.01, 0.02],
                           [-0.02, -0.03, 0.01],
                           [0.01, 0.02, 0.01],
                           [-0.03, -0.02, -0.01],
                           [0.03, -0.02, 0.01],
                           [-0.01, 0.02, -0.01],
                           ]
        draw_velocity_vector(base, velocity_vector)