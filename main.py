import argparse

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from robot import KinovaGen3Lite
from robot import DeviceConnection
from robot import Pose

import numpy as np
from matplotlib import pyplot as plt


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

        default = kinova.create_cartesian_action(base_cyclic=base_cyclic, pose=default_pose)
        kinova.move_sequence(base=base, task_list=[default])

        # Draw square
        p1 = Pose([0.25, 0.0, 0.20, 0.0, 180.0, 90.0])
        p2 = Pose([0.25, 0.1, 0.20, 0.0, 180.0, 90.0])
        p3 = Pose([0.15, 0.1, 0.20, 0.0, 180.0, 90.0])
        p4 = Pose([0.15, 0.0, 0.20, 0.0, 180.0, 90.0])
        tasks = [kinova.create_cartesian_action(base_cyclic, p) for p in [p1, p2, p3, p4]]
        # kinova.move_sequence(base=base, task_list=tasks)

        # Draw circle
        center_x = 0.15
        center_y = 0.15
        radius = 0.05

        points = []
        for theta in range(1, 360):
            points.append(
                Pose(
                    [radius * np.sin(theta) + center_x, radius * np.cos(theta) + center_y,
                     0.20, 0.0, 180.0, 90.0]
                ))
        tasks = [kinova.create_cartesian_action(base_cyclic, p) for p in points]
        kinova.move_sequence(base=base, task_list=tasks)
