#!/usr/bin/env python3
"""TODO: Docstring"""

# Jacob Beck
# TODO: Description

import argparse

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

import urx

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
               "wrist_2_joint", "wrist_3_joint"]


class URInterface(Node):
    """TODO: Docstring"""

    def __init__(self, config):
        """TODO: Docstring"""
        super().__init__('ur_interface')

        # Store parameters
        self._acc = config['acceleration']
        self._vel = config['velocity']

        # Connect to robot
        self.get_logger().info("Connecting to robot at {}".format(config['robot_ip']))
        connected = False
        while not connected:
            try:
                self._robot = urx.Robot(config['robot_ip'], True)
                connected = True
            except OSError as ex:
                self.get_logger().warn("Unable to connect: {}. Retrying...".format(str(ex)))

        # Create subscribers/publishers
        self._joint_state_timer = self.create_timer(1.0 / config['state_update_rate'],
                                                    self._joint_state_callback)
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states')

        self._trajectory_sub = self.create_subscription(JointTrajectory, config['command_topic'],
                                                        self._trajectory_callback)

        return

    def _trajectory_callback(self, msg):
        """Execute trajectory commands."""
        # Verify joint names
        for msg_name, known_name in zip(msg.joint_names, JOINT_NAMES):
            if msg_name != known_name:
                self.get_logger.error("Joint names in message do not match known names. '{}' does"
                                      "not match '{}'".format(msg_name, known_name))
                return

        # Sequence waypoints
        for pt in msg.points:
            self._robot.movej(pt.positions, vel=self._vel, acc=self._acc, wait=True)

        return

    def _joint_state_callback(self):
        """Update robot joint state when called by timer."""
        msg = JointState()
        msg.header.stamp = self.now()
        msg.name = JOINT_NAMES
        msg.position = self._robot.getj()

        self._joint_state_pub.publish(msg)

        return


def main(args=None):
    """TODO: Docstring"""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--robot-ip', type=str, default='192.168.10.77',
                        help="IP address of robot")
    parser.add_argument('-c', '--command-topic', type=str, default='ur5/command',
                        help="Topic to listen for commands on")
    parser.add_argument('-r', '--state-update-rate', type=float, default=30,
                        help="Rate at which joint states are queried (Hz)")
    parser.add_argument('-a', '--acceleration', type=float, default=0.01,
                        help="Robot acceleration (units unknown)")
    parser.add_argument('-v', '--velocity', type=float, default=0.01,
                        help="Robot velocity (units unknown)")
    args = parser.parse_args()

    interface = URInterface(vars(args))
    rclpy.spin(interface)

    interface.destroy_node()
    rclpy.shutdown()

    return


if __name__ == '__main__':
    main()
