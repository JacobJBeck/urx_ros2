#!/usr/bin/env python3
"""Module to interface with Universal Robotics robots."""

# Standard modules
import argparse
import threading as th

# ROS2 modules
import rclpy
from rclpy.node import Node

# ROS message types
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# External modules
import urx

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
               "wrist_2_joint", "wrist_3_joint"]


class URInterface(Node):
    """Node to interface with Universal Robotics robots."""

    def __init__(self, config):
        """Initialize class."""
        super().__init__('ur_interface')

        # Store parameters
        self._acc = config['acceleration']
        self._vel = config['velocity']

        # Create trajectory process
        self._trajectory_thread = th.Thread()

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
        self._joint_state_pub = self.create_publisher(JointState, config['state_topic'])

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

        # Kill existing trajectory
        if self._trajectory_thread.is_alive():
            self._trajectory_thread.join(0.0001)
            self._trajectory_thread.terminate()

        # Start new trajectory
        self._trajectory_thread = th.Thread(target=self._execute_trajectory, args=(msg.points,))
        self._trajectory_thread.start()

        return

    def _execute_trajectory(self, points):
        """Execute a trajectory in a seperate thread."""
        for pt in points:  # TODO: use time_from_start to control speed
            # FIXME: movej causes current move to stop, resulting in unsmooth movement
            self._robot.movej(pt.positions, vel=self._vel, acc=self._acc, wait=False)

        return

    def _joint_state_callback(self):
        """Update robot joint state when called by timer."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self._robot.getj()

        self._joint_state_pub.publish(msg)

        return


def main(args=None):
    """Parse arguments and start node."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--robot-ip', type=str, default='192.168.10.101',
                        help="IP address of robot")
    parser.add_argument('-c', '--command-topic', type=str, default='/urx/joint_command',
                        help="Topic to listen for commands on")
    parser.add_argument('-s', '--state-topic', type=str, default='/urx/joint_states',
                        help="Topic to publish joint state to")
    parser.add_argument('-r', '--state-update-rate', type=float, default=30,
                        help="Rate at which joint states are queried (Hz)")
    parser.add_argument('-a', '--acceleration', type=float, default=0.5,
                        help="Robot acceleration (units unknown)")
    parser.add_argument('-v', '--velocity', type=float, default=1.0,
                        help="Robot velocity (units unknown)")
    args = parser.parse_args()

    interface = URInterface(vars(args))
    rclpy.spin(interface)

    interface.destroy_node()
    rclpy.shutdown()

    return


if __name__ == '__main__':
    main()
