#!/usr/bin/env python3
"""TODO: Docstring"""

# Jacob Beck
# TODO: Description

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf2_ros

import urx

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
               "wrist_2_joint", "wrist_3_joint"]


class URInterface(Node):
    """TODO: Docstring"""
    def __init__(self, config):
        """TODO: Docstring"""
        super().__init__('ur_interface')

        # Store parameters
        self._accel = config['acceleration']
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

        return

    def _transform_callback(self, msg):
        """Looks for target position in TF messages"""
        # Look for our target (I'm sure there's a more pythonic way to do this)
        target = None
        for transform in msg.transforms:
            if transform.child_frame_id == self._target_name:
                target = transform
                break

        if target is None:
            return

        # Check target pose confidence
        if (self._trans_variance > self._max_trans_variance
                or self._rot_variance > self._max_rot_variance):
            return

        # Offset pose from target
        new_pose = PoseStamped()
        new_pose.header.frame_id = target.child_frame_id
        new_pose.position.z = self._target_offset

        # Transform pose into robot coordinates
        transform = self._tf_buffer.lookup_transform('base_link', target.child_frame_id, self.now())
        new_pose = transform.do_transform_pose(new_pose)

        # Align camera with object
        self._robot.set_pose(new_pose, self._accel, self._vel)

        return

    def _joint_state_callback(self):
        """Called by timer to update the the robot joint state"""
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
