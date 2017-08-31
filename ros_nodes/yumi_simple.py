#!/usr/bin/env python
"""A script that allows to send simple ROS commands to command YuMi."""

from autolab_core import RigidTransform
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from yumipy import YuMiRobot

import numpy as np
import rospy
import threading
import tf

yumi_lock = threading.Lock()


def command_pose_right_callback(pose_msg, args):
    '''Command pose callback for the right arm.'''
    yumi = args[0]
    rigid_transform = ros_pose_to_autolab_core_pose(pose_msg)
    with yumi_lock:
        yumi.right.goto_pose(rigid_transform)


def command_pose_left_callback(pose_msg, args):
    '''Command pose callback for the left arm.'''
    yumi = args[0]
    rigid_transform = ros_pose_to_autolab_core_pose(pose_msg)
    with yumi_lock:
        yumi.left.goto_pose(rigid_transform)


def command_home_left_callback(empty_msg, args):
    '''Command home callback for the left arm.'''
    yumi = args[0]
    with yumi_lock:
        yumi.left.reset_home()


def command_home_right_callback(empty_msg, args):
    '''Command home callback for the right arm.'''
    yumi = args[0]
    with yumi_lock:
        yumi.right.reset_home()


def ros_pose_to_autolab_core_pose(pose_msg):
    rigid_transform = RigidTransform()
    rigid_transform.translation = (pose_msg.pose.position.x,
                                   pose_msg.pose.position.y,
                                   pose_msg.pose.position.z)
    quaternion = np.array([
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w
    ])

    rigid_transform.rotation = tf.transformations.quaternion_matrix(
        quaternion)[0:3, 0:3]
    return rigid_transform


def autolab_core_pose_to_ros_pose(pose, ros_now, frame_id):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = ros_now
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position.x = pose.translation[0]
    pose_msg.pose.position.y = pose.translation[1]
    pose_msg.pose.position.z = pose.translation[2]

    transformation_matrix = tf.transformations.translation_matrix(
        (pose.translation[0], pose.translation[1], pose.translation[2]))
    transformation_matrix[0:3, 0:3] = pose.rotation
    quaternion = tf.transformations.quaternion_from_matrix(
        transformation_matrix)
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]

    return pose_msg


def main():
    right_end_effector_frame_id = 'right_end_effector'
    left_end_effector_frame_id = 'left_end_effector'
    rospy.init_node('yumi_simple')
    try:
        yumi = YuMiRobot(arm_type='remote')
    except RuntimeError, e:
        print("Couldn't connect to YuMi. Got error: {}".format(e))
        exit()
    pose_right_pub = rospy.Publisher('right/pose', PoseStamped, queue_size=10)
    pose_left_pub = rospy.Publisher('left/pose', PoseStamped, queue_size=10)
    # robot_state_pub = rospy.Publisher(
    #     'joint_states', RobotState, queue_size=10)
    joint_state_pub = rospy.Publisher(
        'joint_states', JointState, queue_size=10)

    pose_left_sub = rospy.Subscriber("command/left/pose", PoseStamped,
                                     command_pose_left_callback, (yumi, ))
    pose_right_sub = rospy.Subscriber("command/right/pose", PoseStamped,
                                      command_pose_right_callback, (yumi, ))
    home_left_sub = rospy.Subscriber("command/left/home", Empty,
                                     command_home_left_callback, (yumi, ))
    home_right_sub = rospy.Subscriber("command/right/home", Empty,
                                      command_home_right_callback, (yumi, ))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ros_now = rospy.Time.now()
        with yumi_lock:
            right_pose = yumi.right.get_pose()
        if right_pose:
            ros_now = rospy.Time.now()
            pose_right_state_message = autolab_core_pose_to_ros_pose(
                right_pose.copy(), ros_now, right_end_effector_frame_id)
            pose_right_pub.publish(pose_right_state_message)
        with yumi_lock:
            left_pose = yumi.left.get_pose()
        if left_pose:
            ros_now = rospy.Time.now()
            pose_left_state_message = autolab_core_pose_to_ros_pose(
                left_pose.copy(), ros_now, left_end_effector_frame_id)
            pose_left_pub.publish(pose_left_state_message)
        with yumi_lock:
            left_joints = yumi.left.get_state()
            right_joints = yumi.right.get_state()
        if left_joints and right_joints:
            joint_names_right = ['yumi_joint_%d_r' % i for i in range(1, 8)]
            joint_names_left = ['yumi_joint_%d_l' % i for i in range(1, 8)]

            ros_now = rospy.Time.now()
            robot_state_mesage = RobotState()
            joint_state_message = JointState()
            joint_state_message.header.stamp = ros_now
            joints = right_joints.joints + left_joints.joints
            # robot_state_mesage.joint_state.name = joint_names_right + joint_names_left
            # robot_state_mesage.joint_state.position = np.radians(joints)
            # robot_state_pub.publish(robot_state_mesage)
            joint_state_message.name = joint_names_right + joint_names_left
            joint_state_message.position = np.radians(joints)
            joint_state_pub.publish(joint_state_message)
        rate.sleep()


if __name__ == '__main__':
    main()
