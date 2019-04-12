#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Empty,EmptyRequest

from ros_numpy import numpify
import numpy as np



if __name__ == '__main__':

    rospy.init_node('mujoco_ros_py', anonymous=True)

    # control robot
    joint_positions_ = [ 0.0 ] * 6

    jpos_pub_ = rospy.Publisher('/mujoco/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
    jpos_pub_.publish(Float32MultiArray(data=joint_positions_))

    # close the gripper
    gripper_close_ = True

    gripper_pub_ = rospy.Publisher('/mujoco/ur5/command/gripper', Bool, queue_size=1)
    gripper_pub_.publish(Bool(data=gripper_close_))

    # get image
    img_msg_ = rospy.wait_for_message('/mujoco/ros_cam/rgb', Image)
    img_ = numpify(img_msg_)

    # reset simulation
    reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)
    reset_svc_(EmptyRequest())
