#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState


q1 = np.vstack([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.vstack([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
a1gripper_cmd = 0.0
a2gripper_cmd = 0.0
arm_joints = rospy.Publisher('joint_states', JointState, queue_size=1)


rate_HZ = 100.0
dt = 1/rate_HZ

def arm_cb(cmd):
	global q1
	global q2
	q1 = np.vstack([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])
	q2 = np.vstack([cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9], cmd.data[10], cmd.data[11]])

def grip_cb(cmd):
	global a1gripper_cmd, a2gripper_cmd
	a1gripper_cmd, a2gripper_cmd = cmd.data[0], cmd.data[1]

def rviz():
	global q1
	global q2

	rate = rospy.Rate(rate_HZ)

	while not rospy.is_shutdown():
		joint_cmd = JointState()
		joint_cmd.header.stamp = rospy.get_rostime()
		joint_cmd.name = ['arm1-joint1','arm1-joint2','arm1-joint3','arm1-joint4','arm1-joint5','arm1-joint6',\
			'arm1-gripperjoint1','arm1-gripperjoint2','arm1-gripperjoint3',\
			'arm2-joint1','arm2-joint2','arm2-joint3','arm2-joint4','arm2-joint5','arm2-joint6',\
			'arm2-gripperjoint1','arm2-gripperjoint2','arm2-gripperjoint3']
		joint_cmd.position = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
			a1gripper_cmd, a1gripper_cmd, a1gripper_cmd,\
			q2[0], q2[1], q2[2], q2[3], q2[4], q2[5],\
			a2gripper_cmd, a2gripper_cmd, a2gripper_cmd]
		arm_joints.publish(joint_cmd)

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('visualization_2arm', anonymous=True)
		# rospy.Subscriber('joy', Joy, joy_cb)
		rospy.Subscriber('arm_command', Float32MultiArray, arm_cb)
		rospy.Subscriber('/gripper_cmd', Float32MultiArray, grip_cb)
		rviz()

	except rospy.ROSInterruptException:
		pass
