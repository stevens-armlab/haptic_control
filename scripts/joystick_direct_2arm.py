#!/usr/bin/env python

# Deprecated
# Directly control joint commands via joystick
# Each joystick axis/button corresponds to a joint angle

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from jacobian_6dof import jacobian_6dof
# from invkin_6dof import invkin_6dof
from forkin_6dof import forkin_6dof
from scipy.spatial.transform import Rotation as R

arm_pub = rospy.Publisher('arm_command', Float32MultiArray, queue_size=1)
arm_joints = rospy.Publisher('joint_states', JointState, queue_size=1)

a1l1_cmd = 0.0
a1l2_cmd = 0.0
a1l3_cmd = 0.0
a1l4_cmd = 0.0
a1l5_cmd = 0.0
a1l6_cmd = 0.0
a1gripper_cmd = 0.0
a2l1_cmd = 0.0
a2l2_cmd = 0.0
a2l3_cmd = 0.0
a2l4_cmd = 0.0
a2l5_cmd = 0.0
a2l6_cmd = 0.0
a2gripper_cmd = 0.0

joy_lr1 = 0.0
joy_ud1 = 0.0
joy_lr2 = 0.0
joy_ud2 = 0.0
joy_lr3 = 0.0
joy_ud3 = 0.0
gripper_open = 0
gripper_close = 0
gripper_joint = 0
arm_button = 0
arm_left = True

rate_HZ = 100.0
dt = 1/rate_HZ

# X = np.array([700, 0, 0, 0, 0, 0])
# Re = R.as_dcm(R.from_rotvec(X[3:6]))
# q = invkin_6dof(X[0:3], Re)

def joy_cb(data):
	global joy_lr1
	global joy_ud1
	global joy_lr2
	global joy_ud2
	global joy_lr3
	global joy_ud3

	global gripper_open
	global gripper_close
	global arm_button
	global arm_left

	joy_lr1 = data.axes[0]
	joy_ud1 = data.axes[1]
	joy_lr2 = data.axes[2]
	joy_ud2 = data.axes[3]
	joy_lr3 = data.axes[4]
	joy_ud3 = data.axes[5]

	gripper_open = data.buttons[1]
	gripper_close = data.buttons[2]
	arm_button = data.buttons[0]
	if arm_button:
		arm_left = not arm_left


def home():
	global a1l1_cmd
	global a1l2_cmd
	global a1l3_cmd
	global a1l4_cmd
	global a1l5_cmd
	global a1l6_cmd
	global a1gripper_cmd
	global a2l1_cmd
	global a2l2_cmd
	global a2l3_cmd
	global a2l4_cmd
	global a2l5_cmd
	global a2l6_cmd
	global a2gripper_cmd


	
	global joy_ud1
	global joy_lr1
	global joy_ud2
	global joy_lr2
	global joy_ud3
	global joy_lr3
	global gripper_open
	global gripper_close
	global arm_button

	rate = rospy.Rate(rate_HZ)

	while not rospy.is_shutdown():
		if arm_left:
			a1l1_cmd = a1l1_cmd + joy_lr1*dt*5
			a1l2_cmd = a1l2_cmd + joy_ud1*dt*5
			a1l3_cmd = a1l3_cmd + joy_ud2*dt*5
			a1l4_cmd = a1l4_cmd + joy_lr2*dt*5
			a1l5_cmd = a1l5_cmd + joy_ud3*dt*5
			a1l6_cmd = a1l6_cmd + joy_lr3*dt*5
			if gripper_open == 1:
				a1gripper_cmd = a1gripper_cmd + dt*5
			if gripper_close == 1:
				a1gripper_cmd = a1gripper_cmd - dt*5
			if a1gripper_cmd > 0.5:
				a1gripper_cmd = 0.5
			if a1gripper_cmd < -0.5:
				a1gripper_cmd = -0.5
		else:
			a2l1_cmd = a2l1_cmd + joy_lr1*dt*5
			a2l2_cmd = a2l2_cmd + joy_ud1*dt*5
			a2l3_cmd = a2l3_cmd + joy_ud2*dt*5
			a2l4_cmd = a2l4_cmd + joy_lr2*dt*5
			a2l5_cmd = a2l5_cmd + joy_ud3*dt*5
			a2l6_cmd = a2l6_cmd + joy_lr3*dt*5
			if gripper_open == 1:
				a2gripper_cmd = a2gripper_cmd + dt*5
			if gripper_close == 1:
				a2gripper_cmd = a2gripper_cmd - dt*5
			if a2gripper_cmd > 0.5:
				a2gripper_cmd = 0.5
			if a2gripper_cmd < -0.5:
				a2gripper_cmd = -0.5


		arm_cmd_data = Float32MultiArray()
		arm_cmd_data.data = [a1l1_cmd, a1l2_cmd, a1l3_cmd, a1l4_cmd, a1l5_cmd, a1l6_cmd,\
			a1gripper_cmd, a1gripper_cmd, a1gripper_cmd,\
			a2l1_cmd, a2l2_cmd, a2l3_cmd, a2l4_cmd, a2l5_cmd, a2l6_cmd,\
			a2gripper_cmd, a2gripper_cmd, a2gripper_cmd]
		arm_pub.publish(arm_cmd_data)

		joint_cmd = JointState()
		joint_cmd.header.stamp = rospy.get_rostime()
		joint_cmd.name = ['arm1-joint1','arm1-joint2','arm1-joint3','arm1-joint4','arm1-joint5','arm1-joint6',\
			'arm1-gripperjoint1','arm1-gripperjoint2','arm1-gripperjoint3',\
			'arm2-joint1','arm2-joint2','arm2-joint3','arm2-joint4','arm2-joint5','arm2-joint6',\
			'arm2-gripperjoint1','arm2-gripperjoint2','arm2-gripperjoint3']
		joint_cmd.position = [a1l1_cmd, a1l2_cmd, a1l3_cmd, a1l4_cmd, a1l5_cmd, a1l6_cmd,\
			a1gripper_cmd, a1gripper_cmd, a1gripper_cmd,\
			a2l1_cmd, a2l2_cmd, a2l3_cmd, a2l4_cmd, a2l5_cmd, a2l6_cmd,\
			a2gripper_cmd, a2gripper_cmd, a2gripper_cmd]
		arm_joints.publish(joint_cmd)

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('arm_control_joystick', anonymous=True)
		rospy.Subscriber('joy', Joy, joy_cb)
		home()

	except rospy.ROSInterruptException:
		pass
