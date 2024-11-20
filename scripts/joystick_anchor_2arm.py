#!/usr/bin/env python

# Deprecated
# Joystick control of manipulators using anchor points

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from jacobian_6dof import jacobian_6dof
# from invkin_6dof import invkin_6dof
from forkin_6dof import forkin_6dof
from scipy.spatial.transform import Rotation as R

Xd1_pub = rospy.Publisher('Xd1', Float32MultiArray, queue_size=1)
Xd2_pub = rospy.Publisher('Xd2', Float32MultiArray, queue_size=1)

Ra = np.identity(3)
q1 = np.vstack([np.pi/4, np.pi/4, -np.pi/4, -np.pi/4, 0, 0])
q2 = np.vstack([-np.pi/4, np.pi/4, -np.pi/4, np.pi/4, 0, 0])
b1 = np.vstack([0.21813, 0.095525, -0.24012])
b2 = np.vstack([0.21813, -0.095525, -0.24012])

f1 = forkin_6dof(q1, Ra, b1)
f2 = forkin_6dof(q2, Ra, b2)

Pc1 = f1[0:3, 3,  5]
Rc1 = f1[0:3, 0:3,5]
Pc2 = f2[0:3, 3,  5]
Rc2 = f2[0:3, 0:3,5]
Xa1 = Pc1
Xa2 = Pc2
Xd1 = Pc1
Xd2 = Pc2

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
engage_button = 0
engage = False
sP = 0.05
sA = 0.01
rate_HZ = 100.0
dt = 1/rate_HZ

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
	global engage_button
	global engage

	global Xa1
	global Xa2
	global Pc1
	global Pc2

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
	engage_button = data.buttons[3]
	if engage_button and engage == False:
		engage = True
		Xa1 = Pc1
		Xa2 = Pc2
	if not engage_button:
		engage = False



def arm_cb(cmd):
	global q1
	global q2
	q1 = np.vstack([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])
	q2 = np.vstack([cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9], cmd.data[10], cmd.data[11]])


def pose1_cb(cmd):
	global Xa1
	# print(cmd.data)
	Xa1 = cmd.data

def pose2_cb(cmd):
	global Xa2
	# print(cmd.data)
	Xa2 = cmd.data

def joy_anchor():
	# global q1
	# global q2
	# global f1
	# global f2
	global Xd1
	global Xd2
	global Pc1
	global Pc2
	global Xa1
	global Xa2
	global engage

	rate = rospy.Rate(rate_HZ)

	while not rospy.is_shutdown():
		# f1 = forkin_6dof(q1, Ra, b1)
		# f2 = forkin_6dof(q2, Ra, b2)

		# Pc1 = np.append(f1[0:3, 3, 5], R.as_rotvec(R.from_dcm(f1[0:3, 0:3, 5])), axis=0)
		# Pc2 = np.append(f2[0:3, 3, 5], R.as_rotvec(R.from_dcm(f2[0:3, 0:3, 5])), axis=0)

		if engage:
			delta = np.array([sP*joy_ud1, sP*joy_lr1, sP*joy_ud2, sA*joy_lr3, sA*joy_ud3, sA*joy_lr2])
			if arm_left:
				Xd1 = Xa1 + delta
			else: 
				Xd2 = Xa2 + delta

			Xd1_cmd = Float32MultiArray()
			Xd1_cmd.data = Xd1
			Xd1_pub.publish(Xd1_cmd)
			Xd2_cmd = Float32MultiArray()
			Xd2_cmd.data = Xd2
			Xd2_pub.publish(Xd2_cmd)
		# else:
		# 	Xd1 = Xa1
		# 	Xd2 = Xa2

		

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('joystick_anchor_2arm', anonymous=True)
		rospy.Subscriber('joy', Joy, joy_cb)
		# rospy.Subscriber('arm_command', Float32MultiArray, arm_cb)
		rospy.Subscriber('arm1/pose', Float32MultiArray, pose1_cb)
		rospy.Subscriber('arm2/pose', Float32MultiArray, pose2_cb)
		joy_anchor()

	except rospy.ROSInterruptException:
		pass
