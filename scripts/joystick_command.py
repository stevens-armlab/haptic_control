#!/usr/bin/env python

# Joystick/gamepad control for bimanual UVMS teleop
# 'rosrun joy joy_node' to connect device
# Publishes task space commands on topics 'Xd1' and 'Xd2', and gripper open/close commands

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from scipy.spatial.transform import Rotation as R
from forkin_oberon import forkin_oberon
from jacobian_oberon import jacobian_oberon
import tf

# Joystick input variables
joyL_lr = 0.0
joyL_ud = 0.0
joyR_lr = 0.0
joyR_ud = 0.0
pad_lr = 0.0
pad_ud = 0.0

buttonX = 0.0
buttonA = 0.0
buttonB = 0.0
buttonY = 0.0
LB = 0.0
RB = 0.0
LT = 0.0
RT = 0.0
buttonBACK = 0.0
buttonSTART = 0.0
stickL = 0.0
stickR = 0.0

home_button = False

rate_HZ = 100.0
dt = 1/rate_HZ
dx = 0.1 # m/s

arm1_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm2_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# arm1_rot = [0.0, 0.0, 0.0]
# arm2_pos = [0.0, 0.0, 0.0]
# arm2_rot = [0.0, 0.0, 0.0]

pub_x1 = rospy.Publisher('Xd1', Float32MultiArray, queue_size=1)
pub_x2 = rospy.Publisher('Xd2', Float32MultiArray, queue_size=1)
pub_grip = rospy.Publisher('grippers', Float32MultiArray, queue_size=1)
arm1_received = False
arm2_received = False
Xd1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Xd2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gripper_cmd = [0.0, 0.0]
gripL = False
gripR = False


# ROS message callback for joystick input
def joy_cb(data):
	global joyL_lr, joyL_ud, joyR_lr, joyR_ud, pad_lr, pad_ud
	global buttonX, buttonA, buttonB, buttonY, LB, RB, LT, RT, buttonBACK, buttonSTART, stickL, stickR
	global home_button
	global gripper_cmd, gripL, gripR

	joyL_lr = data.axes[0]
	joyL_ud = data.axes[1]
	joyR_lr = data.axes[2]
	joyR_ud = data.axes[3]
	pad_lr = data.axes[4]
	pad_ud = data.axes[5]

	buttonX = data.buttons[0]
	buttonA = data.buttons[1]
	buttonB = data.buttons[2]
	buttonY = data.buttons[3]
	LB = data.buttons[4]
	RB = data.buttons[5]
	LT = data.buttons[6]
	RT = data.buttons[7]
	buttonBACK = data.buttons[8]
	buttonSTART = data.buttons[9]
	stickL = data.buttons[10]
	stickR = data.buttons[11]

	if buttonSTART:
		home_button = True
	else:
		home_button = False

	if LB == 1:
		gripper_cmd[0] = 1
	elif LB == 0:
		gripper_cmd[0] = 0

	if RB == 1:
		gripper_cmd[1] = 1
	elif RB == 0:
		gripper_cmd[1] = 0

	gripper_data = Float32MultiArray()
	gripper_data.data = gripper_cmd
	pub_grip.publish(gripper_data)



def arm1_cb(msg):
	global arm1_pose, arm1_received, Xd1
	arm1_pose[0:5] = msg.data[0:5]

	if arm1_received == False:
		Xd1[0:5] = arm1_pose[0:5]
		arm1_received = True



def arm2_cb(msg):
	global arm2_pose, arm2_received, Xd2
	arm2_pose[0:5] = msg.data[0:5]

	if arm2_received == False:
		Xd2[0:5] = arm2_pose[0:5]
		arm2_received = True



def serial_control():
	global arm1_pose, arm2_pose, arm1_received, arm2_received
	global joyL_lr, joyL_ud, joyR_lr, joyR_ud, pad_lr, pad_ud
	global buttonX, buttonA, buttonB, buttonY, LB, RB, LT, RT, buttonBACK, buttonSTART, stickL, stickR, home_button
	global q1, q2
	global Xd1, Xd2


	Xd1[0] = Xd1[0] + joyL_ud*dt*dx
	Xd1[1] = Xd1[1] + joyL_lr*dt*dx
	Xd1[2] = Xd1[2] + pad_ud*dt*dx
	Xd1[5] = Xd1[5] + pad_lr*dt*dx

	Xd2[0] = Xd2[0] + joyR_ud*dt*dx
	Xd2[1] = Xd2[1] + joyR_lr*dt*dx
	if buttonY == 1 and buttonA == 0:
		Xd2[2] = Xd2[2] + dt*dx
	elif buttonY == 0 and buttonA == 1:
		Xd2[2] = Xd2[2] - dt*dx
	if buttonB == 1 and buttonX == 0:
		Xd2[5] = Xd2[5] - dt*dx
	elif buttonB == 0 and buttonX == 1:
		Xd2[5] = Xd2[5] + dt*dx

	# print(Xd1)
	Xd1_data = Float32MultiArray()
	Xd1_data.data = Xd1
	pub_x1.publish(Xd1_data)

	Xd2_data = Float32MultiArray()
	Xd2_data.data = Xd2
	pub_x2.publish(Xd2_data)






def RR_control():
	global arm1_received
	global arm2_received
	rate = rospy.Rate(rate_HZ)

	while not rospy.is_shutdown():
		if arm1_received == True and arm2_received == True:
			serial_control()
		rate.sleep()



if __name__ == '__main__':
	try:
		rospy.init_node('joystick_command_node', anonymous=True)
		rospy.Subscriber('joy', Joy, joy_cb)
		rospy.Subscriber('arm1/pose', Float32MultiArray, arm1_cb)
		rospy.Subscriber('arm2/pose', Float32MultiArray, arm2_cb)
		# rospy.Subscriber('Xd1', Float32MultiArray, Xd1_cb)
		# rospy.Subscriber('Xd2', Float32MultiArray, Xd2_cb)
		# rospy.Subscriber('/grippers', Float32MultiArray, grip_cb)
		RR_control()

	except rospy.ROSInterruptException:
		pass
