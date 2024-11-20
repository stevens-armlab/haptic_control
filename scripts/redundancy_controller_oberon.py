#!/usr/bin/env python

# Converts task space commands to joint space commands
# Publishes joint commands to 'arm_command' topic

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from scipy.spatial.transform import Rotation as R
from forkin_oberon import forkin_oberon
from jacobian_oberon import jacobian_oberon
import tf


# 	q1 = np.vstack([0, -np.pi/3, -np.pi/3, 0, np.pi/6, 0])
# 	q2 = np.vstack([0, -np.pi/3, -np.pi/3, 0, np.pi/6, 0])
b1 = np.vstack([1.3, -0.5, -0.615])
b2 = np.vstack([1.3, 0.5, -0.615])
Rb1 = np.identity(3)
Rb2 = np.identity(3)
# q1 = np.vstack([0, 0, 0, 0, 0, 0])
# q2 = np.vstack([0, 0, 0, 0, 0, 0])
# q1 = np.vstack([0.4, 0.5, -0.5, 0.4, 1.5708, 0])
# q2 = np.vstack([-0.4, 0.5, -0.5, -0.4, 1.5708, 0])
q1 = np.vstack([0, 0.8, -0.7, 0, 1.5, 0])
q2 = np.vstack([0, 0.8, -0.7, 0, 1.5, 0])

arm_pub = rospy.Publisher('arm_command', Float32MultiArray, queue_size=1)
arm1_pose_pub = rospy.Publisher('arm1/pose', Float32MultiArray, queue_size=1)
arm2_pose_pub = rospy.Publisher('arm2/pose', Float32MultiArray, queue_size=1)
grip_pub = rospy.Publisher('gripper_cmd', Float32MultiArray, queue_size=1)
joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
br = tf.TransformBroadcaster()




f1 = forkin_oberon(q1, Rb1, b1)
f2 = forkin_oberon(q2, Rb2, b2)

Xd1 = np.append(f1[0:3, 3, 5], R.as_rotvec(R.from_dcm(f1[0:3, 0:3, 5])), axis=0)
Xd2 = np.append(f2[0:3, 3, 5], R.as_rotvec(R.from_dcm(f2[0:3, 0:3, 5])), axis=0)

L_Grip = None
R_Grip = None
open_grip = 1.0
close_grip = 0.0
grip_rate = 0.01
L_gripper_pose = 0.0
R_gripper_pose = 0.0

lambdap = 20
lambdaw = 20
ep = 0.001
ew = 0.001
vmin = 0.0
vmax = 0.1
wmin = 0.0
wmax = 0.5

rate_HZ = 100.0
dt = 1/rate_HZ



def Xd1_cb(cmd):
	global Xd1
	Xd1 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])



def Xd2_cb(cmd):
	global Xd2
	Xd2 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])



def grip_cb(cmd):
	"""
    stores the desired gripper state
    1 : Close the gripper (Grasp)
    2 : Open the gripper (Release)
    """
	global L_Grip, R_Grip
	L_Grip, R_Grip = cmd.data[0], cmd.data[1]



def pinv(M):
	size = M.shape
	n = size[0]
	Mplus = np.dot(M.transpose(),\
		np.linalg.inv(np.dot(M, M.transpose()) + 1e-6*np.identity(n)))
	return Mplus



def RR_rates(Pc, Rc, Pd, Rd):
	Re = np.dot(Rd, Rc.transpose())
	th_e = np.arccos((np.trace(Re)-1)*0.5)
	if th_e == 0:
		m_e = np.array([0.0, 0.0, 0.0])
	else:
		m_e = (2*np.sin(th_e))**(-1)*np.array([Re[2,1]-Re[1,2],\
			Re[0,2]-Re[2,0],\
			Re[1,0]-Re[0,1]])

	errorp = Pd - Pc
	errorw = th_e*m_e
	deltap = np.linalg.norm(errorp)
	deltaw = np.linalg.norm(errorw)
	if deltap == 0:
		n = np.array([0.0, 0.0, 0.0])
	else:
		n = errorp/deltap
	if deltap > lambdap*ep:
		vmag = vmax
	else:
		vmag = vmin + (vmax - vmin)*(deltap - ep)/(ep*(lambdap - 1))
	if deltaw > lambdaw*ew:
		wmag = wmax
	else:
		wmag = wmin + (wmax - wmin)*(deltaw - ew)/(ew*(lambdaw - 1))

	pdot = vmag*n
	wdot = wmag*m_e
	xdot = np.append(pdot, wdot, axis=0)

	return xdot


def gripper_control():
	global L_gripper_pose
	global R_gripper_pose
	if (L_Grip == 0) and (L_gripper_pose > close_grip):
		# If desired to be closed AND grip isnt fully closed
		L_gripper_pose += -grip_rate
	if (R_Grip == 0) and (R_gripper_pose > close_grip):
		# If desired to be closed AND grip isnt fully closed
		R_gripper_pose += -grip_rate
	if (L_Grip == 1) and (L_gripper_pose < open_grip):
		# If desired to be opened AND grip isnt fully open
		L_gripper_pose += grip_rate
	if (R_Grip == 1) and (R_gripper_pose < open_grip):
		# If desired to be opened AND grip isnt fully open
		R_gripper_pose += grip_rate
	grip_cmd = Float32MultiArray()
	grip_cmd.data = [L_gripper_pose, R_gripper_pose]
	grip_pub.publish(grip_cmd)

def RR_control():
	global q1
	global q2
	global Xd1
	global Xd2

	rate = rospy.Rate(rate_HZ)

	while not rospy.is_shutdown():

		Pd1 = Xd1[0:3]
		Pd2 = Xd2[0:3]
		Rd1 = R.as_dcm(R.from_rotvec(Xd1[3:6]))
		Rd2 = R.as_dcm(R.from_rotvec(Xd2[3:6]))

		frames1 = forkin_oberon(q1,Rb1,b1)
		frames2 = forkin_oberon(q2,Rb2,b2)
		Pc1 = frames1[0:3,3,5]
		Pc2 = frames2[0:3,3,5]
		Rc1 = frames1[0:3,0:3,5]
		Rc2 = frames2[0:3,0:3,5]
		rv1 = R.as_rotvec(R.from_dcm(Rc1))
		rv2 = R.as_rotvec(R.from_dcm(Rc2))
		quat1 = R. as_quat(R.from_dcm(Rc1))
		quat2 = R. as_quat(R.from_dcm(Rc2))
		x1dot = RR_rates(Pc1, Rc1, Pd1, Rd1)
		x2dot = RR_rates(Pc2, Rc2, Pd2, Rd2)
		J1 = jacobian_oberon(q1, Rb1, b1)
		J2 = jacobian_oberon(q2, Rb2, b2)
		q1dot = np.dot(pinv(J1), x1dot)
		q2dot = np.dot(pinv(J2), x2dot)
		q1 = q1 + np.vstack(q1dot)*dt
		q2 = q2 + np.vstack(q2dot)*dt

		print("======================")
		print("")
		print("Desired Pose 1: " + str(Xd1))
		print("")
		print("Current Pose 1:" + str(np.concatenate((Pc1,rv1), axis=0)))
		print("")
		print("Q1: " + str(q1))
		print("")

		arm_cmd = Float32MultiArray()
		arm_cmd.data = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
			q2[0], q2[1], q2[2], q2[3], q2[4], q2[5]]
		arm_pub.publish(arm_cmd)

		pose1_cmd = Float32MultiArray()
		pose1_cmd.data = np.concatenate((Pc1, rv1), axis=0)
		arm1_pose_pub.publish(pose1_cmd)

		pose2_cmd = Float32MultiArray()
		pose2_cmd.data = np.concatenate((Pc2, rv2), axis=0)
		arm2_pose_pub.publish(pose2_cmd)
		br.sendTransform((frames1[0,3,5], frames1[1,3,5], frames1[2,3,5]),\
			quat1,\
			rospy.Time.now(),\
			"EE1",\
			"base_link")
		br.sendTransform((frames2[0,3,5], frames2[1,3,5], frames2[2,3,5]),\
			quat2,\
			rospy.Time.now(),\
			"EE2",\
			"base_link")

		joint_cmd = JointState()
		joint_cmd.header.stamp = rospy.get_rostime()
		joint_cmd.name = ['azimuthL', 'shoulderL', 'elbowL', 'rollL', 'pitchL', 'wristL',\
			'finger_left_jointL','finger_tip_left_jointL',\
			'finger_right_jointL','finger_tip_right_jointL',\
			'azimuthR', 'shoulderR', 'elbowR', 'rollR', 'pitchR', 'wristR',\
			'finger_left_jointR','finger_tip_left_jointR',\
			'finger_right_jointR','finger_tip_right_jointR']
		joint_cmd.position = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
			L_gripper_pose, 0.0, L_gripper_pose, 0.0,\
			q2[0], q2[1], q2[2], q2[3], q2[4], q2[5],\
			R_gripper_pose, 0.0, R_gripper_pose, 0.0]
		joint_pub.publish(joint_cmd)
		gripper_control()

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('RR_controller_2arm', anonymous=True)
		# rospy.Subscriber('joy', Joy, joy_cb)
		rospy.Subscriber('Xd1', Float32MultiArray, Xd1_cb)
		rospy.Subscriber('Xd2', Float32MultiArray, Xd2_cb)
		rospy.Subscriber('/grippers', Float32MultiArray, grip_cb)
		RR_control()

	except rospy.ROSInterruptException:
		pass
