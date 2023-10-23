#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from scipy.spatial.transform import Rotation as R
from forkin_6dof import forkin_6dof
from jacobian_6dof import jacobian_6dof
import tf

config = input("Enter config (1-4):")
if config == 1:
	b1 = np.vstack([0.2306,  0.095525, -0.23432])
	b2 = np.vstack([0.2306, -0.095525, -0.23432])
	Rb1 = np.identity(3)
	Rb2 = np.identity(3)
	q1 = np.vstack([np.pi/4, np.pi/3, -np.pi/3, -np.pi/4, 0, 0])
	q2 = np.vstack([-np.pi/4, np.pi/3, -np.pi/3, np.pi/4, 0, 0])
elif config == 2:
	b1 = np.vstack([0.2356,  0.19, -0.23432])
	b2 = np.vstack([0.2356, -0.19, -0.23432])
	Rb1 = np.identity(3)
	Rb2 = np.identity(3)
	q1 = np.vstack([np.pi/4, np.pi/3, -np.pi/3, -np.pi/4, 0, 0])
	q2 = np.vstack([-np.pi/4, np.pi/3, -np.pi/3, np.pi/4, 0, 0])
elif config == 3:
	b1 = np.vstack([0.1706,  0.175, -0.23432])
	b2 = np.vstack([0.1706, -0.175, -0.23432])
	Rb1 = np.array([[0, -1, 0],\
		[1, 0, 0],\
		[0, 0, 1]])
	Rb2 = np.array([[0, 1, 0],\
		[-1, 0, 0],\
		[0, 0, 1]])
	q1 = np.vstack([-np.pi/2, np.pi/3, np.pi/3, 0, -np.pi/6, np.pi/2])
	q2 = np.vstack([np.pi/2, np.pi/3, np.pi/3, 0, -np.pi/6, -np.pi/2])
elif config == 4:
	b1 = np.vstack([0.1586,  0.19, -0.33052])
	b2 = np.vstack([0.1586, -0.19, -0.33052])
	Rb1 = np.array([[0, 0, 1],\
		[0, 1, 0],\
		[-1, 0, 0]])
	Rb2 = np.array([[0, 0, 1],\
		[0, 1, 0],\
		[-1, 0, 0]])
	q1 = np.vstack([0, -np.pi/3, -np.pi/3, 0, np.pi/6, 0])
	q2 = np.vstack([0, -np.pi/3, -np.pi/3, 0, np.pi/6, 0])


arm_pub = rospy.Publisher('arm_command', Float32MultiArray, queue_size=1)
arm1_pose_pub = rospy.Publisher('arm1/pose', Float32MultiArray, queue_size=1)
arm2_pose_pub = rospy.Publisher('arm2/pose', Float32MultiArray, queue_size=1)
br = tf.TransformBroadcaster()




f1 = forkin_6dof(q1, Rb1, b1)
f2 = forkin_6dof(q2, Rb2, b2)

Xd1 = np.append(f1[0:3, 3, 5], R.as_rotvec(R.from_dcm(f1[0:3, 0:3, 5])), axis=0)
Xd2 = np.append(f2[0:3, 3, 5], R.as_rotvec(R.from_dcm(f2[0:3, 0:3, 5])), axis=0)


lambdap = 20
lambdaw = 20
ep = 0.001
ew = 0.001
vmin = 0.0
vmax = 1.0
wmin = 0.0
wmax = 1.0

rate_HZ = 100.0
dt = 1/rate_HZ



def Xd1_cb(cmd):
	global Xd1
	Xd1 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])



def Xd2_cb(cmd):
	global Xd2
	Xd2 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])



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

		frames1 = forkin_6dof(q1,Rb1,b1)
		frames2 = forkin_6dof(q2,Rb2,b2)
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
		J1 = jacobian_6dof(q1, Rb1, b1)
		J2 = jacobian_6dof(q2, Rb2, b2)
		q1dot = np.dot(pinv(J1), x1dot)
		q2dot = np.dot(pinv(J2), x2dot)
		q1 = q1 + np.vstack(q1dot)*dt
		q2 = q2 + np.vstack(q2dot)*dt

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
		# joint_cmd = JointState()
		# joint_cmd.header.stamp = rospy.get_rostime()
		# joint_cmd.name = ['arm1-joint1','arm1-joint2','arm1-joint3','arm1-joint4','arm1-joint5','arm1-joint6',\
		# 	'arm1-gripperjoint1','arm1-gripperjoint2','arm1-gripperjoint3',\
		# 	'arm2-joint1','arm2-joint2','arm2-joint3','arm2-joint4','arm2-joint5','arm2-joint6',\
		# 	'arm2-gripperjoint1','arm2-gripperjoint2','arm2-gripperjoint3']
		# joint_cmd.position = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
		# 	a1gripper_cmd, a1gripper_cmd, a1gripper_cmd,\
		# 	q2[0], q2[1], q2[2], q2[3], q2[4], q2[5],\
		# 	a2gripper_cmd, a2gripper_cmd, a2gripper_cmd]
		# arm_joints.publish(joint_cmd)

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('RR_controller_2arm', anonymous=True)
		# rospy.Subscriber('joy', Joy, joy_cb)
		rospy.Subscriber('Xd1', Float32MultiArray, Xd1_cb)
		rospy.Subscriber('Xd2', Float32MultiArray, Xd2_cb)
		RR_control()

	except rospy.ROSInterruptException:
		pass
