#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from scipy.spatial.transform import Rotation as R
from forkin_oberon import forkin_oberon
from jacobian_oberon import jacobian_oberon
import tf


# Edit these values to set desired pose:
# q1_des = np.array([0.4, 0.5, -0.5, 0.4, 1.570796326794897, 0])
# q2_des = np.array([-0.4, 0.5, -0.5, -0.4, 1.570796326794897, 0])
q1_des = np.array([0.4, 0.5, -0.5, 0.4, 1.570796326794897, 0])
q2_des = np.array([-0.4, 0.5, -0.5, -0.4, 1.570796326794897, 0])


rate_HZ = 100.0
dt = 1/rate_HZ
duration = 10.0

q1_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])



q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

arm_pub = rospy.Publisher('arm_command', Float32MultiArray, queue_size=1)

L_Grip = None
R_Grip = None
open_grip = 1.0
close_grip = 0.0
grip_rate = 0.01
L_gripper_pose = 0.0
R_gripper_pose = 0.0



def joint_smooth():
	global q1, q2
	rate = rospy.Rate(rate_HZ)
	rospy.sleep(1)
	start_time = rospy.Time.now()

	while not rospy.is_shutdown():
		now_time = rospy.Time.now()
		sim_time = now_time - start_time
		t = (sim_time.secs + sim_time.nsecs*1e-9)/duration
		# print(start_time.secs)
		# print(now_time)
		# print(t)
		if t < 0:
			S = 0
		elif t < 1:
			S = 3*(t**2) - 2*(t**3)
		else:
			S = 1

		q1 = q1_start + S*(q1_des - q1_start)
		q2 = q2_start + S*(q2_des - q2_start)

		print(q1)


		arm_cmd = Float32MultiArray()
		arm_cmd.data = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
			q2[0], q2[1], q2[2], q2[3], q2[4], q2[5]]
		arm_pub.publish(arm_cmd)


		rate.sleep()
		if t > 1:
			break



if __name__ == '__main__':
	try:
		rospy.init_node('joint_control_smooth', anonymous=True)
		joint_smooth()

	except rospy.ROSInterruptException:
		pass
