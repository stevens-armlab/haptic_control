#!/usr/bin/env python

# Moves manipulator to desired pose using smooth 5th order polynomial
# Helps prevent crashing in Gazebo

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState
from scipy.spatial.transform import Rotation as R
from forkin_oberon import forkin_oberon
from jacobian_oberon import jacobian_oberon
import tf
import csv
import rospkg



# Edit these values to set desired pose:
# q1_des = np.array([0.4, 0.5, -0.5, 0.4, 1.570796326794897, 0])
# q2_des = np.array([-0.4, 0.5, -0.5, -0.4, 1.570796326794897, 0])
# q3_des = np.array([0, 1.2, -0.8, 0, 1.170796326794897, 0])

# q1_des = np.array([0, 0.3, -0.3, 0, 1.570796326794897, 0])
# q2_des = np.array([0, 0.3, -0.3, 0, 1.570796326794897, 0])
# q3_des = np.array([0, 0.3, -0.3, 0, 1.570796326794897, 0])



# # Fence Gauss Initial (all)
# q1_des = np.array([0.103358692158543, 0.189400922283478, 0.275724132346155, 0.115419507318957, 1.10829982450360, -6.98775986413805e-17])
# q2_des = np.array([-0.103677795761425, 0.189119260556039, 0.276727841277373, -0.115661367196912, 1.10816237084974, 7.20243681963352e-17])
# q3_des = np.array([-0.000121838527937644, 0.793565709406123, 0.872995186105687, -0.000480594824473146, -0.0342406922749476, 1.37370507850148e-20])

# # Fence Gauss Alpha 1 Optimized
# q1_des = np.array([0.342559196185885, 1.08730098297676, -1.26659375663757, 0.429706565018176, 1.74446259916428, -1.15725313876028e-17])
# q2_des = np.array([-0.181805166323380, 1.24601173703841, -1.49482519737790, -0.0990435802269892, 1.83702086902302, -1.56106276190605e-18])
# q3_des = np.array([-0.0715634248391338, 1.49407613568548, -1.40359881927172, 0.0142079070397883, 1.52837337030000, -7.95502006443004e-17])

# # Fence Gauss Alpha 2 Optimized
# q1_des = np.array([-0.300239742927328, -0.109679620571518, 1.04637211995585, -0.0946474907665997, 0.610026703879450, 1.39830955314116e-16])
# q2_des = np.array([-0.744072514518678, -0.299501658035207, 0.980942219249053, -0.601766888070849, 0.960668872402063, -4.51121707116264e-18])
# q3_des = np.array([-0.225765565538801, 0.734787268317295, 0.845922079286079, 0.0418670910358823, 0.0764963147740985, -8.87616190020393e-17])

# # Fence Gauss Alpha 3 Optimized
# q1_des = np.array([0.461472732529240, 1.29142080918206, -1.07454173616017, 0.242691383443921, 1.39491150457029, -1.45482565998892e-17])
# q2_des = np.array([-0.645849945888418, -0.327050311139198, 1.04657896536963, -0.947970094759853, 1.05610857702201, -6.15293957056751e-17])
# q3_des = np.array([0.255760471586142, 1.49397728859871, -0.755588853090559, 0.0580056693886460, 0.832124181302375, 4.90377554875334e-17])

# # Fence Gauss Alpha 4 Optimized
# q1_des = np.array([-0.286492643594208, -0.137516195527770, 1.04520976161181, -0.117176105912268, 0.644614352709327, 4.99405969597979e-17])
# q2_des = np.array([-0.692008022308823, -0.278718147423966, 0.968890005571520, -0.544359518583441, 0.909475839505627, 1.43094396548370e-16])
# q3_des = np.array([-0.208248992923121, 0.729381596017934, 0.838216621391643, -0.00113171052175112, 0.0876717088350412, 6.27647913041307e-18])



# Fence Initial Conditions 1
# q1_des = np.array([0.339923050654094, 1.18954113182383, -1.40184149239308, 0.361769930071883, 1.77768609983320, 5.41044246612306e-17])
# q2_des = np.array([-0.132789003504416, 1.18567794172417, -1.46601531747321, -0.120068968227858, 1.86485763610126, 2.18739239124353e-17])
# q3_des = np.array([-0.0459431059271275, 1.52024100357282, -1.44002065800726, -0.0284685656291339, 1.56170476942259, -3.39588995625089e-19])

# # Fence Initial Conditions 2
# q1_des = np.array([0.353034981933468 0.804028893319854 -0.823846047462472 0.268959235693828 1.59527093181556 -1.05238192907891e-16])
# q2_des = np.array([0.313609634883929 0.624153911444988 -0.616756857573428 0.232487172797298 1.56148678007230 7.27963839339911e-18])
# q3_des = np.array([0.0567406640060678 1.57069852278862 -0.207055781936120 -0.0917495191200168 0.206674378777762 1.27960763031905e-18])

# # Fence Initial Conditions 3
# q1_des = np.array([0.788891038639263 1.43299893636661 -1.23863891503486 0.624746700611451 1.47103794408751 1.80413791914411e-16])
# q2_des = np.array([-0.579233321918273 -0.907279034729841 1.04691198724004 -0.700424208736092 1.38927882592809 -1.08862725210545e-16])
# q3_des = np.array([0.419398927261547 1.51184731167971 -1.20407422335827 0.272085674692300 1.32576999237830 2.35265736494471e-16])

# # Fence Initial Conditions 4
# q1_des = np.array([0.424938520754829 0.987736572051643 -1.31862591063576 0.484310539765264 1.88115651638020 -8.03565877829363e-18])
# q2_des = np.array([-0.134896433021313 1.37061742274109 -1.48426828467354 -0.0914520022178611 1.70378945265440 2.94585642838847e-16])
# q3_des = np.array([-0.0343612261879967 1.52182631429093 -1.43175525578982 0.0101429403130432 1.54975197532522 1.94265299137766e-16])

# # Fence Initial Conditions 5
# q1_des = np.array([-1.04712544956026 -0.414599515844500 1.04380328010156 -0.246149470573832 0.932975272462150 -1.65638722481348e-16])
# q2_des = np.array([0.470409593523305 0.836397629586145 -0.275330048179232 1.34623626172882 1.43100257351517 6.36304814767179e-17])
# q3_des = np.array([-0.927212188542262 1.54942840217677 0.0956564579479963 0.114829898105469 -0.0626230499461675 -6.74415169516807e-18])

# # Fence Initial Conditions 6
# q1_des = np.array([-0.633579971226239 -0.863575302401831 0.988099280096899 -0.826275551617586 1.45157264528583 5.06367209101194e-17])
# q2_des = np.array([-0.786311317018642 -0.0621253711110262 1.04670065323991 -1.17076545214646 1.02495920068949 -7.46182489509581e-17])
# q3_des = np.array([0.768394508739685 1.56657969849990 -0.200768588725233 1.16434156879094 0.593709207430918 2.17135353159026e-17])

# # Fence Initial Conditions 7
# q1_des = np.array([0.343051692704855 1.18080591903726 -1.05551562559381 0.131089721357236 1.44576167185530 3.96679134951744e-17])
# q2_des = np.array([-0.674994521962159 0.411544039290637 1.04282655504668 -1.42316265209552 0.844485812909236 -6.82428478526958e-17])
# q3_des = np.array([0.313408594441070 1.57054687038641 -0.494886677861758 0.205202349466412 0.502234440591244 4.95070836429295e-18])

# # Fence Initial Conditions 8
# q1_des = np.array([0.508715554247986 1.25574043424979 -1.17496373365489 0.355856406389206 1.49776715633625 -3.60661949831144e-17])
# q2_des = np.array([-0.908155032079846 0.0345550732071951 1.04320497929144 -1.27324807183932 1.06719598643175 -2.03315066229264e-17])
# q3_des = np.array([0.278244498764565 1.56856779251662 -0.967636876657873 0.135012074182092 1.02122667246999 -5.21424906310771e-17])


config_case = 8

# config_file = 'final_scenarios'
# config_file = 'fence_multiconfig_init'
# config_file = 'fence_multiconfig_init_Fx'
# config_file = 'fence_multiconfig_opt_Fx'
# config_file = 'fence_multiconfig_init_Fy'
# config_file = 'fence_multiconfig_opt_Fy'
# config_file = 'fence_multiconfig_init_Fz'
# config_file = 'fence_multiconfig_opt_Fz'

# config_file = 'pipe_multiconfig_init_Fx'
# config_file = 'pipe_multiconfig_opt_Fx'
# config_file = 'pipe_multiconfig_init_Fy'
# config_file = 'pipe_multiconfig_opt_Fy'
# config_file = 'pipe_multiconfig_init_Fz'
config_file = 'pipe_multiconfig_opt_Fz'

rp = rospkg.RosPack()
path = rp.get_path('uvms-perching')
newpath = path + '/src/configuration_files/' + config_file + '.csv'
with open(newpath, 'rb') as csvfile:
	config_data = np.array(list(csv.reader(csvfile)))
config_data = config_data.astype(np.float)

q_des = config_data[::, config_case-1]



rate_HZ = 100.0
dt = 1/rate_HZ
duration = 20.0

# q1_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q2_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q3_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q3 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_start = np.zeros(18)
# q = np.zeros(18)

arm_pub = rospy.Publisher('arm_command', Float32MultiArray, queue_size=1)

Grip1 = None
Grip2 = None
Grip3 = None

open_grip = 1.0
close_grip = 0.0
grip_rate = 0.01

arm1_gripper_pose = 0.0
arm2_gripper_pose = 0.0
arm3_gripper_pose = 0.0



def joint_smooth():
	# global q1, q2, q3
	# global q
	rate = rospy.Rate(rate_HZ)
	rospy.sleep(1)
	start_time = rospy.Time.now()

	while not rospy.is_shutdown():
		now_time = rospy.Time.now()
		sim_time = now_time - start_time
		t = (sim_time.secs + sim_time.nsecs*1e-9)/duration
		print(t)

		if t < 0:
			S = 0
		elif t < 1:
			S = 3*(t**2) - 2*(t**3)
		else:
			S = 1

		# q1 = q1_start + S*(q1_des - q1_start)
		# q2 = q2_start + S*(q2_des - q2_start)
		# q3 = q3_start + S*(q3_des - q3_start)

		# print(q1)
		q = q_start + S*(q_des - q_start)


		arm_cmd = Float32MultiArray()
		# arm_cmd.data = [q1[0], q1[1], q1[2], q1[3], q1[4], q1[5],\
		# 	q2[0], q2[1], q2[2], q2[3], q2[4], q2[5],\
		# 	q3[0], q3[1], q3[2], q3[3], q3[4], q3[5]]
		arm_cmd.data = [q[0], q[1], q[2], q[3], q[4], q[5],\
			q[6], q[7], q[8], q[9], q[10], q[11],\
			q[12], q[13], q[14], q[15], q[16], q[17]]
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
