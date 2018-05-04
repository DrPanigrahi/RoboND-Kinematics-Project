#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author(s): Harsh Pandya, Smruti Panigrahi

########################################################################################
#############                         IMPORT MODULES                       #############
########################################################################################
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from time import time
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *



########################################################################################
#############                   DEFINE DH PARAMETER SYMBOLS                #############
########################################################################################
# Link lengths: d1:8
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
# Link offsets: a0:7
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
# Twist angles: alpha0:7
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
# Joint angles: q1:8
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

########################################################################################
#############             CREATE DH PARAMETER TABLE/DICTIONARY             #############
########################################################################################
# DH Parameter for KUKA210 Robot Manipulator
DH_TABLE = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:       q1, 
			alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: -pi/2+q2,
			alpha2:     0,  a2:   1.25,  d3:     0,  q3:       q3,
			alpha3: -pi/2,  a3: -0.054,  d4:   1.5,  q4:       q4,
			alpha4:  pi/2,  a4:      0,  d5:     0,  q5:       q5,
			alpha5: -pi/2,  a5:      0,  d6:     0,  q6:       q6,
			alpha6:     0,  a6:      0,  d7: 0.303,  q7:        0}

########################################################################################
#############        USE ATAN2 TO GET SIGNED THETA FROM X=COS(THETA)       #############
########################################################################################
# Returns signed cosine inverse
#cos_inv = lambda X: acos(X)
cos_inv = lambda X: atan2(sqrt(1 - X**2), X)
	
########################################################################################
#############                         LAW OF COSINES                       #############
########################################################################################
# Law of cosines for any triangle with sides (a,b,c): c^2 = a^2 + b^2 - 2a*b*cos(theta)
# Returns cosine of angle between sides a and b
cos_law = lambda a, b, c: (a**2 + b**2 - c**2) / (2 * a * b) 

########################################################################################
#############          DEFINE GENERIC COORDINATE ROTATION MATRICES         #############
########################################################################################
def URDF2DH(r, p, y):
	URDF2DH_ROT_X = Matrix([[      1,           0,          0 ],
							[      0,      cos(r),    -sin(r) ],
							[      0,      sin(r),     cos(r) ]])

	URDF2DH_ROT_Y = Matrix([[ cos(p),           0,     sin(p) ],
							[      0,           1,          0 ],
							[-sin(p),           0,     cos(p) ]])

	URDF2DH_ROT_Z = Matrix([[ cos(y),     -sin(y),          0 ],
							[ sin(y),      cos(y),          0 ],
							[      0,           0,          1 ]])
	return URDF2DH_ROT_X, URDF2DH_ROT_Y, URDF2DH_ROT_Z

########################################################################################
#############           DEFINE HOMOGENEOUS TRANSFORMATION MATRIX           #############
########################################################################################
def TF_MATRIX(alpha, a, d, q):
	TF_MAT = Matrix([
		[            cos(q),             -sin(q),            0,               a],
		[ sin(q)*cos(alpha),   cos(q)*cos(alpha),  -sin(alpha),   -sin(alpha)*d],
		[ sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),    cos(alpha)*d],
		[                 0,                   0,            0,               1]
		])
	return TF_MAT



########################################################################################
#############        ROOT-MEAN-SQUARED-ERROR (RMSE) OF END-EFFECTOR        #############
########################################################################################
class RMSE:
	def __init__(self):
		self.cumulative_error = None
		self.count = 0

	def add(self, vector, vector_estimate):
		if self.cumulative_error is None:
			self.cumulative_error = np.zeros_like(vector)

		self.cumulative_error += np.square(vector_estimate - vector)
		self.count += 1

	def getError(self):
		return np.sqrt(self.cumulative_error/self.count)


RMSE_EE = RMSE()
	
########################################################################################
#############                      FORWARD KINEMATICS                      #############
#############              >> GIVEN JOINT ANGLES OF ALL JOINTS             #############
#############        >> FIND END-EFFECTOR POSITION AND ORIENTATION         #############
########################################################################################
def forward_kinematics():
	########################################################
	###### EVALUATE ALL TRANSFORMATION MATRIXCES AND #######
	###### SUBSTITUTE PARAMETER VALUES FROM DH-TABLE #######
	########################################################
	T0_1 = TF_MATRIX(alpha0, a0, d1, q1).subs(DH_TABLE)
	T1_2 = TF_MATRIX(alpha1, a1, d2, q2).subs(DH_TABLE)
	T2_3 = TF_MATRIX(alpha2, a2, d3, q3).subs(DH_TABLE)
	T3_4 = TF_MATRIX(alpha3, a3, d4, q4).subs(DH_TABLE)
	T4_5 = TF_MATRIX(alpha4, a4, d5, q5).subs(DH_TABLE)
	T5_6 = TF_MATRIX(alpha5, a5, d6, q6).subs(DH_TABLE)
	T6_EE = TF_MATRIX(alpha6, a6, d7, q7).subs(DH_TABLE)

	########################################################
	######## COMPOSITION OF HOMOGENEOUS TRANSFORMS #########
	########################################################
	T0_2 = T0_1 * T1_2 				# base_link to link-2
	T0_3 = T0_2 * T2_3 				# base_link to link-3
	T0_4 = T0_3 * T3_4 				# base_link to link-4
	T0_5 = T0_4 * T4_5 				# base_link to link-5
	T0_6 = T0_5 * T5_6 				# base_link to link-6
	
	########################################################
	################## ROTATION MATRICES ###################
	########################################################
	R0_3 = T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3]  #R3_6 = R3_4*R4_5*R5_6
	R3_6 = T3_4[0:3, 0:3]*T4_5[0:3, 0:3]*T5_6[0:3, 0:3]  #R3_6 = R3_4*R4_5*R5_6
	# print("R3_6 = ", R3_6)
	
	########################################################
	####### END-EFFECTOR TRANSFORM W.R.T. BASE-LINK ########
	########################################################
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
	#print("T0_EE = ", T0_EE.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

	# Inverse of R0_3 is R3_0 = transpose(R0_3)
	R3_0 = R0_3.transpose()

	return R3_0, T0_EE


########################################################################################
#############                        INVERSE KINEMATICS                    #############
#############            >> GIVEN END-EFFECTOR POSITION AND ORIENTATION    #############
#############                 >> FIND JOINT ANGLES OF ALL JOINTS           #############
########################################################################################
def inverse_kinematics(pos, ori, R3_0):
	
	'''
	Since the last three joints in our robot are revolute and their joint axes intersect 
	at a single point, we have a case of spherical wrist with joint_5 being the common 
	intersection point and hence the wrist center (WC). This allows us to kinematically 
	decouple the IK problem into Inverse Position and Inverse Orientation problems.

	First let us solve for the Inverse Position problem. Since we have the case of a 
	spherical wrist involving joints 4,5,6, the position of the wrist center is governed 
	by the first three joints. We can obtain the position of the wrist center by using
	the complete transformation matrix we derived based on the end-effector pose.
	'''

	########################################################
	###################### URDF TO DH ######################
	########################################################
	EE = Matrix(pos)
	ori = Matrix(ori)

	########################################################
	###### GRIPPER/EE COORDNIATE TRANSFORMATION FROM #######
	###### URDF COORDINATE FRAME TO DH COORDINATE FRAME ####
	########################################################
	# Gripper/End-Effector Rotation Matrices [URDF -> DH]
	r, p, y = symbols('r p y')
	rot_x, rot_y, rot_z = URDF2DH(r, p, y)

	########################################################
	###### BODY-FIXED ROTATION OF GRIPPER [UDRF >> DH] #####
	########################################################   
	URDF2DH_EE_ROT_CORRECTION = rot_z.subs(y, pi) * rot_y.subs(p, -pi/2)

	########################################################
	###### FOR ARBITRARY EE ORIENTATION FROM SIMULATION ####
	###### CONVERT GRIPPER/EE ORIENTATION [UDRF >> DH] #####
	########################################################
	ROT_EE = rot_z * rot_y * rot_x
	ROT_EE = ROT_EE * URDF2DH_EE_ROT_CORRECTION
	ROT_EE = ROT_EE.subs({'r': ori[0], 'p': ori[1], 'y': ori[2]})

	R0_6 = ROT_EE   

	EE2WC_TRANSLATION = Matrix([[0],
								[0],
								[DH_TABLE[d7]]])

	WC = EE - R0_6*EE2WC_TRANSLATION 			# Wrist Center w.r.t. Base Link

	print("WC = ", WC)

	########################################################
	############# CALCULATE DISTANCE AND ANGLES ############
	########################################################
	# Calculate wrist center on projected X0-Y0 plane (xc, yc) from base frame coordinates.
	# xc is the component in X0 direction minus link-offset from joint 2
	xc = sqrt(WC[0]**2 + WC[1]**2) - DH_TABLE[a1]  
	# yc is the component in Z0 direction minus link-length from joint 2
	yc = WC[2] - DH_TABLE[d1]

	# Calculate distances between joints considering joint-4 and joint-6 as rigid connection to joint-5
	d2_3 = DH_TABLE[a2]                               # distance between joint-2 to joint-3
	d3_5 = sqrt(DH_TABLE[a3]**2 + DH_TABLE[d4]**2)    # distance between joint-3 to joint-5/WC
	d2_5 = sqrt( xc**2 + yc**2 )                      # distance between joint-2 to joint-5/WC 

	alpha = atan2(yc, xc)
	beta = abs(atan2(DH_TABLE[a3], DH_TABLE[d4]))
	
	cos_a = cos_law(d2_5, d2_3, d3_5)
	cos_b = cos_law(d2_3, d3_5, d2_5)
	cos_c = cos_law(d3_5, d2_5, d2_3)

	angle_a = cos_inv(cos_a)
	angle_b = cos_inv(cos_b)
	angle_c = cos_inv(cos_c)

	########################################################
	######### CALCULATE THETA-1, THETA-2, THETA-3 ##########
	########################################################
	theta1 = atan2(WC[1], WC[0]).evalf()
	theta2 = ( pi/2 - (angle_a + alpha) ).evalf()
	theta3 =   ( pi/2 - (angle_b + beta) ).evalf()
	
	########################################################
	########### CALCULATE R3_6 ROTATION MATRIX #############
	########################################################	
	R3_0 = R3_0.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
	R3_6 = R3_0 * ROT_EE # Rotation matrix from joint-3 to gripper

	########################################################
	######### CALCULATE THETA-4, THETA-5, THETA-6 ##########
	########################################################
	theta5 = atan2( sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2] ).evalf() 
	if (theta5 > pi) :
		theta4 = atan2(-R3_6[2,2], R3_6[0,2]).evalf() 
		theta6 = atan2(R3_6[1,1], -R3_6[1,0]).evalf() 
	else:
		theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf() 
		theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf() 
	
	print("theta1: ", theta1)
	print("theta2: ", theta2)
	print("theta3: ", theta3)
	print("theta4: ", theta4)
	print("theta5: ", theta5)
	print("theta6: ", theta6)

	return [theta1, theta2, theta3, theta4, theta5, theta6]
	

def handle_calculate_IK(req):
	rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
	if len(req.poses) < 1:
		print "No valid poses received"
		return -1
	else:
		RMSE_EE_MAT = np.zeros((len(req.poses),3))
		# One-time forward kinematics operation
		R3_0, T0_EE = forward_kinematics()	
		# Initialize service response
		joint_trajectory_list = []

		for i in xrange(0, len(req.poses)):
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

			########################################################
			##########  EXTRACT END-EFFECTOR POSITION AND  #########
			########## ORIENTATION FROM SIMULATION REQUEST #########
			########################################################
			# end-effector position
			px = req.poses[i].position.x
			py = req.poses[i].position.y
			pz = req.poses[i].position.z
			# Orientation angles in quaternions
			oa = req.poses[i].orientation.x
			ob = req.poses[i].orientation.y
			oc = req.poses[i].orientation.z
			od = req.poses[i].orientation.w
			# Convert orientation angles from quaternions to Euler angles
			# roll, pitch, yaw = end-effector orientation
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([oa, ob, oc, od])
			pos = [px, py, pz]
			ori = [roll, pitch, yaw]

			# Populate response for the IK request
			joint_trajectory_point = JointTrajectoryPoint()            
			[theta1, theta2, theta3, theta4, theta5, theta6] = inverse_kinematics(pos, ori, R3_0)

			joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
			#joint_trajectory_point.positions = inverse_kinematics(pos, ori, R3_0)
			joint_trajectory_list.append(joint_trajectory_point)

			########################################################
			####     END-EFFECTOR POSITION ESTIMATION ERROR     ####
			########################################################
			T0_EE_EST = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
			EE_POS = np.array((px, py, pz), np.float64)
			EE_POS_EST = np.array((T0_EE_EST[0,3], T0_EE_EST[1,3], T0_EE_EST[2,3]), np.float64)
			RMSE_EE.add(EE_POS, EE_POS_EST)
			print ("END-EFFECTOR ERROR:", RMSE_EE.getError())
			# Save errors in a matrix for plotting
			RMSE_EE_MAT[i,:] = RMSE_EE.getError() 
			
		
		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

		# plot RMSE errors by (X, Y, Z) columns
		plt.plot(RMSE_EE_MAT) 
		plt.show()
		
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
	global start_time
	start_time = time()
	print ("\nStart time is %04.4f seconds" % start_time)

	# initialize node and declare calculate_ik service
	rospy.init_node('IK_server')
	s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
	print "Ready to receive an IK request"
	rospy.spin()

if __name__ == "__main__":
	IK_server()
