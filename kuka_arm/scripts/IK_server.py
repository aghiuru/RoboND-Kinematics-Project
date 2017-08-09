#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            # Joint angle symbols
            # Modified DH params
            # Define Modified DH Transformation matrix
            # Create individual transformation matrices
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

            s = {
                alpha0: 0, a0: 0, d1: .75,
                alpha1: -pi/2, a1: .35, d2: 0, q2: q2 - pi/2,
                alpha2: 0, a2: 1.25, d3: 0,
                alpha3: -pi/2, a3: -.054, d4: 1.5,
                alpha4:  pi/2, a4: 0, d5: 0,
                alpha5: -pi/2, a5: 0, d6: 0,
                alpha6: 0, a6: 0, d7: .303, q7: 0
            }

            T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                           [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                           [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                           [0, 0, 0, 1]])

            T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                           [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                           [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                           [0, 0, 0, 1]])

            T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                           [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                           [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                           [0, 0, 0, 1]])

            T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                           [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                           [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                           [0, 0, 0, 1]])

            T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                           [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                           [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                           [0, 0, 0, 1]])

            T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                           [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                           [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                           [0, 0, 0, 1]])

            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            R_total = r_x(yaw) * r_y(pitch) * r_z(roll)
            R_corr = r_y(+np.pi/2) * r_x(-np.pi)

            p = Matrix([[px], [py], [pz]])
            n = Matrix([[0], [0], [d7]])

            wc = simplify(p - R_total * R_corr * n).subs(s)

            p_wc = (float(wc[0]), float(wc[1]), float(wc[2]))
            print('WC: {}'.format(wc))

            theta1 = simplify(atan2(wc[1], wc[0])).subs(s)
            print('theta1: ', theta1)

            # Find O2
            o2 = (cos(theta1) * a1, sin(theta1) * a1, d1)
            print('O2: ', o2)

            d_xy = sqrt((wc[0] - o2[0])**2 + (wc[1] - o2[1])**2)
            print('d_xy: ', d_xy)

            theta_21 = atan2(wc[2] - o2[2], d_xy)
            print('theta_21: ', theta_21)

            c = sqrt((wc[2] - o2[2])**2 + d_xy**2)
            print('c: ', c)

            b = sqrt(a3**2 + d4**2)

            theta_22 = acos((c**2 + a2**2 - b**2) / (2 * c * a2))
            print('theta_22: ', theta_22)

            # Find O3
            o3z = o2[2] + sin(theta_21 + theta_22) * a2
            print('o3z: ', o3z)

            theta2 = (pi / 2 - theta_21 - theta_22).subs(s)
            print('theta2: ', theta2)

            theta31 = atan2(a3, d4)
            theta32 = acos((b**2 + a2**2 - c**2) / (2 * a2 * b))
            print('theta31: ', theta31)
            print('theta32: ', theta32)

            theta3 = (pi / 2 + theta31 - theta32).subs(s)

            R0_3 = simplify(T0_1.subs(s) * T1_2.subs(s) * T2_3.subs(s))[:3, :3]
            R0_3t = R0_3.transpose().subs({q1: theta1,
                                           q2: theta2,
                                           q3: theta3})

            print('---------')

            lhs = (T3_4 * T4_5 * T5_6).subs(s)[:3, :3]
            pprint(simplify(lhs))

            rhs = R0_3t * R_total * R_corr
            pprint(rhs)

            # alpha, beta, gamma = tf.transformations.euler_from_matrix(np.array(rhs).astype(np.float64), "ryzy")
            # print(yaw, pitch, roll)
            # print(alpha, beta, gamma)

            r11 = rhs[0,0]
            r12 = rhs[0,1]
            r13 = rhs[0,2]
            r21 = rhs[1,0]
            r22 = rhs[1,1]
            r23 = rhs[1,2]
            r31 = rhs[2,0]
            r32 = rhs[2,1]
            r33 = rhs[2,2]

            theta4 = atan2(r33, -r13)
            print(theta4)

            theta5 = atan2(sqrt(r21**2 * r22**2), r23)
            print(theta5)

            theta6 = atan2(-r22, r21)
            print(theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
