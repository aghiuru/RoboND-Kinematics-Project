from sympy import *
from sympy.solvers import solve
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.153,0.0,1.946],
                  [0,0,0,0]],
                 [1.85,0,1.94],
                 [0,0,0,0,0,0]],
              5:[]}


def r_x(a):
    return Matrix([[cos(a), -sin(a), 0],
                   [sin(a),  cos(a), 0],
                   [     0,       0, 1]])

#    return Matrix([[cos(a), -sin(a), 0, 0],
#                   [sin(a),  cos(a), 0, 0],
#                   [     0,       0, 1, 0],
#                   [     0,       0, 0, 1]])

def r_y(a):
    return Matrix([[ cos(a), 0, sin(a)],
                   [      0, 1,      0],
                   [-sin(a), 0, cos(a)]])

    # return Matrix([[ cos(a), 0, sin(a), 0],
    #                [      0, 1,      0, 0],
    #                [-sin(a), 0, cos(a), 0],
    #                [      0, 0,      0, 1]])


def r_z(a):
    return Matrix([[ 1,      0,       0],
                   [ 0, cos(a), -sin(a)],
                   [ 0, sin(a),  cos(a)]])

    # return Matrix([[ 1,      0,       0, 0],
    #                [ 0, cos(a), -sin(a), 0],
    #                [ 0, sin(a),  cos(a), 0],
    #                [ 0,      0,       0, 1]])


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##

    ## Insert IK code here!

    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[0].position.x
    py = req.poses[0].position.y
    pz = req.poses[0].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[0].orientation.x, req.poses[0].orientation.y,
            req.poses[0].orientation.z, req.poses[0].orientation.w])

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
                   [sin(q1) * cos(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                   [0, 0, 0, 1]])

    T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                   [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                   [sin(q2) * cos(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                   [0, 0, 0, 1]])

    T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                   [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                   [sin(q3) * cos(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                   [0, 0, 0, 1]])

    T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                   [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                   [sin(q4) * cos(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                   [0, 0, 0, 1]])

    T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                   [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                   [sin(q5) * cos(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                   [0, 0, 0, 1]])

    T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                   [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                   [sin(q6) * cos(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                   [0, 0, 0, 1]])

    T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
                   [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                   [sin(q7) * cos(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                   [0, 0, 0, 1]])

    R_total = r_x(yaw) * r_y(pitch) * r_z(roll)
    R_corr = r_y(+np.pi/2) * r_x(-np.pi)
    R_rpy = R_total * R_corr

    p = Matrix([[px], [py], [pz]])
    n = Matrix([[0], [0], [d7]])

    wc = simplify(p - R_rpy * n).subs(s)

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
    print('theta3: ', theta3)

    R0_3 = (T0_1 * T1_2 * T2_3)[:3, :3]
    R0_3t = R0_3.transpose()

    lhs = R3_6 = (T3_4 * T4_5 * T5_6)[:3, :3]

    rhs = simplify(R0_3t * R_rpy).subs(s).subs({q1: theta1,
                                                q2: theta2,
                                                q3: theta3})
    print('---------')
    print(lhs.subs(s))
    print(rhs)

    print(solve(lhs - rhs, [q4, q5, q6]))

    theta4 = 0
    theta5 = 0
    theta6 = 0

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = wc # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
