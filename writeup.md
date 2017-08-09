## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/q1.jpg
[image2]: ./misc_images/q2q3.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

This proved very helpful for calculating the first three angles, I could move the arm by different angles and verify the position for the second and third joints.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | .75 | qi
1->2 | -pi/2 | .35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 | -pi/2 | -.054 | 1.5 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | .303 | 0

```
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

T6_EE = Matrix([[cos(q7), -sin(q7), 0, a6],
                [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                [0, 0, 0, 1]])
```

Of course, the total homogeneous transform between base and gripper is the product of the matrices above, in the order of the joints:

_T~0_EE~ = (T~0_1~ * T~1_2~ * T~2_3~ * T~3_4~ * T~4_5~ * T~5_6~ * T~6_EE~)_

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


Before solving the inverse problems, I had to find out the position of the wrist center (WC).

Given the fact that we're given both the position and orientation of the end effector (EE), and the fact that WC is located _d~7~_ units back down the Z axis, I had to determine the projection of the EE along the axes of the WC frame to know exactly how much I needed to go back.

WC = EE - R~EE~ * [0 0 _d~7~_]^T^

We're given R~rpy~, but since it's in URDF orientation, it has to be corrected by applying two additional rotations, so:
R~corr~ = R~y~(pi/2) * R~x~(-pi)
R~EE~ = R~rpy~ * R~corr~

After determining R~EE~, I got the WC location.

##### Inverse position
###### q1
I want to have the X axis of the arm face the WC, so it can reach further (since link 2 is _d~1~_ units away on the X axis).

Having decided that, determining the first angle (_q~1~_) becomes a very easy task if you look at the arm from above:

![View from above][image1]

_q~1~ = atan(WC~y~, WC~x~_)

###### q2, q3
For the second and the third angles, there were also multiple solutions -- I could choose between two positions for the third joint: above the WC or below the WC. I decided to go with the above one, because I was worried that the arm might hit the floor when reaching for objects on the lowest shelf.

The figure I used for the determining the angles:

![Figure for q2, q3][image2]

_q~2~_ becomes then a difference of angles:
_q~2~ = pi/2 - q~21~ - q~22~_
and
_q~21~ = atan(e, f)_
_q~22~ = acos(c^2^ + a~2~^2^ - b^2^ / 2ca~2~)_ (law of cosines)

_q~3~_ is a bit trickier because of the orientation, but, as represented in the figure:
_q~3~ = pi/2 + q~31~ - q~32~_
_q~31~ = atan(a~3~, d~4~)_ (small drop from link 3 to link 4)
_q~32~ = acos(b^2^ + a~2~^2^ - c^2^ / 2ba~2~)_ (again, law of cosines)

I also computed _O~2~_ and _O~3~_ and used forward kinematics to make sure the angles are right.

###### q4, q5, q6
Given that
_R~3_6~ = R~0_3~^T^ * R~EE~_

I simplified _R~3_6~_ in sympy to see what it's comprised of:
```
Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

_R~EE~_ had been calculated before.

For _R~0_3~^T^_, I had to multiply the rotations for the first three joints and compute the inverse (same as the transpose since the matrix is orthogonal):
_R~0_3~^T^ = (R~0_1~ * R~1_2~ * R~2_3~)^-1^_

For determining the angles I used:
_q~4~ = atan(sin(q~4~)*sin(q~5~), -(-sin(q~4~)*sin(q~6~))_ --- terms 33 and 13, reducing _sin(q~4~)_

_q~5~ = atan(sqrt((sin(q~5~)*cos(q~6~))^2^ * (-sin(q~5~)*sin(q~6~))^2^), cos(q~5~))_ --- terms 21, 22, 23, _q~6~_ terms reduce and I was left with only _q~5~_ terms)

_q~6~ = atan(-(-sin(q~5~)*sin(q~6~)), sin(q~5~)*cos(q~6~))_ --- terms 22 and 21, reducing _sin(q~5~)_

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The coding was pretty straightforward really.

After filling out DH params and DH transforms, then I went on to calculate the values for the angles as per the formulas presented at 3.

I used symbols for all vars, for the first three angles I substituted at the very end, thinking that it might reduce errors. For the last three ones, I substituted as soon as possible to make inverse orientation run faster.

The only position I haven't tried is the bottom right one -- even after I set position to 9 in the `target_description.launch` file -- all the other ones are resolved correctly.
