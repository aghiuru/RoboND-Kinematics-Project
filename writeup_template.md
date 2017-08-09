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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


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

#### ARM FROM ABOVE

_q~1~ = atan(WC~y~, WC~x~_)

###### q2, q3
For the second and the third angles, there were also multiple solutions -- I could choose between two positions for the third joint: above the WC or below the WC. I decided to go with the above one, because I was worried that the arm might hit the floor when reaching for objects on the lowest shelf.

The figure I used for the determining the angles:

#### ARMZZsz

_q~2~_ becomes then a difference of angles:
_q~2~ = pi/2 - q~21~ - q~22~_
and
_q~21~ = atan(e, c)_
_q~22~ = acos(c^2^ + a~2~^2^ - b^2^ / 2ca~2~)_ (law of cosines)

_q~3~_ is a bit trickier because of the orientation, but, as represented in the figure:
_q~3~ = pi/2 + q~31~ - q~32~_
_q~31~ = atan(a~3~, d~4~)_ (small drop from link 3 to link 4)
_q~32~ = acos(b^2^ + a~2~^2^ - c^2^ / 2ba~2~)_ (again, law of cosines)

I also computed _O~2~_ and _O~3~_ to make sure

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.


And just for fun, another example image:
![alt text][image3]
