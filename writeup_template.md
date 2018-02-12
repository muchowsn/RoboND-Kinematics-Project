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

[image1]: ./misc_images/Running_demo.png
[image2]: ./misc_images/Joints.jpg
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.
`roslaunch kuka_arm forward_kinematics.launch`:

![alt text][image1]<br/>
The DH parameters were derived from the arm according to these axis assignments:  <br/>
![alt text][image2]<br/>

| joint | x | y | z | roll | pitch | yaw |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
joint<sub>1</sub> | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint<sub>2</sub> | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint<sub>3</sub> | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint<sub>4</sub> | 0.96 | 0 | -0.54 | 0 | 0 | 0 |
joint<sub>5</sub> | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint<sub>6</sub> | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper | 0.11 | 0 | 0 | 0 | 0 | 0 |  


##### Table 2: Modified DH Parameters
α<sub>i-1</sub>: twist angle, angle between Ζ<sub>i-1</sub> and Ζ measured about Χ<sub>i-1</sub>  
ɑ<sub>i-1</sub>: link length, distance from Ζ<sub>i-1</sub> and Ζ measured about Χ<sub>i-1</sub>  
ɗ<sub>i</sub>: link offset, signed distance from Χ<sub>i-1</sub> to Χ<sub>i</sub> measured along Ζ<sub>i</sub>  
θ<sub>i</sub>: joing angle, angle between Χ<sub>i-1</sub> to Χ<sub>i</sub>

| joint | α<sub>i-1</sub> | ɑ<sub>i-1</sub> |ɗ<sub>i</sub> | θ<sub>i</sub> |
| :---: | :---: | :---: | :---: | :---: |
joint<sub>1</sub> | 0 | 0 | 0.75 | q1 |
joint<sub>2</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | 0.35 | 0 | q2: q2 - <sup>π</sup>&frasl;<sub>2</sub> |
joint<sub>3</sub> | 0 | 1.25 | 0 | q3 |
joint<sub>4</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | -0.054 | 1.5 | q4 |
joint<sub>5</sub> | <sup>π</sup>&frasl;<sub>2</sub> | 0 | 0 | q5 |
joint<sub>6</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | 0 | 0 | q7 |
gripper | 0 | 0 | 0.303 | q7: 0 |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### DH Parameter Table
```python
s = {alpha0:       0, a0:      0, d1:  0.75,
     alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
     alpha2:       0, a2:   1.25, d3:     0,
     alpha3: -pi / 2, a3: -0.054, d4:   1.5,
     alpha4:  pi / 2, a4:      0, d5:     0,
     alpha5: -pi / 2, a5:      0, d6:     0,
     alpha6:       0, a6:      0, d7: 0.303, q7: 0}
```  

##### Individual Transformation Matrices About Each Joint
```python
# base_link to link_1
T0_1 = Matrix([[              cos(q1), -sin(q1)             ,            0,                a0],
               [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0) ,  cos(alpha0) * d1],
               [                    0,                     0,            0,                 1]])

# link_1 to link_2
T1_2 = Matrix([[              cos(q2),              -sin(q2),            0,                a1],
               [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1),  cos(alpha1),  cos(alpha1) * d2],
               [                    0,                     0,            0,                 1]])

# link_2 to link_3
T2_3 = Matrix([[              cos(q3),              -sin(q3),            0,                a2],
               [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2),  cos(alpha2),  cos(alpha2) * d3],
               [                    0,                     0,            0,                 1]])               

# link_3 to link_4
T3_4 = Matrix([[             cos(q4) ,              -sin(q4),            0,                a3],
               [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
               [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3),  cos(alpha3),  cos(alpha3) * d4],
               [                    0,                     0,            0,                 1]])

# link_4 to link_5
T4_5 = Matrix([[              cos(q5),              -sin(q5),            0,                a4],
               [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
               [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4),  cos(alpha4),  cos(alpha4) * d5],
               [                    0,                     0,           0,                  1]])

# link_5 to link_6
T5_6 = Matrix([[              cos(q6),              -sin(q6),            0,                a5],
               [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
               [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5),  cos(alpha5),  cos(alpha5) * d6],
               [                    0,                     0,            0,                 1]])

# link_6 to gripper
T6_EE = Matrix([[              cos(q7),              -sin(q7),            0,                a6],
               [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
               [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6),  cos(alpha6),  cos(alpha6) * d7],
               [                    0,                     0,            0,                 1]])
```

##### Generalized homogeneous transform between base_link and gripper using only the gripper pose.
```python
T0_EE= T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image3]


see below for derivations of theta angles.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


* Find theta 1 by:<br/>
```python
theta1 = atan2(WC[1], WC[0]) # where WC[1] and WC[0] are the wrist center y and x coordinates, respectively.
```

* For Theta 2 and 3 I use SSS to find angles of the triangle.<br/>
```python
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35),2) + pow((WC[2] -0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for the dag in link4 of -0.054m
```

* For theta4, theta5, theta6 I found the rotational matrices.<br/>
```python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = Transpose(R0_3) * ROT_EE
```

* Find infividual Rotational Matrix Element. <br/>
```python
r12, r13 = R3_6[0, 1], R3_6[0, 2]
r21, r22, r23 = R3_6[1, 0], R3_6[1, 1], R3_6[1, 2]
r32, r33 = R3_6[2, 1], R3_6[2, 2]
```

* Then calculate theta4, theta5, and theta6. First calculate Theta5 to see if its larger than 0, this is to check which position the arm is in. then calculate theta4 and theta theta5 is above or below 0. by using atan2 you get already take into account the angle limits.
```python
theta5 = atan2(sqrt(r13**2 + r33**2), r23)
             
if sin(theta5) < 0:
     theta4 = atan2(-r33, r13)
     theta6 = atan2(r22, -r21)
else:
     theta4 = atan2(r33, -r13)
     theta6 = atan2(-r22, r21)            
```







