[//]: # (Image References)
[image1]: ./misc_images/KUKA_KR210.jpg
[image2]: ./misc_images/KUKA_KR210_MESH1.jpg
[image3]: ./misc_images/KUKA_KR210_MESH2.jpg
[image4]: ./misc_images/Kuka_KR210_Example.png

[image5]: ./misc_images/Joint_Axes_rviz.png
[image6]: ./misc_images/DH_Coordinate_Frames_1.png
[image7]: ./misc_images/DH_Coordinate_Frames_2.png
[image8]: ./misc_images/DH_Coordinate_Frames_3.png
[image9]: ./misc_images/URDF_DH_Convention_Coordinates.png
[image10]: ./misc_images/URDF2DH_Gripper_Rotation.png
[image11]: ./misc_images/Gripper_URDF_Code.png

[image12]: ./misc_images/eq1.png
[image13]: ./misc_images/eq2.png
[image14]: ./misc_images/eq3.png
[image15]: ./misc_images/eq4.png
[image16]: ./misc_images/eq5.png
[image17]: ./misc_images/eq6.png
[image18]: ./misc_images/eq7.png
[image19]: ./misc_images/eq8.png
[image20]: ./misc_images/eq9.png
[image21]: ./misc_images/eq10.png
[image22]: ./misc_images/Intrinsic_xyz.png
[image23]: ./misc_images/Intrinsic_Rotations.png
[image24]: ./misc_images/Extrinsic_Rotations.png
[image25]: ./misc_images/Extrinsic_Intrinsic_Rotations.png
[image26]: ./misc_images/Homogeneous_Transform_1.png
[image27]: ./misc_images/Homogeneous_Transformation_Matrix_1.png
[image28]: ./misc_images/Homogeneous_Transformation_Matrix_2.png
[image29]: ./misc_images/Homogeneous_Transform_Rot_Trans_1.png
[image30]: ./misc_images/Homogeneous_Transform_Rot_Trans_2.png
[image31]: ./misc_images/Homogeneous_Transform_Rot_Trans_3.png

[image32]: ./misc_images/IK_Theta12.png
[image33]: ./misc_images/IK_Theta23.png

[image34]: ./misc_images/Displaying_Target_Location.png
[image35]: ./misc_images/Retrieving_Object.png
[image36]: ./misc_images/Reached_DropOff_Location.png
[image37]: ./misc_images/pick-and-place-fk.png
[image38]: ./misc_images/pick-and-place-pose.png
[image39]: ./misc_images/pick-and-place-tf.png

[image40]: ./output/RMSE_Error_Loc4_1.png
[image41]: ./output/RMSE_Error_Loc4_2.png
[image42]: ./output/RMSE_Error_Loc7_1.png
[image43]: ./output/RMSE_Error_Loc7_2.png
[video01]: ./output/Kuka_Robotic_Arm_Project_Video_4p4.mov

# Project: Kinematics Pick & Place

In this project, we will be writing code to perform Forward Kinematics (FK) and Inverse Kinematics (IK) of a pick and place robot arm manipulator.  We will be using a simulator environment to test both algorithms applied to a KUKA KR210 robotic arm.  In particular we will be using ROS, rviz and Gazebo for the implementation of our kinematics to the robotic serial manipulator.

In FK, we will be estimating the position of the gripper/end-effector given the angles of the 6 links in the Kuka KR210 manipulator.  In IK however, given a list of end-effector poses, we will be calculating the joint angles for the Kuka KR210.

Since the last three joints in our Kuka KR210 robot are revolute and their joint axes intersect at a single point, we have a case of spherical wrist with joint_5 being the common intersection point and hence the wrist center (WC). This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation problems.  First we will solve for the Inverse Position problem. Since we have the case of a spherical wrist involving joints 4, 5, 6, the position of the wrist center in the 3D space is governed by the first three joints (joints 1, 2, 3). We can obtain the position of the wrist center by using the complete transformation matrix we derived based on the end-effector pose.  The last three joints are designed for the precise positioning of the gripper to execute pick and place operations successfully. 

Our first goal is to setup our environment properly. Then explore the forward kinematics with Kuka KR210 to learn more about the robot's geometry and derive DH parameters. Once we have the DH parameters, we will run the complete pick and place project in demo mode to get an understanding of the complete project scenario.

Next we will perform kinematic analysis of the robot and derive equations for individual joint angles. In addition, we will write the actual Inverse Kinematics code inside of `IK_server.py` file. The general Inverse Kinematics content can be found [here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/3bc41e14-e43d-4105-887c-8268a7402750) or a quick recap [here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0). 

![alt text][image1]


**The goals / steps of this project are the following:**

* Setup ROS Workspace ([instructions here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/658c94f5-f806-4273-9001-9e2838e56856/concepts/a777bc7a-95d4-44ca-b4e3-119718e3a213)).
* Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the src directory of your ROS Workspace.
* Experiment with the forward_kinematics environment and get familiar with the robot ([details here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/7259f438-36a0-4bc1-ac53-39af669ba3c9))
* Launch the pick and place demo ([instructions here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193)) to get familiar with various steps in the project
* Perform forward and inverse kinematic analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view) along with a proper writeup. Use the `IK_debug.py` to perform initial analysis on the forward kinematics.
* Fill in the `IK_server.py` with Forward Kinematics (FK) and Inverse Kinematics (IK) code. Instructions on how to do the math behind IK can be found [here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0) and how to convert IK math into code can be found [here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/0ccca55a-22d3-43e5-95c1-b10afccba718)


**Forward Kinematics**

* Define DH parameter symbols and describe DH-table consisting of parameter values for Kuka KR210. 
* Define homogeneous transformation matrices consisting of rotation and translation elements for each joint of the Kuka KR210.
* Evaluate all transformation matrices and substitute DH parameter values from the DH-table
* Evaluate composition of homogeneous transformation matrices and extract rotation transformation (R0_3) from joint 0 to joint 3
* Evaluate end-effector position and orientation (T0_EE) with respect to base link.
* Evaluate inverse rotation transformation (R3_0) of the rotation matrix R0_3.

**Inverse Kinematics**

* Given the end-effector position and orientation in URDF coordinate, convert to DH coordinate frame.
* Evaluate theta_1, theta_2 and theta_3 using the position of the wrist-center, and DH parameter values. 
* Evaluate rotation matrix R3_0 using the computed joint angles 1 to 3.
* Evaluate R3_6 using R0_6 and R3_0.
* Evaluate theta_4, theta_5 and theta_6 using the elements of R3_6 matrix.

**Error in End-Effector Position Estimation**

* Evaluate estimated end-effector position using the joint angles computed in the Inverse Kinematics step. 
* Evaluate root-mean-squared-error (RMSE) of the estimated end-effector position w.r.t. the known end-effector position.
* Plot the error in end-effector pose generated by the joint angle commands from the inverse kinematics.

## 1. Software Installation, Dependencies, and Environment Setup
We will need Python 2 and ROS installation for this project. For this, we will use the [robo-nd Virtual Machine Image](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/2cd33882-d29e-43e8-9ff7-10398c8b5351/concepts/0cabcedf-a31b-4f41-b85a-3e0aedbdf5e8) or have Ubuntu+ROS installed locally.


### 1.1 ROS Installation Steps for Ubuntu
The detailed instructions on how to install ROS Kinetic and other dependencies can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
#### Setup your sources.list
Setup your computer to accept software from packages.ros.org.  
```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Set up your keys
```sh
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command.

#### Installation
First, make sure your Debian package index is up-to-date:
```sh
$ sudo apt-get update
```

There are many different libraries and tools in ROS. Some default configurations are shown below. 

Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
```sh
$ sudo apt-get install ros-kinetic-desktop-full
```

Desktop Install: ROS, rqt, rviz, and robot-generic libraries
```sh
$ sudo apt-get install ros-kinetic-desktop
```

ROS-Base: (Bare Bones) ROS package, build, and communication libraries. No GUI tools.
```sh
$ sudo apt-get install ros-kinetic-ros-base
```

Individual Package: 
To find available packages, use:
```sh
$ apt-cache search ros-kinetic
```

To install a specific ROS package (replace underscores with dashes of the package name):
sudo apt-get install ros-kinetic-PACKAGE
e.g.
```sh
$ sudo apt-get install ros-kinetic-slam-gmapping
```

#### Initialize rosdep
Before we can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.
```sh
$ sudo rosdep init
$ rosdep update
```

#### Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```sh
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.

If you just want to change the environment of your current shell, instead of the above you can type:
```sh
$ source /opt/ros/kinetic/setup.bash
```

If you use zsh instead of bash you need to run the following commands to set up your shell:
```sh
$ echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
$ source ~/.zshrc
```

#### Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```sh
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 1.2 One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```

To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```

### 1.3 Create ROS Workspace and Compile ROS Package

#### Setup ROS workspace and Update Dependencies
For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly. If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that we have a workspace, clone or download project repository into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now we will install missing dependencies using the almighty $ rosdep install command:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Next we will change the permissions of script files to turn them executable:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod u+x target_spawn.py
$ sudo chmod u+x IK_server.py
$ sudo chmod u+x safe_spawner.sh
```

Now that everything is in place, it is time to build the project. From within your workspace you can run catkin_make to build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Since the pick and place simulator spins up different nodes in separate terminals, you need to add the following to your .bashrc file for auto-sourcing:
```sh
$ source ~/catkin_ws/devel/setup.bash
```

As this project uses custom 3D models, we need to inform Gazebo (our simulation software) where to look for them. Thanks to environment variables, this can be easily accomplished by adding a single line to your .bashrc

Open a terminal window and type in the following:
```sh
$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models" >> ~/.bashrc
```

Alternately, open the .bashrc file in your favorite editor (I use Sublime Text)
```sh
$ subl ~/.bashrc
```

and add the following to your .bashrc file
```sh
$ export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models
$ source ~/catkin_ws/devel/setup.bash
```

#### Launch the Kuka KR210 Project Demo
For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, we will see the following in the gazebo world:
* Robot
* Shelf
* Blue cylindrical target in one of the shelves
* Dropbox right next to the robot

and the following in the rviz world:
* Robot
* Shelf
* Dropbox right next to the robot

![alt text][image5]

![alt text][image6]

Once all these items are confirmed, open rviz window, hit Next button. To view the complete demo keep hitting Next after previous action is completed successfully or hit Continue once to perform all actions in series. Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up. The demo ends when the robot arm reaches at the top of the drop location.  In case the demo fails, close all three terminal windows and rerun the script.

## 2 Forward Kinematic Analysis
### 2.1 Define DH Parameters
We will use the Denavit-Hartenberg (DH) notations to define our parameters for the link lengths (d1 to d7), link offsets (a0 to a6), twist angles (alpha0 to alpha6), and joint angles (q1 to q7). The KUka KR210 in its zero configuration is shown below.'

![alt text][image7]

Reference frames in URDF coordinate can be seen in the figure below

![alt text][image8]

The DH-table consisting of parameter values for Kuka KR210 are as shown below.

| Links | i   | alpha(i-1) | a(i-1) | d(i-1) | theta(i)  |
| :-----|:---:| ----------:| ------:| ------:| ---------:|
| 0->1  | 1   |      0     |      0 |   0.75 |         q1|
| 1->2  | 2   | - pi/2     |   0.35 |      0 | -pi/2 + q2|
| 2->3  | 3   |      0     |   1.25 |      0 |         q3|
| 3->4  | 4   | - pi/2     | -0.054 |    1.5 |         q4|
| 4->5  | 5   |   pi/2     |      0 |      0 |         q5|
| 5->6  | 6   | - pi/2     |      0 |      0 |         q6|
| 6->EE | 7   |      0     |      0 |  0.303 |          0|

```python
DH_TABLE = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:       q1, 
	    alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: -pi/2+q2,
	    alpha2:     0,  a2:   1.25,  d3:     0,  q3:       q3,
	    alpha3: -pi/2,  a3: -0.054,  d4:   1.5,  q4:       q4,
	    alpha4:  pi/2,  a4:      0,  d5:     0,  q5:       q5,
	    alpha5: -pi/2,  a5:      0,  d6:     0,  q6:       q6,
	    alpha6:     0,  a6:      0,  d7: 0.303,  q7:        0}
```

### 2.2 Define Homogeneous Transformation 
The homogeneous transformation matrix consisting of rotation and translation elements between two joints of the Kuka KR210 can be evaluated using the DH convention as shown below:

![alt text][image27]

Substituting the rotation and orientation matrices the omogeneous matrix is 

![alt text][image28]

For implementation, we define a function to evaluate the transformation matrices.
```python
def TF_MATRIX(alpha, a, d, q):
	TF_MAT = Matrix([[            cos(q),             -sin(q),            0,                a],
			 [ sin(q)*cos(alpha),	cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
			 [ sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),     cos(alpha)*d],
			 [                 0,                   0,            0,                1]])
	return TF_MAT
```

### 2.3 Evaluate Transformation Matrices
Evaluate the transformation matrices and their compositions by substituting the DH parameters into the transformation matrices as shown here.

#### Transformation matrix computation
Individual transformation matrices are evalunated by substituting the parameters defined in the DH_TABLE. 
```python
T0_1 = TF_MATRIX(alpha0, a0, d1, q1).subs(DH_TABLE)
T1_2 = TF_MATRIX(alpha1, a1, d2, q2).subs(DH_TABLE)
T2_3 = TF_MATRIX(alpha2, a2, d3, q3).subs(DH_TABLE)
...
```

#### Composition of the transformation matrices
The composition matrices are evaluated by multiplying the matrices in the desired order as shown here.
```python
T0_2 = T0_1 * T1_2 				# base_link to link-2
T0_3 = T0_2 * T2_3 				# base_link to link-3
...
```

#### Evaluate end-effector position and orientation (T0_EE) with respect to base link
The total composition matrix provides the rotation and orientation of the end-effector with respect to the base link.
```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### Extract rotation transformation (R0_3) from joint 0 to joint 3
The rotation of joint 3 with respect to the base link can be found from the composition of the rotation components of the transformation matrices as shown below.
```python
R0_3 = T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3]  #R3_6 = R3_4*R4_5*R5_6
```

#### Evaluate inverse rotation transformation (R3_0) of the rotation matrix R0_3
Since the rotation matrix symmetric, the inverse is equal to the transpose of the matrix.
```python
R3_0 = R0_3.transpose()
```
#### Examples of end-effector position and orientation computation
Various example of the end-effector position and and orientation computations shown below. 
Evaluate the end-effector position of the robot manipulator in its zero configuration
```python
print("T0_EE = ", T0_EE.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
```
Evaluate the end-effector position of the robot manipulator when the joint-3 is rotated by 45 degrees
```python
print("T0_EE = ", T0_EE.evalf(subs={q1: 0, q2: 0, q3: pi/4, q4: 0, q5: 0, q6: 0}))
```
Evaluate the end-effector position of the robot manipulator when the joint-2 is rotated by -30 degrees and joint-3 is rotated by 45 degrees
```python
print("T0_EE = ", T0_EE.evalf(subs={q1: 0, q2: -pi/6, q3: pi/4, q4: 0, q5: 0, q6: 0}))
```


## 3. Inverse Kinematic Analysis
When a task is assigned to the robot, it is given with the position of the target object and the needed orientation of the gripper when the grasping action is performed. 
```python
 #end-effector position
px = req.poses[i].position.x
py = req.poses[i].position.y
pz = req.poses[i].position.z
 #orientation angles in quaternions
oa = req.poses[i].orientation.x
ob = req.poses[i].orientation.y
oc = req.poses[i].orientation.z
od = req.poses[i].orientation.w
```

We receive the orientation angle in quaternion, hence we need to convert them into yaw-pitch-roll angle for the gripper. We used the euler_from_quaternion function to transform angles into euler representation.
```python
 #Convert orientation angles from quaternions to Euler angles
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([oa, ob, oc, od])
pos = [px, py, pz]
ori = [roll, pitch, yaw]
```

### 3.1 Transform from URDF to DH Coordinate frame
The coordinates in used in the simulation are URDF which we need to convert to DH wherever necessary for the Robot to orient itself in the right configurations.

![alt text][image11]

Given the end effector position, we need to convert end-effector position and orientation from URDF to DH coordinate system. 

![alt text][image10]

The function for this transformations as below:
```python
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
```
The gribber coordinate transformation from URDF coordinate frame to DH coordinate frame is then computed using the following
```python
r, p, y = symbols('r p y')
rot_x, rot_y, rot_z = URDF2DH(r, p, y)
```

The body-fixed rotation of the gripper used for transformation from URDF to DH coordinate is obtained by a rotation about the z-axis by 180 degrees followed by -90 degrees rotation about the y-axis.
```python
URDF2DH_EE_ROT_CORRECTION = rot_z.subs(y, pi) * rot_y.subs(p, -pi/2)
```
The intrinsic rotation from URDF to DH is 

![alt text][image22]

At arbitrary oritentation of the gripper, incorporating the intrinsic rotation transformation, the configuration of the end effector is calculated as below.
```python
EE = Matrix(pos)
ori = Matrix(ori)

ROT_EE = rot_z * rot_y * rot_x
ROT_EE = ROT_EE * URDF2DH_EE_ROT_CORRECTION
ROT_EE = ROT_EE.subs({'r': ori[0], 'p': ori[1], 'y': ori[2]})
```

Since the gripper link is rigidly connected to Link-6, we have q7=0 and R6_EE = I. Hence ROT_EE = R0_EE = R0_6*R6_EE = R0_6*I = R0_6. The end-effector is d7 distance from the wrist center (join-5). Hence, we compute the wrist center (WC) position w.r.t. the base link as shown below.

![alt text][image21]

The wrist center can be found using the following matrix multiplication and subtracting the result from the position of the end-effector.

![alt text][image30]

![alt text][image31]

```python
R0_6 = ROT_EE
EE2WC_TRANSLATION = Matrix([[0],
			    [0],
			    [DH_TABLE[d7]]])
 # WC = EE - DH_TABLE[d7]*URDF2DH_ROT_EE[:,2]
WC = EE - R0_6*EE2WC_TRANSLATION 	# Wrist Center w.r.t. Base Link
```


### 3.2 Evaluate theta_1, theta_2 and theta_3
Since we have a decoupled system, using the position of the wrist-center, and DH parameter values, we will first evaluate the theta-1, theta2, and theta-3 values. 

The angle theta-1 is the angle in the x-y plane, made by the projection of the wrist center (WC) onto x-y plane. 

![alt text][image32]

Now we calculate wrist center on projected X0-Y0 plane (xc, yc) from joint-2. Here, xc is the component in X0 direction minus link-offset from joint 2 (a1) and yc is the component in Z0 direction minus link-length from joint 2

```python
xc = sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - DH_TABLE[a1]
yc = WC[2] - DH_TABLE[d1]
```

In joint-2's zero configuration, the angle made by the wrist center about joint-2's z-axis is computed as below:
```python
alpha = atan2(yc, xc)
``` 
The angle between joint-2 and joint-5 due to the geometry of the link-design is:
```python
beta = abs(atan2(DH_TABLE[a3], DH_TABLE[d4]))	
```
For angle theta-2 and theta3, we will use the cosine law for the triangle shown below.

![alt text][image33]

The sides of the above triagle are the distances between joints 2, 3, and 5 considering joint-4 and joint-6 as rigid connection to joint-5. 
```python
d2_3 = DH_TABLE[a2]                               			# distance between joint-2 to joint-3
d3_5 = sqrt(DH_TABLE[a3]*DH_TABLE[a3] + DH_TABLE[d4]*DH_TABLE[d4])    	# distance between joint-3 to joint-5/WC
d2_5 = sqrt( xc*xc + yc*yc )                     			# distance between joint-2 to joint-5/WC 
```
Given the lengths of the sides of the triangle, the angles can be computed using the cosine law. The angle between sides a and b can be found using the formula, cos(theta) = (a^2 + b^2 - c^2) / 2a*b.
```python
cos_law = lambda a, b, c: (a*a + b*b - c*c) / (2 * a * b) 

cos_a = cos_law(d2_5, d2_3, d3_5)
cos_b = cos_law(d2_3, d3_5, d2_5)
cos_c = cos_law(d3_5, d2_5, d2_3)

cos_inv = lambda X: atan2(sqrt(1 - X*X), X)

angle_a = cos_inv(cos_a)
angle_b = cos_inv(cos_b)
angle_c = cos_inv(cos_c)
```

Using the above computation we can then find the angles theta-1, theta-2, and theta-3 as shown below:
```python
theta1 = atan2(WC[1], WC[0]).evalf()
theta2 = ( pi/2 - (angle_a + alpha) ).evalf()
theta3 = ( pi/2 - (angle_b + beta) ).evalf()	
```

### 3.3 Evaluate rotation matrices R3_0 and R3_6
Using the joint angles computed above, we can now compute the orientation of the joint-0 with respect to joint-3
```python
R3_0 = R3_0.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```
Now the rotation matrix from joint-3 to the gripper/end-effector is
```python
R3_6 = R3_0 * ROT_EE
```

Also using the homogeneous transformation, we know the matrix elements of R3_6 as below, when sin(theta5) is not zero:
```python
R3_6 = 
[[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - cos(q4)*cos(q5)*sin(q6), -cos(q4)*sin(q5)],
 [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
 [-sin(q4)*cos(q5)*cos(q6) - cos(q4)*sin(q6),  sin(q4)*cos(q5)*sin(q6) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]
```
and when sin(theta5) = 0
```python
R3_6 = 		# [theta5 = 0 or theta5 = 2*pi]
[[ cos(q4)*cos(q6), -cos(q4)*sin(q6),   0],
 [               0,                0,   1],
 [-sin(q4)*cos(q6),  sin(q4)*sin(q6),   0]]

R3_6 = 		# [theta5 = pi]
[[-cos(q4)*cos(q6),  cos(q4)*sin(q6),   0],
 [               0,                0,  -1],
 [ sin(q4)*cos(q6), -sin(q4)*sin(q6),   0]]
```

### 3.4 Evaluate theta_4, theta_5 and theta_6
Using the elements of R3_6 matrix we will evaluate theta-4, theta-5, and theta-6 as shown below:
```python
theta5 = atan2( sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2] ).evalf() 
if (theta5 > pi) :
	theta4 = atan2(-R3_6[2,2], R3_6[0,2]).evalf() 
	theta6 = atan2(R3_6[1,1], -R3_6[1,0]).evalf() 
else:
	theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf() 
	theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf() 
```


## 4. Results
Some screen shots of the pick and place operations are shown below.  The target (the blue cylinder) was on the 7th slot of the self (i.e. `spawn_location = 7` in the `target_description.launch` file). 

The Kuka KR210 is retrieving the object from the seventh slot of the shelf as shown below.

![alt text][image35]

The Robotic arm is dropping off the object the desired bin. Also displayed is the path the robot planned and executed for both pick-up and drop-off operations.

![alt text][image36]

* Overall I really enjoyed this project since it touches on multiple areas (ROS, Gazebo, rviz, sympy). I particularly liked learning the sympy applications, even though it makes the computation slower than using numpy. Using sympy makes he code look cleaner than using numbers.

* I learned some matix operations in symbolic form and value substitution.  This makes the code reusable for other applications. 

### 4.1 Optimization of Grasping Parameters
After poking around for quite some time, I found some parameters to modify in the source code .cpp file.  I modified some parameters inside the `trajectory_sampler.cpp` to let the gripper close firmly by changing the gripper_joint_positions[0] to 0.035 and gripper_joint_positions[1] to 0.015.  Also, I allowed longer time to grip by increasing the Duration of grasping from 1.5 seconds to 2.5 seconds. This helped the gripper grasp more firmly and did not let the sample slip-off the fingers while moving the sample from the shelf to the dropbox. 

```cpp
bool TrajectorySampler::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.035;  // radians
    gripper_joint_positions[1] = 0.015;  // radians
  }
  else
  {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(2.5).sleep();

  bool success = eef_group.move();
  return success;
}

```

### 4.2 Error Analysis

**Errors in End-Effector Position Estimation**
I have performed an error analysis to compute the root-mean-squared-error (RMSE) for the gripper/end-effector position estimation from the inverse kinematics as discussed here.

##### Evaluate estimated end-effector position using the joint angles computed in the Inverse Kinematics step. 
```python
EE_POS = np.array((px, py, pz), np.float64)
T0_EE_EST = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
EE_POS_EST = np.array((T0_EE_EST[0,3], T0_EE_EST[1,3], T0_EE_EST[2,3]), np.float64)

```
##### Evaluate root-mean-squared-error (RMSE) of the estimated end-effector position w.r.t. the known end-effector position. The class for the RMSE computations is shown here:
```python
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
```

Create a global instantiation of the the RMSE class as below:
```python
RMSE_EE = RMSE()	
```
and add data points to the class and evaluate the error as shown below:
```python
RMSE_EE.add(EE_POS, EE_POS_EST)
print ("END-EFFECTOR ERROR:", RMSE_EE.getError())
 #Save errors in a matrix for plotting
RMSE_EE_MAT[i,:] = RMSE_EE.getError() 
```

##### Plot the error in end-effector pose generated by the joint angle commands from the inverse kinematics.
```python
plt.plot(RMSE_EE_MAT) # plot RMSE errors by (X, Y, Z) columns
plt.show()
```
The RMSE errors were minimal in the order of 10^(-16), which indicates that the gripper always reaches the desired target location. Error plots for one of the pick and place operation when the blue cylinder is in the `spawn location = 4` is shown below.  The first plot is for reaching the target location to pick the blue cylinder.  The second plot shows the error between desired drop-off location and the estimated drop-off location computed using the joint angles calculated from the inverse kinematics. 

![alt text][image40]
![alt text][image41]

Another example of the inverse kinematics error is below for `spawn location = 7`

![alt text][image42]
![alt text][image43]

### 4.3 Output Video
An output video created by screen recording can be found in the output folder.
![alt text][video01]

## 5. Issues
* Most of the times the picking of the blue cylinder works well.  However, I have noticed in several runs that even though the gripper is in the right pose and orientation while grabbing the blue cylinder, the cylinder slips off the gripper either during the pick-up operation or during the operation. It would have been better if the grasping part was also covered in the classroom.  It took me quite some time to figure out how to modify the grasping parameters as discueed in the Results section above. 
* In more than 50% of the time, Gazebo either crashes or does not open at all.  So it was time-consuming project. I used Gazebo 7.8.1 for all the simulations.

## 6. Possible Optimizations
* The arms move around quite bit to put quite a performance for some operations that are farther from the trash-bin, i.e. for spawn loacations 3, 6, and 9.  It would have been great if some optimization was covered during the lessons. Hopefully there are more lessons in the future that covers some optimization aspects for pick and place type operations based on the distance from the object.
* More optimization of the code neded for the wrist to stop unnecessary rotations of above pi or 2*pi.
* Completely removing sympy and replacing all symbols with precalculated transformation matrices would improve the computational time. 

