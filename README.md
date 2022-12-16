# cs133_hri_arm_project
Authors: Kelly MacDonald, Lidi Yafei, Mavis Murlock

This project uses Kinova Gen3 Lite robot arm, to implement robot manipulation and shared autonomy.

-----
### Launching the program
use the command `roslaunch grab_target moveit_example.launch`

## Moving/Manipulating the arm
The Kinova arm web app is available at: `192.168.1.10/monitoring` 

`grab_target/scripts/plan_and_move`:

  (note that `example` is a ExampleMoveItTrajectories object )

  To open the gripper: `example.reach_gripper_position(1)`
  To close the gripper 50%: `example.reach_gripper_position(0.5)`

  To reach a given pose: `example.reach_cartesian_pose(pose=pose_goal, tolerance=0.01, constraints=None)`
  (where pose_goal is a PoseStamped())

  To get the current pose: `example.get_cartesian_pose`
  
### Detecting Arm Position
`get_xyz.py` uses forward kinematics to calculate the position of the end effector, using the data from `gen3_lite_gripper.urdf`.
  
 ### Camera
 Requires the realsense2_camera [package](https://github.com/IntelRealSense/realsense-ros#installation-instructions). For debugging, see the [documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages).
 To view the color image from the camera, use the topic `/camera/color/image_raw`
 To view the depth image from the camera, use the topic `/camera/depth/image_rect_raw`
 
 ## Shared Autonomy
 ### Goal
 Given a set of objects arranged in a semicircle, we have a user control the robotic arm using an xbox controller. We implement shared autonomy to predict which object they have selected.
 
 ### Controller Input
 An xbox controller is connected to the computer. We read user input by using the [Joy](http://library.isr.ist.utl.pt/docs/roswiki/joy(2f)Tutorials(2f)ConfiguringALinuxJoystick.html) library. 
 
 ### Algorithm
 Our code calculates the probability of a particular target having been chosen given the input from the user and the position of the end effector in relation to each cup. 
