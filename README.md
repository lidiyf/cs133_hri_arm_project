# cs133_hri_arm_project
Authors: Kelly MacDonald, Lidi Yafei, Mavis Murlock

This project uses Kinova Gen3 Lite robot arm, to implement robot manipulation and shared autonomy.

-----
### Launching the program
use the command `roslaunch grab_target moveit_example.launch`

### Moving/Manipulating the arm
`grab_target/scripts/plan_and_move`:

  (note that `example` is a ExampleMoveItTrajectories object )

  To open the gripper: `example.reach_gripper_position(1)`
  To close the gripper 50%: `example.reach_gripper_position(0.5)`

  To reach a given pose: `example.reach_cartesian_pose(pose=pose_goal, tolerance=0.01, constraints=None)`
  (where pose_goal is a PoseStamped())

  To get the current pose: `example.get_cartesian_pose`
