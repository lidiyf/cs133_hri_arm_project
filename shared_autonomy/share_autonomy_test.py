#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState 
import kinpy as kp
import math
from pathlib import Path


class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

# cup1pos = [0, 0, 0]
# cup2pos = [0, 0, 0]
# cup3pos = [0, 0, 0]
# cup4pos = [0, 0, 0]
cup1Prob = 0
cup2Prob = 0
cup3Prob = 0
cup4Prob = 0
endTask = True
gripperDistanceThreshold = 100
prevEEpos = [0,0,0.9]
currEEpos = [0,0,0.9]
gripperDestination = [0, 0, 0]
meetGripperThreshold = False
noise = 0.07
gen3LiteChain = kp.build_chain_from_urdf(open("/home/mavis/hri_arm_final/src/grab_target/kinematics_urdf/gen3_lite_arm.urdf").read())
example = ExampleMoveItTrajectories()
success = False

def sharedAutonomy():
  rospy.loginfo("In shared autonomy")
  rospy.Subscriber("/my_gen3_lite/base_feedback/joint_state", JointState, callback)
  rospy.spin()

def callback(data):
  global prevEEpos
  global gripperDestination
  global success
  global currEEpos
  global meetGripperThreshold
  global noise
  # ret = 0
  # print(Path.cwd())
  transform = math.pi / 180.0
  th = {'J0': data.position[0], 'J1': data.position[1],
          'J2': data.position[2], 'J3': data.position[3],
          'J4': data.position[4], 'J5': data.position[5]}
  currEEpos = gen3LiteChain.forward_kinematics(th)["DUMMY"].pos
  # rospy.logerr("currEEpos")
  # rospy.logerr(currEEpos)
  # rospy.logerr("prevEEpos")
  # rospy.logerr(prevEEpos)
  xInput = currEEpos[0] - prevEEpos[0]
  yInput = currEEpos[1] - prevEEpos[1]
  zInput = currEEpos[2] - prevEEpos[2]

  if (not meetGripperThreshold) and \
    ((xInput > noise) or (xInput < -noise) or \
    (yInput > noise) or (yInput < -noise) or \
    (zInput > noise) or (zInput < -noise)): # haven't reach threshold position:
    # update probability
    for i in range(3):
      gripperDestination[i] = currEEpos[i] - 1.5 * (currEEpos[i] - prevEEpos[i])
    rospy.logerr("currEEpos")
    rospy.logerr(currEEpos)
    rospy.logerr("prevEEpos")
    rospy.logerr(prevEEpos)
    rospy.logerr("gripperDestination")
    rospy.logerr(gripperDestination)
    prevEEpos = currEEpos
    # rospy.loginfo("Reaching Cartesian Pose...")
    # actual_pose = example.get_cartesian_pose()

    pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header = actual_pose.header.seq
    # pose_goal.header.stamp = actual_pose.header.stamp
    # pose_goal.header.frame_id = ref_frame 
    # pose_goal.pose.orientation.w = 0.9
    # pose_goal.pose.orientation.x = 0.155
    # pose_goal.pose.orientation.y = 0.127
    # pose_goal.pose.orientation.z = 0.05
    pose_goal.pose.position.x = gripperDestination[0]
    pose_goal.pose.position.y = gripperDestination[1]
    pose_goal.pose.position.z = gripperDestination[2]
    # actual_pose = example.get_cartesian_pose()
    # actual_pose.position.z -= 0.2
    success &= example.reach_cartesian_pose(pose=pose_goal, tolerance=0.01, constraints=None)
    rospy.loginfo("*************************")
    rospy.loginfo(success)

    # if xInput > noise:
    #     update_probability(cup2Prob, xInput)
    #     update_probability(cup4Prob, xInput)
    # else if xInput < -noise:
    #     update_probability(cup1Prob, xInput)
    #     update_probability(cup3Prob, xInput)
    # if yInput > noise:
    #     update_probability(cup1Prob, yInput)
    #     update_probability(cup2Prob, yInput)
    # else if yInput < -noise:
    #     update_probability(cup3Prob, yInput)
    #     update_probability(cup4Prob, yInput)
    # if zInput < -noise:
    #   update_probability(max(cup1Prob, cup2Prob, cup3Prob, cup4Prob), zInput)
    # else if zInput > noise:
    #   update_probability(max(cup1Prob, cup2Prob, cup3Prob, cup4Prob), zInput)

    # # Move the arm
    # cupWithMaxProb = max(cup1Prob, cup2Prob, cup3Prob, cup4Prob)
    # if cupWithMaxProb == cup1Prob:
    #   gripperDestination += (cup1pos - gripperDestination) * 0.1
    # elif cupWithMaxProb == cup2Prob:
    #   gripperDestination += (cup2pos - gripperDestination) * 0.1
    # elif cupWithMaxProb == cup3Prob:
    #   gripperDestination += (cup3pos - gripperDestination) * 0.1
    # else cupWithMaxProb == cup4Prob:
    #   gripperDestination += (cup4pos - gripperDestination) * 0.1

    #### Call ros script to run the arm to gripperDestination


    # TBA: determine if threshold is meet
    # if (...):
    # 	meetGripperThreshold = True

  # else:
    # TBA: go and pick up the item

# def update_probability(cubProb, displacement): # update mechanism
# 	cubProb += displacement * 5

def main():
  global success
  # example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  if success:
    rospy.loginfo("Reaching Named Target Vertical...")
    success &= example.reach_named_position("vertical")
    print (success)

  
  # if success:
  #   rospy.loginfo("Reaching Named Target Home...")
  #   success &= example.reach_named_position("home")
  #   print (success) 

  rospy.loginfo("Begin shared autonomy")
  sharedAutonomy()

  # if success:
  #   rospy.loginfo("Reaching Cartesian Pose...")
  #   actual_pose = example.get_cartesian_pose()

  #   pose_goal = geometry_msgs.msg.PoseStamped()
  #   # pose_goal.header = actual_pose.header.seq
  #   # pose_goal.header.stamp = actual_pose.header.stamp
  #   # pose_goal.header.frame_id = ref_frame 
  #   pose_goal.pose.orientation.w = 0.9
  #   pose_goal.pose.orientation.x = 0.155
  #   pose_goal.pose.orientation.y = 0.127
  #   pose_goal.pose.orientation.z = 0.05
  #   pose_goal.pose.position.x = -0.1
  #   pose_goal.pose.position.y = -0.518
  #   pose_goal.pose.position.z = 1.655
    
  #   # actual_pose = example.get_cartesian_pose()
  #   # actual_pose.position.z -= 0.2
  #   success &= example.reach_cartesian_pose(pose=pose_goal, tolerance=0.01, constraints=None)
  #   print (success)
    
  # if example.degrees_of_freedom == 7 and success:
  #   rospy.loginfo("Reach Cartesian Pose with constraints...")
  #   # Get actual pose
  #   actual_pose = example.get_cartesian_pose()
  #   actual_pose.position.y -= 0.3
    
  #   # Orientation constraint (we want the end effector to stay the same orientation)
  #   constraints = moveit_msgs.msg.Constraints()
  #   orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  #   orientation_constraint.orientation = actual_pose.orientation
  #   constraints.orientation_constraints.append(orientation_constraint)

  #   # Send the goal
  #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  # if example.is_gripper_present and success:
  #   rospy.loginfo("Opening the gripper...")
  #   success &= example.reach_gripper_position(0)
  #   print (success)

  #   rospy.loginfo("Closing the gripper 50%...")
  #   success &= example.reach_gripper_position(0.5)
  #   print (success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
