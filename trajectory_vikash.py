#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#ddd
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

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  # def __init__(self):

  #   # Initialize the node
  #   super(ExampleMoveItTrajectories, self).__init__()
  #   moveit_commander.roscpp_initialize(sys.argv)
  #   rospy.init_node('example_move_it_trajectories')

  #   try:
  #     self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
  #     if self.is_gripper_present:
  #       gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
  #       self.gripper_joint_name = gripper_joint_names[0]
  #     else:
  #       self.gripper_joint_name = ""
  #     self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

  #     # Create the MoveItInterface necessary objects
  #     arm_group_name = "arm"
  #     self.robot = moveit_commander.RobotCommander("robot_description")
  #     self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
  #     self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
  #     self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
  #                                                   moveit_msgs.msg.DisplayTrajectory,
  #                                                   queue_size=20)

  #     if self.is_gripper_present:
  #       gripper_group_name = "gripper"
  #       self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

  #     rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
  #   except Exception as e:
  #     print (e)
  #     self.is_init_success = False
  #   else:
  #     self.is_init_success = True
  def __init__(self, planning_time=10.0):  # Added planning_time parameter with default value of 10.0 seconds

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
      self.robot = moveit_commander.RobotCommander("kinova_arm/robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      
      # Set the planning time
      self.arm_group.set_planning_time(planning_time)  # Add this line to set the planning time
      
      # self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
      #                                               moveit_msgs.msg.DisplayTrajectory,
      #                                               queue_size=20)

      self.display_trajectory_publisher = rospy.Publisher('kinova_arm/' + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
        self.gripper_group.set_planning_time(planning_time)  # Add this line to set the planning time for gripper

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
  def get_new_pose_from_user(self):
        # Ask user for new coordinates
        rospy.loginfo("Please enter the new coordinates:")
        x = float(input("Enter X: "))
        y = float(input("Enter Y: "))
        z = float(input("Enter Z: "))
        
        # Create and return a new pose
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0.5
        pose.orientation.y = 0.5
        pose.orientation.z = 0.5
        pose.orientation.w = 0.5
        
        return pose
# def main():
#   example = ExampleMoveItTrajectories(planning_time=40.0)

#   # For testing purposes
#   success = example.is_init_success
#   try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#   except:
#       pass
  
#   if example.is_gripper_present and success:
#         rospy.loginfo("Opening the gripper...")
#         success &= example.reach_gripper_position(0)
#         print(success)
  
#   #if success:
#   #  rospy.loginfo("Reaching Named Target Home...")
#   #  success &= example.reach_named_position("home")
#   #  print (success)
#   if success:
#     rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     #actual_pose.position.z -= 0.2
#     #my lines of code
#     pose=geometry_msgs.msg.Pose()
#     pose.position.x=0.5-0.06
#     pose.position.y=-0.59
#     pose.position.z=0.7
#     pose.orientation.x=0.5
#     pose.orientation.y=0.5
#     pose.orientation.z=0.5
#     pose.orientation.w=0.5
#     success &= example.reach_cartesian_pose(pose, tolerance=0.01, constraints=None)
#     # pose.position.x += 0.06
#     success &= example.reach_cartesian_pose(pose, tolerance=0.01, constraints=None)
#     print(f"moving to position {pose.position.x},{pose.position.y},{pose.position.z}")
#     print (success)
    
# #   if example.degrees_of_freedom == 7 and success:
# #     rospy.loginfo("Reach Cartesian Pose with constraints...")
# #     # Get actual pose
# #     actual_pose = example.get_cartesian_pose()
# #     actual_pose.position.y -= 0.3
    
# #     # Orientation constraint (we want the end effector to stay the same orientation)
# #     constraints = moveit_msgs.msg.Constraints()
# #     orientation_constraint = moveit_msgs.msg.OrientationConstraint()
# #     orientation_constraint.orientation = actual_pose.orientation
# #     constraints.orientation_constraints.append(orientation_constraint)

# #     # Send the goal
# #     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

#   if example.is_gripper_present and success:
#     rospy.loginfo("Opening the gripper...")
#     success &= example.reach_gripper_position(0)
#     print (success)

#     rospy.loginfo("Closing the gripper 50%...")
#     success &= example.reach_gripper_position(0.5)
#     print (success)

#   # For testing purposes
#   rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#   if not success:
#       rospy.logerr("The example encountered an error.")

def main():
    example = ExampleMoveItTrajectories(planning_time=40.0)

    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print(success)

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")
        initial_pose = example.get_cartesian_pose()

        attempt = 0
        max_attempts = 2

        while attempt < max_attempts and not success:
            attempt += 1
            rospy.loginfo(f"Attempt {attempt}/{max_attempts}")

            # Plan the trajectory to the initial pose
            success = example.reach_cartesian_pose(initial_pose, tolerance=0.01, constraints=None)
            
            if not success:
                rospy.logerr("Failed to reach the position. Waiting for 10 seconds before retrying...")
                time.sleep(10)
                
                # Get new pose from the user
                initial_pose = example.get_new_pose_from_user()

        if not success:
            rospy.logerr("Unable to reach the desired position after multiple attempts. Exiting cleanly.")
            rospy.signal_shutdown("Failed to reach position")

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print(success)

        rospy.loginfo("Closing the gripper 50%...")
        success &= example.reach_gripper_position(0.5)
        print(success)

    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")


if __name__ == '__main__':
  main()









