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
# Modified by Alexandre Vannobel and [Your Name] for image capture integration

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

# Additional imports for image capture functionality:
import subprocess
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os

# Global variables for camera handling:
bridge = CvBridge()
camera_subscriber = None  # This will hold the subscriber handle so we can later unsubscribe

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
            print(e)
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
        for p in joint_positions:
            rospy.loginfo(p)

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
        for p in new_joint_positions:
            rospy.loginfo(p)
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

# Helper function to launch the camera node using roslaunch
def launch_camera():
    """
    Launch the kinova_vision node by running the kinova_vision.launch file.
    Returns the subprocess handle so that the process can be later killed.
    """
    try:
        proc = subprocess.Popen(["roslaunch", "kinova_vision", "kinova_vision.launch"])
        rospy.loginfo("Camera node launched successfully.")
        return proc
    except Exception as e:
        rospy.logerr("Failed to launch camera node: " + str(e))
        return None

# Callback function for image capture and saving
def image_callback(msg):
    global camera_subscriber  # so we can unregister after capturing
    try:
        # Convert the ROS image message to an OpenCV image (BGR format)
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + str(e))
        return

    # Ensure the directory exists
    save_dir = "/home/pc/pic"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Define the filename; here we use a fixed name or you can use a timestamp
    image_path = os.path.join(save_dir, "captured_image.jpg")
    
    # Save the image using OpenCV
    cv2.imwrite(image_path, cv_image)
    rospy.loginfo("Image saved to " + image_path)

    # Unsubscribe so that we only capture one image
    if camera_subscriber:
        camera_subscriber.unregister()
        rospy.loginfo("Camera subscriber unsubscribed after capturing the image.")

def main():
    example = ExampleMoveItTrajectories()

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Reaching Named Target Vertical...")
        success &= example.reach_named_position("vertical")
        print(success)
    
    if success:
        rospy.loginfo("Reaching Joint Angles...")  
        success &= example.reach_joint_angles(tolerance=0.01)  # rad
        print(success)
    
    if success:
        rospy.loginfo("Reaching Named Target Home...")
        success &= example.reach_named_position("home")
        print(success)

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.z -= 0.2
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
        print(success)
        
    if example.degrees_of_freedom == 7 and success:
        rospy.loginfo("Reach Cartesian Pose with constraints...")
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.y -= 0.3
        constraints = moveit_msgs.msg.Constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.orientation = actual_pose.orientation
        constraints.orientation_constraints.append(orientation_constraint)
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print(success)
        rospy.loginfo("Closing the gripper 50%...")
        success &= example.reach_gripper_position(0.5)
        print(success)

    # --------------------------------------------------------------------
    # New Code: Launch the camera, capture an image, and then turn the camera off.
    if success:
        rospy.loginfo("Arm reached the goal. Turning on the camera node...")
        camera_proc = launch_camera()
        rospy.sleep(5)  # Wait for the camera node to initialize
        global camera_subscriber
        camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
        rospy.sleep(3)  # Wait for the image callback to process the image
        if camera_proc is not None:
            camera_proc.kill()
            rospy.loginfo("Camera node turned off.")
    # --------------------------------------------------------------------

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)
    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()