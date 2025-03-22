#!/usr/bin/env python3

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
    def __init__(self):
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
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, '/kinova_arm/robot_description',ns='/kinova_arm')
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

    def reach_cartesian_pose(self, pose, tolerance, constraints=None):
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

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

def main():
    example = ExampleMoveItTrajectories()

    # For testing purposes
    success = example.is_init_success

    if success:
        # Define the target poses
        location_1 = geometry_msgs.msg.Pose()
        location_1.position.x = 0.5882
        location_1.position.y = -0.3101
        location_1.position.z = 0.1821
        location_1.orientation.x = 0.5711
        location_1.orientation.y = 0.5473
        location_1.orientation.z = 0.3788
        location_1.orientation.w = 0.4805

        location_2 = geometry_msgs.msg.Pose()
        location_2.position.x = 0.5951
        location_2.position.y = 0.0828
        location_2.position.z = 0.2202
        location_2.orientation.x = 0.5785
        location_2.orientation.y = 0.5469
        location_2.orientation.z = 0.3968
        location_2.orientation.w = 0.4570

        location_3 = geometry_msgs.msg.Pose()
        location_3.position.x = 0.7028
        location_3.position.y = 0.4135
        location_3.position.z = 0.1986
        location_3.orientation.x = 0.4849
        location_3.orientation.y = 0.5998
        location_3.orientation.z = 0.5073
        location_3.orientation.w = 0.3844

        # List of locations to visit
        locations = [location_1, location_2, location_3]

        for i, location in enumerate(locations):
            rospy.loginfo(f"Reaching Location {i+1}...")
            success &= example.reach_cartesian_pose(location, tolerance=0.01)
            if success:
                rospy.loginfo(f"Successfully reached Location {i+1}")
                rospy.sleep(5)  # Stay at the location for 5 seconds
            else:
                rospy.logerr(f"Failed to reach Location {i+1}")
                break

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()
