#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String  # Import standard ROS message

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # MoveIt setup
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, '/kinova_arm/robot_description', ns='/kinova_arm')

            # Publisher for image saving requests
            self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

            rospy.loginfo("Node initialized in namespace " + rospy.get_namespace())

        except Exception as e:
            rospy.logerr(f"Initialization error: {e}")
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_cartesian_pose(self, pose, tolerance):
        """Moves the robot to the specified pose."""
        self.arm_group.set_goal_position_tolerance(tolerance)
        self.arm_group.set_pose_target(pose)
        rospy.loginfo("Moving to target pose...")
        return self.arm_group.go(wait=True)

    def save_image(self, file_path):
        """Publishes a request to save an image"""
        rospy.loginfo(f"Requesting image save: {file_path}")
        self.image_save_pub.publish(file_path)

def main():
    example = ExampleMoveItTrajectories()
    success = example.is_init_success

    if success:
        locations = [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, -0.3101, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5711, 0.5473, 0.3788, 0.4805)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5951, 0.0828, 0.2202),
                orientation=geometry_msgs.msg.Quaternion(0.5785, 0.5469, 0.3968, 0.4570)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.7028, 0.4135, 0.1986),
                orientation=geometry_msgs.msg.Quaternion(0.4849, 0.5998, 0.5073, 0.3844)
            ),
        ]

        for i, location in enumerate(locations):
            rospy.loginfo(f"Reaching Location {i+1}...")
            success &= example.reach_cartesian_pose(location, tolerance=0.01)

            if success:
                rospy.loginfo(f"Successfully reached Location {i+1}")
                file_path = f"/home/pc/pic/location_{i+1}.jpg"
                example.save_image(file_path)
                rospy.sleep(5)  # Wait for 5 seconds at the location
            else:
                rospy.logerr(f"Failed to reach Location {i+1}")
                break

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()

