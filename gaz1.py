#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_robot():
    # Initialize the ROS node
    rospy.init_node('move_gen3_robot')

    # Create an action client for the joint trajectory controller
    action_client = actionlib.SimpleActionClient(
        '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    rospy.loginfo("Waiting for action server...")
    action_client.wait_for_server()
    rospy.loginfo("Action server connected!")

    # Define the target joint positions (in radians)
    target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

    # Create a JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = [
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6", "joint_7"
    ]

    point = JointTrajectoryPoint()
    point.positions = target_joints
    point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

    trajectory.points.append(point)

    # Create a goal message and send it
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    action_client.send_goal(goal)

    rospy.loginfo("Sent goal to move the arm")
    action_client.wait_for_result()
    rospy.loginfo("Motion completed!")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

