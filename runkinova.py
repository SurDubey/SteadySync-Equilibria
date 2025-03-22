#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def plan_trajectory():
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object for the Kinova Gen3 arm
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a DisplayTrajectory ROS publisher
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Define the target pose for the end effector
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.5
    pose_goal.orientation.y = 0.5
    pose_goal.orientation.z = 0.5
    pose_goal.orientation.w = 0.5
    pose_goal.position.x = 0.1
    pose_goal.position.y = 0.1
    pose_goal.position.z = 1.25
    move_group.set_pose_target(pose_goal)

    # Plan the trajectory to the target pose
    plan = move_group.go(wait=True)

    # Ensure that there is no residual movement
    move_group.stop()

    # Clear the target pose
    move_group.clear_pose_targets()
    
    # Return the plan
    return plan

if __name__ == '__main__':
    try:
        plan=plan_trajectory()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

