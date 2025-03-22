#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""

    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')
        rospy.loginfo("here")
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
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20
            )
           
            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
                rospy.loginfo("******")
            else:
                rospy.loginfo("==================land_early>")
               
            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True
    
    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose


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
           
    def plan_trajectory(self, actual_pose):

        # Subscribe to the goal position topic and wait for a message
        #position_msg = rospy.wait_for_message("/bottle_position", geometry_msgs.msg.Point)

        # Extract the position from the received message
        #position = position_msg
        move_group = self.arm_group
        

        # Set the tolerance
        move_group.set_goal_position_tolerance(0.01)

        # Orientation constraint (we want the end effector to stay the same orientation)
        constraints = moveit_msgs.msg.Constraints()
        #orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        #orientation_constraint.orientation = actual_pose.orientation
        #constraints.orientation_constraints.append(orientation_constraint)
        constraints=None
        
        # Set the trajectory constraint if one is specified
        if constraints is not None:
            move_group.set_path_constraints(constraints)

        # Set a higher planning time (in seconds)
        move_group.set_planning_time(10.0)  # Adjust the value as needed
       
        # Define the target pose for the end effector
        pose_goal = geometry_msgs.msg.Pose()
        #position.x = round(position.x,2)
        #position.y = round(position.y,2)
        #position.z = round(position.z,2)
        pose_goal.position.x = 0.53
        pose_goal.position.y = -0.02
        pose_goal.position.z = 0.53
        pose_goal.orientation.x = 0.5
        pose_goal.orientation.y = 0.5
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5
       
        move_group.set_pose_target(pose_goal)
           
        # Plan the trajectory to the target pose
        plan = move_group.go(wait=True)

        # Ensure that there is no residual movement
        #move_group.stop()
       
        # Clear the target pose
        #move_group.clear_pose_targets()
       
        #rospy.loginfo(f"Moving to position: ({position.x}, {position.y}, {position.z})")
        return plan
           

def main():

    example = ExampleMoveItTrajectories()
   
    # For testing purposes
    success = example.is_init_success    
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass
       
    if success:
        rospy.loginfo('reaching to a gripper position: ')
        
        actual_pose = example.get_cartesian_pose()

        try:
            example.plan_trajectory(actual_pose)
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            pass

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print(success)

        rospy.loginfo("Closing the gripper 50%...")
        success &= example.reach_gripper_position(0.17)
        print(success)

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")
   

if __name__ == '__main__':
    main()

