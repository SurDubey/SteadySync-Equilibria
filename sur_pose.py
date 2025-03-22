#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class EndEffectorPoseMonitor:
    def __init__(self):
        rospy.init_node('end_effector_pose_monitor', anonymous=True)

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")  # Change if your group name is different

        rospy.loginfo("End Effector Pose Monitor Initialized.")

        # Set loop rate (10 Hz)
        self.rate = rospy.Rate(10)  

    def print_pose(self):
        """ Continuously prints the current end-effector pose """
        while not rospy.is_shutdown():
            # Get the current pose
            pose: PoseStamped = self.arm_group.get_current_pose()

            # Extract position & orientation
            position = pose.pose.position
            orientation = pose.pose.orientation

            # Display in terminal
            rospy.loginfo(
                f"\nEnd-Effector Pose:\n"
                f"Position -> x: {position.x:.4f}, y: {position.y:.4f}, z: {position.z:.4f}\n"
                f"Orientation -> x: {orientation.x:.4f}, y: {orientation.y:.4f}, "
                f"z: {orientation.z:.4f}, w: {orientation.w:.4f}\n"
                "-------------------------------------------"
            )

            self.rate.sleep()  # Sleep to maintain loop rate

if __name__ == '__main__':
    try:
        monitor = EndEffectorPoseMonitor()
        monitor.print_pose()
    except rospy.ROSInterruptException:
        rospy.logwarn("Pose monitor interrupted.")

