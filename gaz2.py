#!/usr/bin/env python

import rospy
import actionlib
import cv2
import numpy as np
import signal
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Gen3EdgeDetection:
    def __init__(self):
        rospy.init_node('gen3_edge_detection', anonymous=True)

        # Handle shutdown signals
        signal.signal(signal.SIGINT, self.shutdown)

        # Initialize action client for robot movement
        self.action_client = actionlib.SimpleActionClient(
            '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for action server...")
        self.action_client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

        # **NEW: Publisher for edge-detected image**
        self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

        self.running = True  # Control flag

    def move_robot(self):
        """Moves the robot to a predefined position"""
        target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

        trajectory = JointTrajectory()
        trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

        trajectory.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self.action_client.send_goal(goal)

        rospy.loginfo("Sent goal to move the arm")
        self.action_client.wait_for_result()
        rospy.loginfo("Motion completed!")

    def detect_edges(self, frame):
        """Detects edges in the live camera feed"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        return edges  # Return edge-detected image

    def image_callback(self, msg):
        """Callback function for processing live camera feed"""
        if not self.running:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Perform edge detection
            edges = self.detect_edges(cv_image)

            # Publish the edge-detected image to ROS topic
            try:
                edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
                self.image_pub.publish(edge_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert image: {e}")

            # Display the processed image in OpenCV window
            cv2.imshow("Edge Detection", edges)
            key = cv2.waitKey(1)

            if key == 27:  # Press 'ESC' to stop
                self.shutdown(None, None)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def shutdown(self, signum, frame):
        """Handle shutdown properly"""
        rospy.loginfo("Shutting down edge detection node.")
        self.running = False
        self.image_sub.unregister()  # Unsubscribe from topic
        cv2.destroyAllWindows()  # Close OpenCV windows
        rospy.signal_shutdown("User Interruption")
        sys.exit(0)

    def run(self):
        """Main function to move robot and detect edges"""
        self.move_robot()
        rospy.spin()  # Keep node running for continuous image processing


if __name__ == '__main__':
    try:
        detector = Gen3EdgeDetection()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")

