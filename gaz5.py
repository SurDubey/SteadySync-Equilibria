#!/usr/bin/env python

import rospy
import actionlib
import cv2
import numpy as np
import signal
import sys
import os
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Gen3FeatureDetection:
    def __init__(self):
        rospy.init_node('gen3_feature_detection', anonymous=True)

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

        # Publisher for processed images
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)

        self.image = None
        self.running = True  # Control flag
        self.image_save_path = "/home/pc/pic"
        os.makedirs(self.image_save_path, exist_ok=True)  # Ensure directory exists

    def move_robot(self, target_joints):
        """Moves the robot to the given joint positions."""
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
        rospy.sleep(2)  # Wait 2 seconds for stability

    def capture_image(self, position_name):
        """Captures and saves an image at the given position."""
        if self.image is not None:
            image_path = os.path.join(self.image_save_path, f"{position_name}.jpg")
            cv2.imwrite(image_path, self.image)
            rospy.loginfo(f"Saved image at {image_path}")
        else:
            rospy.logwarn("No image captured yet!")

    def detect_edges(self, frame):
        """Detects edges in the image using Canny edge detection."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        return edges

    def detect_corners(self, frame):
        """Detects corners using Harris Corner Detection."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray, 2, 3, 0.04)
        dst = cv2.dilate(dst, None)
        frame[dst > 0.01 * dst.max()] = [0, 0, 255]  # Mark corners in red
        return frame

    def process_image(self, frame):
        """Processes the image to detect both edges and corners."""
        edges = self.detect_edges(frame)
        corners = self.detect_corners(frame)
        combined = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert edges to 3-channel
        combined = cv2.addWeighted(combined, 0.5, corners, 0.5, 0)  # Blend edges and corners
        return combined

    def image_callback(self, msg):
        """Callback function for processing live camera feed."""
        if not self.running:
            return

        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def shutdown(self, signum, frame):
        """Handle shutdown properly."""
        rospy.loginfo("Shutting down feature detection node.")
        self.running = False
        self.image_sub.unregister()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User Interruption")
        sys.exit(0)

    def run(self):
        """Main function to move robot, capture images, and process them."""
        # Move to position 1 and capture image
        pos1 = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]
        self.move_robot(pos1)
        self.capture_image("POS1")

        # Process and display features at POS1
        processed_img1 = self.process_image(self.image)
        cv2.imshow("Processed Image POS1", processed_img1)
        cv2.waitKey(1000)

        # Move to position 2 and capture image
        pos2 = [0.130, 0.858, 3.134, -0.339, -0.024, -1.788, 1.703]
        self.move_robot(pos2)
        self.capture_image("POS2")

        # Process and display features at POS2
        processed_img2 = self.process_image(self.image)
        cv2.imshow("Processed Image POS2", processed_img2)
        cv2.waitKey(1000)

        rospy.spin()

if __name__ == '__main__':
    try:
        detector = Gen3FeatureDetection()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")

