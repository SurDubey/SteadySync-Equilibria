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

class Gen3RectangleDetection:
    def __init__(self):
        rospy.init_node('gen3_rectangle_detection', anonymous=True)

        # Handle shutdown signals
        signal.signal(signal.SIGINT, self.shutdown)

        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

        # Publisher for processed image
        self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

        self.running = True  # Control flag

    def order_corners(self, pts):
        """ Order the four corners as (top-left, top-right, bottom-right, bottom-left) """
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)

        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left

        return rect

    def calculate_distances(self, corners):
        """Calculate Euclidean distances between rectangle corners"""
        distances = []
        for i in range(4):
            pt1 = corners[i]
            pt2 = corners[(i + 1) % 4]  # Next point (loop back at end)
            distance = np.linalg.norm(pt1 - pt2)  # Euclidean distance
            distances.append(distance)
        return distances

    def detect_rectangle(self, frame):
        """Detects the largest rectangle in the live camera feed and overlays distances"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_rectangle = None
        max_area = 0

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  # Ensure it is a quadrilateral
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    largest_rectangle = approx

        if largest_rectangle is not None:
            ordered_corners = self.order_corners(largest_rectangle.reshape(4, 2))

            # Compute distances
            distances = self.calculate_distances(ordered_corners)

            # Draw detected rectangle
            cv2.drawContours(frame, [largest_rectangle], -1, (0, 255, 0), 3)

            # Draw circles at corners and annotate distances
            for i, (x, y) in enumerate(ordered_corners):
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                pt1 = tuple(ordered_corners[i].astype(int))
                pt2 = tuple(ordered_corners[(i + 1) % 4].astype(int))
                mid_pt = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)

                # Display distance in pixels on screen
                cv2.putText(frame, f"{distances[i]:.1f} px", mid_pt, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

            # Print distances in terminal
            print("\nDetected Rectangle Corner Distances (in pixels):")
            for i, dist in enumerate(distances):
                print(f"Distance {i+1}-{(i+2)%4+1}: {dist:.2f} px")

        return frame  # Return frame with detected rectangle and distances

    def image_callback(self, msg):
        """Callback function for processing live camera feed"""
        if not self.running:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Detect rectangle and show distances
            processed_image = self.detect_rectangle(cv_image)

            # Publish the processed image
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
                self.image_pub.publish(processed_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert image: {e}")

            # Display the processed image in OpenCV window
            cv2.imshow("Rectangle Detection with Distances", processed_image)
            key = cv2.waitKey(1)

            if key == 27:  # Press 'ESC' to stop
                self.shutdown(None, None)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def shutdown(self, signum, frame):
        """Handle shutdown properly"""
        rospy.loginfo("Shutting down rectangle detection node.")
        self.running = False
        self.image_sub.unregister()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User Interruption")
        sys.exit(0)

    def run(self):
        """Main function to start detecting rectangle"""
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = Gen3RectangleDetection()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")

