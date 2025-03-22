#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class CornerExtractor:
    def __init__(self):
        rospy.init_node("corner_extractor", anonymous=True)
        
        # ROS Image Subscriber
        self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

        # Publisher for detected corners
        self.corners_pub = rospy.Publisher("/detected_corners", Float32MultiArray, queue_size=10)

        self.bridge = CvBridge()

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

    def detect_corners(self, image):
        """Detects the four corners of a rectangular block in an image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
            return ordered_corners.tolist()  # Return as list of (x, y) tuples

        return []

    def image_callback(self, msg):
        """Receives an image and detects rectangle corners."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners = self.detect_corners(cv_image)

            if corners:
                # Convert to ROS message format
                corners_flat = [coord for point in corners for coord in point]
                corner_msg = Float32MultiArray()
                corner_msg.data = corners_flat

                self.corners_pub.publish(corner_msg)
                rospy.loginfo(f"Published corner coordinates: {corners_flat}")
            else:
                rospy.logwarn("No rectangle detected.")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    try:
        node = CornerExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


