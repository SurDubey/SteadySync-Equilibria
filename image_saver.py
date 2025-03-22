#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.save_sub = rospy.Subscriber('/save_image_topic', String, self.save_image_callback)
        self.latest_image = None
        rospy.loginfo("Image saver node initialized.")

    def image_callback(self, msg):
        """Updates the latest received image."""
        self.latest_image = msg

    def save_image_callback(self, msg):
        """Saves the latest image when a request is received."""
        if self.latest_image is None:
            rospy.logerr("No image available to save.")
            return

        file_path = msg.data  # File path comes from the MoveIt node
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            cv2.imwrite(file_path, cv_image)
            rospy.loginfo(f"Saved image to: {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")

if __name__ == '__main__':
    ImageSaver()
    rospy.spin()

