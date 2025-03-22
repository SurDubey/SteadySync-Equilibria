#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class DepthExtractor:
    def __init__(self):
        rospy.init_node("depth_extractor", anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to compressed depth image
        rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw/compressedDepth", CompressedImage, self.depth_callback)

    def depth_callback(self, msg):
        """Callback function to process compressed depth image"""
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        if depth_image is None:
            rospy.logwarn("Failed to decode depth image.")
            return

        # If depth is stored in uint16 format, convert to meters
        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) / 1000.0  # Convert from mm to meters

        rospy.loginfo("Depth image received. Extracting depth...")

        # Example: Extract depth at a specific pixel (modify with real coordinates)
        u, v = 328, 133  # Example pixel coordinates (replace with real feature points)
        depth_value = depth_image[v, u]

        if np.isfinite(depth_value):  # Check if valid depth value
            rospy.loginfo(f"Depth at ({u}, {v}) = {depth_value:.3f} meters")
        else:
            rospy.logwarn(f"Invalid depth at ({u}, {v}).")

if __name__ == "__main__":
    try:
        DepthExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

