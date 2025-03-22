#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo

class NormalizedPixelConverter:
    def __init__(self):
        rospy.init_node("normalized_pixel_converter", anonymous=True)

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribe to camera info to get intrinsics
        rospy.Subscriber("/my_gen3/rrbot/camera1/camera_info", CameraInfo, self.camera_info_callback)

        # Subscribe to detected corner pixel coordinates
        rospy.Subscriber("/detected_corners", Float32MultiArray, self.corner_callback)

        # Publisher for normalized coordinates
        self.pub_normalized = rospy.Publisher("/normalized_corners", Float32MultiArray, queue_size=10)

    def camera_info_callback(self, msg):
        """Extracts camera intrinsic parameters from CameraInfo message."""
        self.fx = msg.K[0]  # Focal length in x
        self.fy = msg.K[4]  # Focal length in y
        self.cx = msg.K[2]  # Principal point x
        self.cy = msg.K[5]  # Principal point y

        rospy.loginfo(f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def normalize_coordinates(self, pixel_coords):
        """Converts pixel coordinates to normalized image coordinates."""
        if None in (self.fx, self.fy, self.cx, self.cy):
            rospy.logwarn("Camera intrinsics not yet received.")
            return []

        normalized_coords = []
        for i in range(0, len(pixel_coords), 2):
            u = pixel_coords[i]   # x pixel coordinate
            v = pixel_coords[i+1] # y pixel coordinate

            x_norm = (u - self.cx) / self.fx
            y_norm = (v - self.cy) / self.fy

            normalized_coords.extend([x_norm, y_norm])

        return normalized_coords

    def corner_callback(self, msg):
        """Receives 4 corner pixel coordinates and converts them to normalized coordinates."""
        pixel_coords = msg.data
        rospy.loginfo(f"Received pixel coordinates: {pixel_coords}")

        normalized_coords = self.normalize_coordinates(pixel_coords)

        if normalized_coords:
            rospy.loginfo(f"Normalized coordinates: {normalized_coords}")

            # Publish normalized coordinates
            norm_msg = Float32MultiArray()
            norm_msg.data = normalized_coords
            self.pub_normalized.publish(norm_msg)

if __name__ == "__main__":
    try:
        converter = NormalizedPixelConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

