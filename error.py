#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

class FeatureErrorPublisher:
    def __init__(self):
        rospy.init_node("feature_error_publisher", anonymous=True)

        # Error matrix publisher
        self.pub_error = rospy.Publisher("/error_matrix", Float32MultiArray, queue_size=10)

        # Subscribe to detected normalized corners
        rospy.Subscriber("/normalized_corners", Float32MultiArray, self.detected_callback)

        # Desired feature points (normalized coordinates)
        self.desired_features = [
            -0.1521, -0.5611,  # Corner 1 (x1*, y1*)
             0.3136, -0.5590,  # Corner 2 (x2*, y2*)
             0.3283, -0.2129,  # Corner 3 (x3*, y3*)
            -0.1521, -0.1919   # Corner 4 (x4*, y4*)
        ]

    def detected_callback(self, msg):
        """Callback for detected normalized corners."""
        detected_features = msg.data
        rospy.loginfo(f"Received detected normalized corners: {detected_features}")

        if len(detected_features) != 8:
            rospy.logwarn("Expected 8 elements (4 corners * 2 values each), but received incorrect size.")
            return

        # Compute error matrix
        error_matrix = [(detected_features[i] - self.desired_features[i]) for i in range(8)]

        rospy.loginfo(f"Computed error matrix: {error_matrix}")

        # Publish the error matrix as an 8×1 matrix
        error_msg = Float32MultiArray()
        error_msg.layout.dim.append(Float32MultiArray().layout.dim[0])
        error_msg.layout.dim[0].size = 8  # 8 rows
        error_msg.layout.dim[0].stride = 1
        error_msg.layout.dim[0].label = "error_matrix"

        error_msg.data = error_matrix
        self.pub_error.publish(error_msg)

if __name__ == "__main__":
    try:
        node = FeatureErrorPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

