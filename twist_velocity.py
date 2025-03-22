#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class CameraTwistPublisher:
    def __init__(self):
        rospy.init_node("camera_twist_publisher", anonymous=True)

        # Parameters
        self.lambda_gain = rospy.get_param("~lambda", 1.0)  # Default lambda = 1.0
        
        # Subscribers
        rospy.Subscriber("/inverse_interaction_matrix", Float32MultiArray, self.inv_matrix_callback)
        rospy.Subscriber("/error_matrix", Float32MultiArray, self.error_callback)
        
        # Publisher
        self.pub_twist = rospy.Publisher("/camera_twist", Float32MultiArray, queue_size=10)

        # Storage for latest messages
        self.Ls_pinv = None  # Inverse interaction matrix (6x8)
        self.error_vector = None  # Error matrix (8x1)

    def inv_matrix_callback(self, msg):
        """Receives and stores the pseudo-inverse interaction matrix."""
        data = np.array(msg.data).reshape(6, 8)  # Ensure correct shape (6x8)
        self.Ls_pinv = data
        self.compute_twist()

    def error_callback(self, msg):
        """Receives and stores the error matrix."""
        data = np.array(msg.data).reshape(8, 1)  # Ensure correct shape (8x1)
        self.error_vector = data
        self.compute_twist()

    def compute_twist(self):
        """Computes and publishes the camera twist velocity."""
        if self.Ls_pinv is None or self.error_vector is None:
            return  # Wait for both messages

        # Compute camera twist: v_c = -lambda * (L_s_pinv @ error)
        twist_velocity = -self.lambda_gain * np.dot(self.Ls_pinv, self.error_vector)  # (6x1)
        
        # Publish twist velocity
        twist_msg = Float32MultiArray()
        twist_msg.data = twist_velocity.flatten().tolist()
        self.pub_twist.publish(twist_msg)

        rospy.loginfo(f"Published camera twist velocity: {twist_velocity.flatten()}")

if __name__ == "__main__":
    try:
        node = CameraTwistPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

