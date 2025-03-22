#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class InverseInteractionMatrix:
    def __init__(self):
        rospy.init_node("inverse_interaction_matrix", anonymous=True)

        # Subscriber for normalized coordinates
        rospy.Subscriber("/normalized_corners", Float32MultiArray, self.normalized_callback)

        # Publisher for pseudo-inverse interaction matrix
        self.pub_inv_matrix = rospy.Publisher("/inverse_interaction_matrix", Float32MultiArray, queue_size=10)

    def compute_interaction_matrix(self, x, y, Z=0.5):
        """Constructs the 8x6 interaction matrix L_s."""
        L_s = np.zeros((8, 6))

        for i in range(4):
            xi, yi = x[i], y[i]

            # First two rows (for x-coordinates)
            L_s[i, 0] = -Z
            L_s[i, 1] = 1
            L_s[i, 2] = 0
            L_s[i, 3] = xi * Z
            L_s[i, 4] = 1 + yi**2
            L_s[i, 5] = -xi * yi

            # Next two rows (for y-coordinates)
            L_s[i+4, 0] = 0
            L_s[i+4, 1] = -Z
            L_s[i+4, 2] = 1
            L_s[i+4, 3] = yi * Z
            L_s[i+4, 4] = -xi * yi
            L_s[i+4, 5] = -(1 + xi**2)

        return L_s

def normalized_callback(self, msg):
    """Receives normalized coordinates, computes pseudo-inverse of L_s, and publishes it."""
    data = msg.data
    if len(data) != 8:
        rospy.logwarn("Received incorrect number of normalized coordinates.")
        return

    # Correctly extract x and y coordinates
    x = data[0:4]  # First 4 elements
    y = data[4:8]  # Last 4 elements

    # Compute interaction matrix
    L_s = self.compute_interaction_matrix(x, y)

    try:
        # Compute pseudo-inverse of the interaction matrix
        L_s_pinv = np.linalg.pinv(L_s)  # Moore-Penrose pseudo-inverse

        # Publish result
        inv_msg = Float32MultiArray()
        inv_msg.data = L_s_pinv.flatten().tolist()
        self.pub_inv_matrix.publish(inv_msg)

        rospy.loginfo("Published pseudo-inverse of interaction matrix.")

    except np.linalg.LinAlgError:
        rospy.logerr("Pseudo-inverse computation failed.")


if __name__ == "__main__":
    try:
        node = InverseInteractionMatrix()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

