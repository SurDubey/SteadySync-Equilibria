#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from std_msgs.msg import Float32MultiArray

class CameraToEEVelocity:
    def __init__(self):
        rospy.init_node("camera_to_ee_velocity", anonymous=True)

        # TF Listener (to get real-time transforms)
        self.tf_listener = tf.TransformListener()

        # Subscribe to camera twist velocity
        rospy.Subscriber("/camera_twist", Float32MultiArray, self.camera_twist_callback)

        # Publisher for end-effector velocity
        self.pub_ee_twist = rospy.Publisher("/ee_twist", Float32MultiArray, queue_size=10)

        rospy.loginfo("Camera to End-Effector Velocity Mapping Node Started")

    def get_real_time_tf(self):
        """Fetches the real-time transformation from camera frame to end-effector."""
        try:
            self.tf_listener.waitForTransform("end_effector_link", "camera_depth_frame", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("end_effector_link", "camera_depth_frame", rospy.Time(0))

            # Convert quaternion to rotation matrix
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]  # Extract 3x3 rotation part
            return R
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("Could not get TF transform, using last known transformation.")
            return None

    def camera_twist_callback(self, msg):
        """Converts camera twist velocity to end-effector velocity dynamically."""
        camera_twist = np.array(msg.data).reshape(6, 1)  # Ensure 6x1 shape

        # Get latest transform from TF
        R_cam_to_ee = self.get_real_time_tf()
        if R_cam_to_ee is None:
            return  # Skip if no valid transform

        # Construct full transformation for twist (6x6)
        R_block = np.block([
            [R_cam_to_ee, np.zeros((3, 3))],
            [np.zeros((3, 3)), R_cam_to_ee]
        ])

        # Transform camera twist to end-effector twist
        ee_twist = np.dot(R_block, camera_twist)

        # Publish result
        twist_msg = Float32MultiArray()
        twist_msg.data = ee_twist.flatten().tolist()
        self.pub_ee_twist.publish(twist_msg)

        rospy.loginfo(f"Published EE twist: {ee_twist.flatten()}")

if __name__ == "__main__":
    try:
        node = CameraToEEVelocity()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

