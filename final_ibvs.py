# #!/usr/bin/env python3

# import rospy
# import numpy as np
# import cv2
# import tf
# from moveit_commander import RobotCommander
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# class VisualServoing:
#     def __init__(self):
#         rospy.init_node("visual_servoing", anonymous=True)

#         # Initialize camera bridge
#         self.bridge = CvBridge()

#         # MoveIt setup
#         self.robot = RobotCommander()
#         self.arm_group = self.robot.get_group("arm")

#         # Camera parameters (initialized later from /camera_info)
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None

#         # TF Listener
#         self.tf_listener = tf.TransformListener()

#         # Desired feature points (normalized coordinates)
#         self.desired_features = np.array([
#             [-0.1521, -0.5611],  # Corner 1
#             [0.3136, -0.5590],  # Corner 2
#             [0.3283, -0.2129],  # Corner 3
#             [-0.1521, -0.1919]  # Corner 4
#         ])

#         rospy.loginfo("Visual Servoing Node Initialized")

#     def get_camera_intrinsics(self):
#         """Fetch camera intrinsics from ROS topic"""
#         rospy.loginfo("Waiting for camera intrinsics...")
#         camera_info = rospy.wait_for_message("/my_gen3/rrbot/camera1/camera_info", CameraInfo)
#         self.fx = camera_info.K[0]
#         self.fy = camera_info.K[4]
#         self.cx = camera_info.K[2]
#         self.cy = camera_info.K[5]
#         rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

#     def extract_corners(self, image):
#         """Detects rectangle corners in the given image."""
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)

#         # Find contours
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         largest_rectangle = None
#         max_area = 0

#         for contour in contours:
#             epsilon = 0.02 * cv2.arcLength(contour, True)
#             approx = cv2.approxPolyDP(contour, epsilon, True)

#             if len(approx) == 4:  # Ensure it is a quadrilateral
#                 area = cv2.contourArea(approx)
#                 if area > max_area:
#                     max_area = area
#                     largest_rectangle = approx

#         if largest_rectangle is not None:
#             return largest_rectangle.reshape(4, 2).astype(float)

#         return None

#     def normalize_corners(self, corners):
#         """Converts pixel coordinates to normalized coordinates."""
#         normalized = np.zeros((4, 2))
#         for i, (u, v) in enumerate(corners):
#             normalized[i, 0] = (u - self.cx) / self.fx
#             normalized[i, 1] = (v - self.cy) / self.fy
#         return normalized

#     def compute_interaction_matrix(self, x, y, Z=0.4):
#         """Computes the 8x6 interaction matrix L_s."""
#         L_s = np.zeros((8, 6))
#         for i in range(4):
#             xi, yi = x[i], y[i]
#             L_s[i, :] = [-Z, 1, 0, xi * Z, 1 + yi**2, -xi * yi]
#             L_s[i+4, :] = [0, -Z, 1, yi * Z, -xi * yi, -(1 + xi**2)]
#         return L_s

#     def compute_camera_twist(self, detected_features):
#         """Computes the camera twist velocity from the error vector."""
#         error_vector = (detected_features - self.desired_features).flatten()
#         L_s = self.compute_interaction_matrix(detected_features[:, 0], detected_features[:, 1])
#         L_s_pinv = np.linalg.pinv(L_s)  # Moore-Penrose pseudo-inverse
#         lambda_gain = 1.0
#         return -lambda_gain * np.dot(L_s_pinv, error_vector)

#     def get_real_time_tf(self):
#         """Fetches transformation from camera to end-effector."""
#         try:
#             self.tf_listener.waitForTransform("end_effector_link", "camera_depth_frame", rospy.Time(0), rospy.Duration(1.0))
#             (trans, rot) = self.tf_listener.lookupTransform("end_effector_link", "camera_depth_frame", rospy.Time(0))
#             R = tf.transformations.quaternion_matrix(rot)[:3, :3]  # Extract 3x3 rotation part
#             return R
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException):
#             rospy.logwarn("TF Transform unavailable, using identity matrix.")
#             return np.eye(3)

#     def map_camera_to_ee_velocity(self, camera_twist):
#         """Transforms camera twist to end-effector twist."""
#         R_cam_to_ee = self.get_real_time_tf()
#         R_block = np.block([
#             [R_cam_to_ee, np.zeros((3, 3))],
#             [np.zeros((3, 3)), R_cam_to_ee]
#         ])
#         return np.dot(R_block, camera_twist)

#     def compute_joint_velocities(self, ee_twist):
#         """Computes joint velocities from end-effector twist using MoveIt Jacobian."""
#         joint_positions = self.arm_group.get_current_joint_values()
#         J = np.array(self.robot.compute_jacobian(joint_positions)).reshape(6, 7)
#         J_pinv = np.linalg.pinv(J)  # Pseudo-inverse of Jacobian
#         return np.dot(J_pinv, ee_twist).flatten()

#     def send_joint_velocities(self, joint_velocities):
#         """Publishes joint velocity commands to the robot."""
#         traj_msg = JointTrajectory()
#         traj_msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

#         point = JointTrajectoryPoint()
#         point.velocities = joint_velocities.tolist()
#         point.time_from_start = rospy.Duration(0.1)  # Smooth updates every 100ms

#         traj_msg.points.append(point)

#         pub_joint_traj = rospy.Publisher("/my_gen3/gen3_joint_trajectory_controller/command", JointTrajectory, queue_size=10)
#         pub_joint_traj.publish(traj_msg)

#         rospy.loginfo(f"Sent Joint Velocity Command: {joint_velocities}")

#     def process_image(self, image_msg):
#         """Main pipeline to process image and execute visual servoing."""
#         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

#         # Step 1: Detect and normalize corners
#         detected_corners = self.extract_corners(cv_image)
#         if detected_corners is None:
#             rospy.logwarn("No rectangle detected.")
#             return
#         normalized_corners = self.normalize_corners(detected_corners)

#         # Step 2: Compute Camera Twist Velocity
#         camera_twist = self.compute_camera_twist(normalized_corners)

#         # Step 3: Convert Camera Twist to EE Twist
#         ee_twist = self.map_camera_to_ee_velocity(camera_twist)

#         # Step 4: Compute Joint Velocities
#         joint_velocities = self.compute_joint_velocities(ee_twist)

#         # Step 5: Move Robot
#         self.send_joint_velocities(joint_velocities)


# if __name__ == "__main__":
#     node = VisualServoing()
#     node.get_camera_intrinsics()
#     rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, node.process_image)
#     rospy.spin()

#!/usr/bin/env python3

# import rospy
# import numpy as np
# import cv2
# import tf
# from moveit_commander import RobotCommander, MoveGroupCommander
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# class VisualServoing:
#     def __init__(self):
#         rospy.init_node("visual_servoing", anonymous=True)

#         # Initialize camera bridge
#         self.bridge = CvBridge()

#         # MoveIt setup
#         self.robot = RobotCommander()
#         self.arm_group = MoveGroupCommander("arm")  # Corrected MoveGroupCommander

#         # Camera parameters (initialized later from /camera_info)
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None

#         # TF Listener
#         self.tf_listener = tf.TransformListener()

#         # Desired feature points (normalized coordinates)
#         self.desired_features = np.array([
#             [-0.1521, -0.5611],  # Corner 1
#             [0.3136, -0.5590],  # Corner 2
#             [0.3283, -0.2129],  # Corner 3
#             [-0.1521, -0.1919]  # Corner 4
#         ])

#         rospy.loginfo("Visual Servoing Node Initialized")

#     def get_camera_intrinsics(self):
#         rospy.loginfo("Waiting for camera intrinsics...")
#         camera_info = rospy.wait_for_message("/my_gen3/rrbot/camera1/camera_info", CameraInfo)
#         self.fx = camera_info.K[0]
#         self.fy = camera_info.K[4]
#         self.cx = camera_info.K[2]
#         self.cy = camera_info.K[5]
#         rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

#     def extract_corners(self, image):
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         largest_rectangle = None
#         max_area = 0

#         for contour in contours:
#             epsilon = 0.02 * cv2.arcLength(contour, True)
#             approx = cv2.approxPolyDP(contour, epsilon, True)
#             if len(approx) == 4:
#                 area = cv2.contourArea(approx)
#                 if area > max_area:
#                     max_area = area
#                     largest_rectangle = approx

#         if largest_rectangle is not None:
#             return largest_rectangle.reshape(4, 2).astype(float)
#         return None

#     def normalize_corners(self, corners):
#         normalized = np.zeros((4, 2))
#         for i, (u, v) in enumerate(corners):
#             normalized[i, 0] = (u - self.cx) / self.fx
#             normalized[i, 1] = (v - self.cy) / self.fy
#         return normalized

#     def compute_interaction_matrix(self, x, y, Z=0.4):
#         L_s = np.zeros((8, 6))
#         for i in range(4):
#             xi, yi = x[i], y[i]
#             L_s[i, :] = [-1/Z, 0, xi/Z, xi * yi, -(1 + xi**2), yi]
#             L_s[i+4, :] = [0, -1/Z, yi/Z, 1 + yi**2, -xi * yi, -xi]

#         return L_s

#     def compute_camera_twist(self, detected_features):
#         error_vector = (detected_features - self.desired_features).flatten()
#         L_s = self.compute_interaction_matrix(detected_features[:, 0], detected_features[:, 1])
#         L_s_pinv = np.linalg.pinv(L_s)
#         lambda_gain = 1.0
#         return -lambda_gain * np.dot(L_s_pinv, error_vector)

#     def get_real_time_tf(self):
#         try:
#             self.tf_listener.waitForTransform("end_effector_link", "camera_depth_frame", rospy.Time(0), rospy.Duration(1.0))
#             (trans, rot) = self.tf_listener.lookupTransform("end_effector_link", "camera_depth_frame", rospy.Time(0))
#             R = tf.transformations.quaternion_matrix(rot)[:3, :3]
#             return R
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException):
#             rospy.logwarn("TF Transform unavailable, using identity matrix.")
#             return np.eye(3)

#     def map_camera_to_ee_velocity(self, camera_twist):
#         R_cam_to_ee = self.get_real_time_tf()
#         R_block = np.block([
#             [R_cam_to_ee, np.zeros((3, 3))],
#             [np.zeros((3, 3)), R_cam_to_ee]
#         ])
#         return np.dot(R_block, camera_twist)

#     def compute_joint_velocities(self, ee_twist):
#         joint_positions = self.arm_group.get_current_joint_values()
#         J = np.array(self.arm_group.get_jacobian_matrix(joint_positions))  # Fixed Jacobian computation
#         J_pinv = np.linalg.pinv(J)
#         return np.dot(J_pinv, ee_twist).flatten()

#     def send_joint_velocities(self, joint_velocities):
#         traj_msg = JointTrajectory()
#         traj_msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
#         point = JointTrajectoryPoint()
#         point.velocities = joint_velocities.tolist()
#         point.time_from_start = rospy.Duration(0.1)
#         traj_msg.points.append(point)
#         pub_joint_traj = rospy.Publisher("/my_gen3/gen3_joint_trajectory_controller/command", JointTrajectory, queue_size=10)
#         pub_joint_traj.publish(traj_msg)
#         rospy.loginfo(f"Sent Joint Velocity Command: {joint_velocities}")

#     def process_image(self, image_msg):
#         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
#         detected_corners = self.extract_corners(cv_image)
#         if detected_corners is None:
#             rospy.logwarn("No rectangle detected.")
#             return
#         normalized_corners = self.normalize_corners(detected_corners)
#         camera_twist = self.compute_camera_twist(normalized_corners)
#         ee_twist = self.map_camera_to_ee_velocity(camera_twist)
#         joint_velocities = self.compute_joint_velocities(ee_twist)
#         self.send_joint_velocities(joint_velocities)


# if __name__ == "__main__":
#     node = VisualServoing()
#     node.get_camera_intrinsics()
#     rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, node.process_image)
#     rospy.spin()



# import rospy
# import numpy as np
# import cv2
# import tf
# from moveit_commander import RobotCommander, MoveGroupCommander
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# class VisualServoing:
#     def __init__(self):
#         rospy.init_node("visual_servoing", anonymous=True)

#         # Initialize camera bridge
#         self.bridge = CvBridge()

#         # MoveIt setup
#         self.robot = RobotCommander()
#         self.arm_group = MoveGroupCommander("arm")  # Corrected MoveGroupCommander

#         # Camera parameters (initialized later from /camera_info)
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None

#         # TF Listener
#         self.tf_listener = tf.TransformListener()

#         # Desired feature points (normalized coordinates)
#         self.desired_features = np.array([
#             [-0.1521, -0.5611],  # Corner 1
#             [0.3136, -0.5590],  # Corner 2
#             [0.3283, -0.2129],  # Corner 3
#             [-0.1521, -0.1919]  # Corner 4
#         ])

#         rospy.loginfo("Visual Servoing Node Initialized")

#     def get_camera_intrinsics(self):
#         rospy.loginfo("Waiting for camera intrinsics...")
#         camera_info = rospy.wait_for_message("/my_gen3/rrbot/camera1/camera_info", CameraInfo)
#         self.fx = camera_info.K[0]
#         self.fy = camera_info.K[4]
#         self.cx = camera_info.K[2]
#         self.cy = camera_info.K[5]
#         rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

#     def extract_corners(self, image):
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         largest_rectangle = None
#         max_area = 0

#         for contour in contours:
#             epsilon = 0.02 * cv2.arcLength(contour, True)
#             approx = cv2.approxPolyDP(contour, epsilon, True)
#             if len(approx) == 4:
#                 area = cv2.contourArea(approx)
#                 if area > max_area:
#                     max_area = area
#                     largest_rectangle = approx

#         if largest_rectangle is not None:
#             return largest_rectangle.reshape(4, 2).astype(float)
#         return None

#     def normalize_corners(self, corners):
#         normalized = np.zeros((4, 2))
#         for i, (u, v) in enumerate(corners):
#             normalized[i, 0] = (u - self.cx) / self.fx
#             normalized[i, 1] = (v - self.cy) / self.fy
#         return normalized

#     def compute_interaction_matrix(self, x, y, Z=0.4):
#         L_s = np.zeros((8, 6))
#         for i in range(4):
#             xi, yi = x[i], y[i]
#             L_s[i, :] = [-1/Z, 0, xi/Z, xi * yi, -(1 + xi**2), yi]
#             L_s[i+4, :] = [0, -1/Z, yi/Z, 1 + yi**2, -xi * yi, -xi]

#         return L_s

#     def compute_camera_twist(self, detected_features):
#         error_vector = (detected_features - self.desired_features).flatten()
#         L_s = self.compute_interaction_matrix(detected_features[:, 0], detected_features[:, 1])
#         L_s_pinv = np.linalg.pinv(L_s)
#         lambda_gain = 1.0
#         return -lambda_gain * np.dot(L_s_pinv, error_vector)

#     def get_real_time_tf(self):
#         try:
#             self.tf_listener.waitForTransform("end_effector_link", "camera_depth_frame", rospy.Time(0), rospy.Duration(1.0))
#             (trans, rot) = self.tf_listener.lookupTransform("end_effector_link", "camera_depth_frame", rospy.Time(0))
#             R = tf.transformations.quaternion_matrix(rot)[:3, :3]
#             return R
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException):
#             rospy.logwarn("TF Transform unavailable, using identity matrix.")
#             return np.eye(3)

#     def map_camera_to_ee_velocity(self, camera_twist):
#         R_cam_to_ee = self.get_real_time_tf()
#         R_block = np.block([
#             [R_cam_to_ee, np.zeros((3, 3))],
#             [np.zeros((3, 3)), R_cam_to_ee]
#         ])
#         return np.dot(R_block, camera_twist)

#     def compute_joint_velocities(self, ee_twist):
#         joint_positions = self.arm_group.get_current_joint_values()
#         J = np.array(self.arm_group.get_jacobian_matrix(joint_positions))  # Fixed Jacobian computation
#         J_pinv = np.linalg.pinv(J)
#         return np.dot(J_pinv, ee_twist).flatten()

#     def update_joint_positions(self, joint_velocities, dt=0.1):
#         current_joint_positions = np.array(self.arm_group.get_current_joint_values())
#         new_joint_positions = current_joint_positions + joint_velocities * dt
#         self.arm_group.set_joint_value_target(new_joint_positions.tolist())
#         self.arm_group.go(wait=True)
#         rospy.loginfo(f"Updated Joint Positions: {new_joint_positions}")

#     def send_joint_velocities(self, joint_velocities):
#         self.update_joint_positions(joint_velocities)

#     def process_image(self, image_msg):
#         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
#         detected_corners = self.extract_corners(cv_image)
#         if detected_corners is None:
#             rospy.logwarn("No rectangle detected.")
#             return
#         normalized_corners = self.normalize_corners(detected_corners)
#         camera_twist = self.compute_camera_twist(normalized_corners)
#         ee_twist = self.map_camera_to_ee_velocity(camera_twist)
#         joint_velocities = self.compute_joint_velocities(ee_twist)
#         self.send_joint_velocities(joint_velocities)


# if __name__ == "__main__":
#     node = VisualServoing()
#     node.get_camera_intrinsics()
#     rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, node.process_image)
#     rospy.spin()


import rospy
import numpy as np
import cv2
import tf
from moveit_commander import RobotCommander, MoveGroupCommander
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VisualServoing:
    def __init__(self):
        rospy.init_node("visual_servoing", anonymous=True)

        # Initialize camera bridge
        self.bridge = CvBridge()

        # MoveIt setup
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("arm")  # Corrected MoveGroupCommander

        # Camera parameters (initialized later from /camera_info)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Desired feature points (normalized coordinates)
        self.desired_features = np.array([
            [-0.1521, -0.5611],  # Corner 1
            [0.3136, -0.5590],  # Corner 2
            [0.3283, -0.2129],  # Corner 3
            [-0.1521, -0.1919]  # Corner 4
        ])

        # Get joint limits
        self.joint_limits = self.get_joint_limits()

        rospy.loginfo("Visual Servoing Node Initialized")

    def get_camera_intrinsics(self):
        rospy.loginfo("Waiting for camera intrinsics...")
        camera_info = rospy.wait_for_message("/my_gen3/rrbot/camera1/camera_info", CameraInfo)
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]
        rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def get_joint_limits(self):
        """Retrieve joint limits for safety."""
        joint_limits = {}
        for joint_name in self.arm_group.get_active_joints():
            joint_limits[joint_name] = self.robot.get_joint(joint_name).bounds()
        return joint_limits

    def extract_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_rectangle = None
        max_area = 0

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4:
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    largest_rectangle = approx

        if largest_rectangle is not None:
            return largest_rectangle.reshape(4, 2).astype(float)
        return None

    def normalize_corners(self, corners):
        normalized = np.zeros((4, 2))
        for i, (u, v) in enumerate(corners):
            normalized[i, 0] = (u - self.cx) / self.fx
            normalized[i, 1] = (v - self.cy) / self.fy
        return normalized

    def compute_interaction_matrix(self, x, y, Z=0.4):
        L_s = np.zeros((8, 6))
        for i in range(4):
            xi, yi = x[i], y[i]
            L_s[i, :] = [-1/Z, 0, xi/Z, xi * yi, -(1 + xi**2), yi]
            L_s[i+4, :] = [0, -1/Z, yi/Z, 1 + yi**2, -xi * yi, -xi]

        return L_s

    def compute_camera_twist(self, detected_features):
        error_vector = (detected_features - self.desired_features).flatten()
        L_s = self.compute_interaction_matrix(detected_features[:, 0], detected_features[:, 1])
        L_s_pinv = np.linalg.pinv(L_s)
        lambda_gain = 0.5  # Reduced gain for stability
        return -lambda_gain * np.dot(L_s_pinv, error_vector)

    def get_real_time_tf(self):
        try:
            self.tf_listener.waitForTransform("end_effector_link", "camera_depth_frame", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("end_effector_link", "camera_depth_frame", rospy.Time(0))
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            return R
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("TF Transform unavailable, using identity matrix.")
            return np.eye(3)

    def map_camera_to_ee_velocity(self, camera_twist):
        R_cam_to_ee = self.get_real_time_tf()
        R_block = np.block([
            [R_cam_to_ee, np.zeros((3, 3))],
            [np.zeros((3, 3)), R_cam_to_ee]
        ])
        return np.dot(R_block, camera_twist)

    def compute_joint_velocities(self, ee_twist):
        joint_positions = self.arm_group.get_current_joint_values()
        J = np.array(self.arm_group.get_jacobian_matrix(joint_positions))  
        J_pinv = np.linalg.pinv(J)
        return np.dot(J_pinv, ee_twist).flatten()

    def is_within_limits(self, joint_positions):
        """Check if joint positions are within limits."""
        for i, joint_name in enumerate(self.arm_group.get_active_joints()):
            min_limit, max_limit = self.joint_limits[joint_name]
            if joint_positions[i] < min_limit or joint_positions[i] > max_limit:
                rospy.logwarn(f"Joint {joint_name} out of bounds: {joint_positions[i]}")
                return False
        return True

    def update_joint_positions(self, joint_velocities, dt=0.5):
        """Update joint positions with limit checks."""
        current_joint_positions = np.array(self.arm_group.get_current_joint_values())
        new_joint_positions = current_joint_positions + joint_velocities * dt

        # Clip values to joint limits
        for i, joint_name in enumerate(self.arm_group.get_active_joints()):
            min_limit, max_limit = self.joint_limits[joint_name]
            new_joint_positions[i] = np.clip(new_joint_positions[i], min_limit, max_limit)

        # Check if the movement is within valid range
        if not self.is_within_limits(new_joint_positions):
            rospy.logwarn("Computed joint positions are out of bounds. Skipping motion.")
            return

        self.arm_group.set_joint_value_target(new_joint_positions.tolist())
        success = self.arm_group.go(wait=True)

        if success:
            rospy.loginfo(f"Updated Joint Positions: {new_joint_positions}")
        else:
            rospy.logwarn("MoveIt failed to execute motion.")

    def send_joint_velocities(self, joint_velocities):
        self.update_joint_positions(joint_velocities)

    def process_image(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        detected_corners = self.extract_corners(cv_image)
        if detected_corners is None:
            rospy.logwarn("No rectangle detected.")
            return
        normalized_corners = self.normalize_corners(detected_corners)
        camera_twist = self.compute_camera_twist(normalized_corners)
        ee_twist = self.map_camera_to_ee_velocity(camera_twist)
        joint_velocities = self.compute_joint_velocities(ee_twist)
        self.send_joint_velocities(joint_velocities)


if __name__ == "__main__":
    node = VisualServoing()
    node.get_camera_intrinsics()
    rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, node.process_image)
    rospy.spin()

