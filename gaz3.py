# 


#!/usr/bin/env python

#!/usr/bin/env python

#!/usr/bin/env python

#!/usr/bin/env python

# import rospy
# import actionlib
# import cv2
# import numpy as np
# import signal
# import sys
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class Gen3RectangleDetection:
#     def __init__(self):
#         rospy.init_node('gen3_rectangle_detection', anonymous=True)

#         # Handle shutdown signals
#         signal.signal(signal.SIGINT, self.shutdown)

#         # Initialize action client for robot movement
#         self.action_client = actionlib.SimpleActionClient(
#             '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )

#         rospy.loginfo("Waiting for action server...")
#         self.action_client.wait_for_server()
#         rospy.loginfo("Action server connected!")

#         # Initialize CV bridge for image conversion
#         self.bridge = CvBridge()

#         # Subscribe to the camera topic
#         self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

#         # Publisher for processed image
#         self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

#         self.running = True  # Control flag

#     def move_robot(self):
#         """Moves the robot to a predefined position"""
#         target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

#         trajectory = JointTrajectory()
#         trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

#         point = JointTrajectoryPoint()
#         point.positions = target_joints
#         point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

#         trajectory.points.append(point)

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory = trajectory
#         self.action_client.send_goal(goal)

#         rospy.loginfo("Sent goal to move the arm")
#         self.action_client.wait_for_result()
#         rospy.loginfo("Motion completed!")

#     def detect_rectangle(self, frame):
#         """Detects the largest rectangle using adaptive thresholding and Harris corner detection."""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
#         # **Step 1: Adaptive Thresholding** (Better than Canny)
#         adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                                                 cv2.THRESH_BINARY_INV, 11, 2)

#         # **Step 2: Morphological Transformations** (Enhance rectangle detection)
#         kernel = np.ones((3, 3), np.uint8)
#         closed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

#         # **Step 3: Find Contours**
#         contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
#             ordered_corners = self.order_corners(largest_rectangle.reshape(4, 2))

#             # **Step 4: Harris Corner Detection for Stability**
#             harris_corners = cv2.cornerHarris(gray, 2, 3, 0.04)
#             harris_corners = cv2.dilate(harris_corners, None)
            
#             # Filter detected corners
#             for x, y in ordered_corners:
#                 if harris_corners[int(y), int(x)] > 0.01 * harris_corners.max():
#                     cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 255), -1)  # Draw detected corners

#             # **Draw rectangle and corners**
#             cv2.drawContours(frame, [largest_rectangle], -1, (0, 255, 0), 2)

#             print("\nDetected Rectangle Corners (u, v) pixels:")
#             for i, (u, v) in enumerate(ordered_corners):
#                 print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

#         return frame  # Return frame with detected rectangle

#     def order_corners(self, pts):
#         """ Order the four corners as (top-left, top-right, bottom-right, bottom-left) """
#         rect = np.zeros((4, 2), dtype="float32")
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)

#         rect[0] = pts[np.argmin(s)]  # Top-left
#         rect[2] = pts[np.argmax(s)]  # Bottom-right
#         rect[1] = pts[np.argmin(diff)]  # Top-right
#         rect[3] = pts[np.argmax(diff)]  # Bottom-left

#         return rect

#     def image_callback(self, msg):
#         """Callback function for processing live camera feed"""
#         if not self.running:
#             return

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#             # Detect rectangle and show its corners
#             processed_image = self.detect_rectangle(cv_image)

#             # Publish the processed image
#             try:
#                 processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
#                 self.image_pub.publish(processed_msg)
#             except CvBridgeError as e:
#                 rospy.logerr(f"Failed to convert image: {e}")

#             # Display the processed image in OpenCV window
#             cv2.imshow("Rectangle Detection", processed_image)
#             key = cv2.waitKey(1)

#             if key == 27:  # Press 'ESC' to stop
#                 self.shutdown(None, None)

#         except Exception as e:
#             rospy.logerr(f"Failed to process image: {e}")

#     def shutdown(self, signum, frame):
#         """Handle shutdown properly"""
#         rospy.loginfo("Shutting down rectangle detection node.")
#         self.running = False
#         self.image_sub.unregister()
#         cv2.destroyAllWindows()
#         rospy.signal_shutdown("User Interruption")
#         sys.exit(0)

#     def run(self):
#         """Main function to move robot and detect rectangle"""
#         self.move_robot()
#         rospy.spin()


# if __name__ == '__main__':
#     try:
#         detector = Gen3RectangleDetection()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node interrupted.")


#!/usr/bin/env python
#shi tomas
# import rospy
# import actionlib
# import cv2
# import numpy as np
# import signal
# import sys
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class Gen3RectangleDetection:
#     def __init__(self):
#         rospy.init_node('gen3_rectangle_detection', anonymous=True)

#         # Handle shutdown signals
#         signal.signal(signal.SIGINT, self.shutdown)

#         # Initialize action client for robot movement
#         self.action_client = actionlib.SimpleActionClient(
#             '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )

#         rospy.loginfo("Waiting for action server...")
#         self.action_client.wait_for_server()
#         rospy.loginfo("Action server connected!")

#         # Initialize CV bridge for image conversion
#         self.bridge = CvBridge()

#         # Subscribe to the camera topic
#         self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

#         # Publisher for processed image
#         self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

#         self.running = True  # Control flag

#     def move_robot(self):
#         """Moves the robot to a predefined position"""
#         target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

#         trajectory = JointTrajectory()
#         trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

#         point = JointTrajectoryPoint()
#         point.positions = target_joints
#         point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

#         trajectory.points.append(point)

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory = trajectory
#         self.action_client.send_goal(goal)

#         rospy.loginfo("Sent goal to move the arm")
#         self.action_client.wait_for_result()
#         rospy.loginfo("Motion completed!")

#     def detect_rectangle(self, frame):
#         """Detects the largest rectangle and finds its four corners accurately."""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # **Step 1: Adaptive Thresholding (Better than Canny)**
#         adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                                                 cv2.THRESH_BINARY_INV, 11, 2)

#         # **Step 2: Morphological Operations to remove noise**
#         kernel = np.ones((3, 3), np.uint8)
#         processed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

#         # **Step 3: Find Contours**
#         contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
#             ordered_corners = self.order_corners(largest_rectangle.reshape(4, 2))

#             # **Step 4: Shi-Tomasi Corner Detection (More Stable)**
#             refined_corners = self.refine_corners(gray, ordered_corners)

#             # Draw detected rectangle
#             cv2.drawContours(frame, [largest_rectangle], -1, (0, 255, 0), 2)

#             # Draw circles at refined corners
#             for (x, y) in refined_corners:
#                 cv2.circle(frame, (int(x), int(y)), 6, (0, 0, 255), -1)

#             # Print corner coordinates
#             print("\nDetected Rectangle Corners (u, v) pixels:")
#             for i, (u, v) in enumerate(refined_corners):
#                 print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

#         return frame  # Return frame with detected rectangle

#     def refine_corners(self, gray, corners):
#         """Refine corner positions using Shi-Tomasi corner detection."""
#         corners = np.array(corners, dtype=np.float32)

#         refined_corners = cv2.goodFeaturesToTrack(gray, maxCorners=4, qualityLevel=0.1, minDistance=10)
#         if refined_corners is not None:
#             refined_corners = refined_corners.reshape(-1, 2)
#             return self.order_corners(refined_corners)
#         return corners

#     def order_corners(self, pts):
#         """Order the four corners as (top-left, top-right, bottom-right, bottom-left)."""
#         rect = np.zeros((4, 2), dtype="float32")
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)

#         rect[0] = pts[np.argmin(s)]  # Top-left
#         rect[2] = pts[np.argmax(s)]  # Bottom-right
#         rect[1] = pts[np.argmin(diff)]  # Top-right
#         rect[3] = pts[np.argmax(diff)]  # Bottom-left

#         return rect

#     def image_callback(self, msg):
#         """Callback function for processing live camera feed"""
#         if not self.running:
#             return

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#             # Detect rectangle and show its corners
#             processed_image = self.detect_rectangle(cv_image)

#             # Publish the processed image
#             try:
#                 processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
#                 self.image_pub.publish(processed_msg)
#             except CvBridgeError as e:
#                 rospy.logerr(f"Failed to convert image: {e}")

#             # Display the processed image in OpenCV window
#             cv2.imshow("Rectangle Detection", processed_image)
#             key = cv2.waitKey(1)

#             if key == 27:  # Press 'ESC' to stop
#                 self.shutdown(None, None)

#         except Exception as e:
#             rospy.logerr(f"Failed to process image: {e}")

#     def shutdown(self, signum, frame):
#         """Handle shutdown properly"""
#         rospy.loginfo("Shutting down rectangle detection node.")
#         self.running = False
#         self.image_sub.unregister()
#         cv2.destroyAllWindows()
#         rospy.signal_shutdown("User Interruption")
#         sys.exit(0)

#     def run(self):
#         """Main function to move robot and detect rectangle"""
#         self.move_robot()
#         rospy.spin()


# if __name__ == '__main__':
#     try:
#         detector = Gen3RectangleDetection()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node interrupted.")

#!/usr/bin/env python

#!/usr/bin/env python

# import rospy
# import actionlib
# import cv2
# import numpy as np
# import signal
# import sys
# from control_msgs.msg import FollowJointTrajectoryAction
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from itertools import combinations

# class Gen3RectangleDetection:
#     def __init__(self):
#         rospy.init_node('gen3_rectangle_detection', anonymous=True)

#         # Handle shutdown signals
#         signal.signal(signal.SIGINT, self.shutdown)

#         # Initialize CV bridge for image conversion
#         self.bridge = CvBridge()

#         # Subscribe to the camera topic
#         self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

#         # Publisher for processed image
#         self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

#         self.running = True  # Control flag

#     def detect_rectangle(self, frame):
#         """Detects the largest rectangle using Hough Transform & finds its four corners accurately."""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # **Step 1: Adaptive Thresholding**
#         adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                                                 cv2.THRESH_BINARY_INV, 11, 2)

#         # **Step 2: Morphological Operations**
#         kernel = np.ones((3, 3), np.uint8)
#         processed = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

#         # **Step 3: Detect Edges & Lines using Hough Transform**
#         edges = cv2.Canny(processed, 50, 150, apertureSize=3)
#         lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=80, minLineLength=50, maxLineGap=10)

#         if lines is None:
#             return frame  # No rectangle detected

#         # **Step 4: Find Intersection Points of Detected Lines**
#         points = []
#         for (line1, line2) in combinations(lines, 2):
#             pt = self.compute_intersection(line1[0], line2[0], frame.shape)
#             if pt is not None:
#                 points.append(pt)

#         # **Step 5: Filter Four Strongest Points**
#         if len(points) < 4:
#             return frame  # Not enough corners detected

#         points = np.array(points, dtype=np.float32)
#         rect_corners = self.order_corners(points[:4])

#         # **Step 6: Draw detected rectangle**
#         cv2.polylines(frame, [np.int32(rect_corners)], isClosed=True, color=(0, 255, 0), thickness=2)

#         # **Step 7: Draw corners & print coordinates**
#         for (x, y) in rect_corners:
#             cv2.circle(frame, (int(x), int(y)), 6, (0, 0, 255), -1)

#         print("\nDetected Rectangle Corners (u, v) pixels:")
#         for i, (u, v) in enumerate(rect_corners):
#             print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

#         return frame

#     def compute_intersection(self, line1, line2, img_shape):
#         """Computes the intersection point of two lines given as (x1, y1, x2, y2)."""
#         x1, y1, x2, y2 = line1
#         x3, y3, x4, y4 = line2

#         denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
#         if denom == 0:
#             return None  # Lines are parallel

#         px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
#         py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

#         # Clamp the points to image boundaries
#         px = np.clip(px, 0, img_shape[1] - 1)
#         py = np.clip(py, 0, img_shape[0] - 1)

#         return (px, py)

#     def order_corners(self, pts):
#         """Orders the four corners as (top-left, top-right, bottom-right, bottom-left)."""
#         rect = np.zeros((4, 2), dtype="float32")
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)

#         rect[0] = pts[np.argmin(s)]  # Top-left
#         rect[2] = pts[np.argmax(s)]  # Bottom-right
#         rect[1] = pts[np.argmin(diff)]  # Top-right
#         rect[3] = pts[np.argmax(diff)]  # Bottom-left

#         return rect

#     def image_callback(self, msg):
#         """Callback function for processing live camera feed"""
#         if not self.running:
#             return

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#             # Detect rectangle and show its corners
#             processed_image = self.detect_rectangle(cv_image)

#             # Publish the processed image
#             if self.image_pub.get_num_connections() > 0:
#                 try:
#                     processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
#                     self.image_pub.publish(processed_msg)
#                 except CvBridgeError as e:
#                     rospy.logerr(f"Failed to convert image: {e}")

#             # Display the processed image in OpenCV window
#             cv2.imshow("Rectangle Detection", processed_image)
#             key = cv2.waitKey(1)

#             if key == 27:  # Press 'ESC' to stop
#                 self.shutdown(None, None)

#         except Exception as e:
#             rospy.logerr(f"Failed to process image: {e}")

#     def shutdown(self, signum, frame):
#         """Handle shutdown properly"""
#         rospy.loginfo("Shutting down rectangle detection node.")
#         self.running = False
#         self.image_sub.unregister()
#         cv2.destroyAllWindows()
#         rospy.signal_shutdown("User Interruption")
#         sys.exit(0)

#     def run(self):
#         """Main function to process the images"""
#         rospy.spin()


# if __name__ == '__main__':
#     try:
#         detector = Gen3RectangleDetection()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node interrupted.")

#!/usr/bin/env python

#!/usr/bin/env python
#ORB
# import rospy
# import actionlib
# import cv2
# import numpy as np
# import signal
# import sys
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class Gen3RectangleDetection:
#     def __init__(self):
#         rospy.init_node('gen3_rectangle_detection', anonymous=True)

#         # Handle shutdown signals
#         signal.signal(signal.SIGINT, self.shutdown)

#         # Initialize action client for robot movement
#         self.action_client = actionlib.SimpleActionClient(
#             '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )

#         rospy.loginfo("Waiting for action server...")
#         self.action_client.wait_for_server()
#         rospy.loginfo("Action server connected!")

#         # Initialize CV bridge for image conversion
#         self.bridge = CvBridge()

#         # Subscribe to the camera topic
#         self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

#         # Publisher for processed image
#         self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

#         self.orb = cv2.ORB_create(nfeatures=500)  # ORB feature detector
#         self.running = True  # Control flag

#     def move_robot(self):
#         """Moves the robot to a predefined position"""
#         target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

#         trajectory = JointTrajectory()
#         trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

#         point = JointTrajectoryPoint()
#         point.positions = target_joints
#         point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

#         trajectory.points.append(point)

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory = trajectory
#         self.action_client.send_goal(goal)

#         rospy.loginfo("Sent goal to move the arm")
#         self.action_client.wait_for_result()
#         rospy.loginfo("Motion completed!")

#     def detect_rectangle_orb(self, frame):
#         """Detects keypoints using ORB and filters for rectangle corners."""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
#         # Detect ORB keypoints and descriptors
#         keypoints, descriptors = self.orb.detectAndCompute(gray, None)

#         if len(keypoints) < 4:
#             rospy.logwarn("Not enough keypoints detected!")
#             return frame

#         # Convert keypoints to (x, y) array
#         pts = np.array([kp.pt for kp in keypoints], dtype=np.float32)

#         # Convex hull to find boundary points
#         hull = cv2.convexHull(pts)

#         if len(hull) >= 4:
#             ordered_corners = self.order_corners(hull.reshape(-1, 2))

#             # Draw detected corners
#             for (x, y) in ordered_corners:
#                 cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)

#             print("\nDetected Rectangle Corners (u, v) pixels:")
#             for i, (u, v) in enumerate(ordered_corners):
#                 print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

#         # Draw keypoints for visualization
#         frame = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
#         return frame

#     def order_corners(self, pts):
#         """ Orders the four corners as (top-left, top-right, bottom-right, bottom-left) """
#         rect = np.zeros((4, 2), dtype="float32")
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)

#         rect[0] = pts[np.argmin(s)]  # Top-left
#         rect[2] = pts[np.argmax(s)]  # Bottom-right
#         rect[1] = pts[np.argmin(diff)]  # Top-right
#         rect[3] = pts[np.argmax(diff)]  # Bottom-left

#         return rect

#     def image_callback(self, msg):
#         """Callback function for processing live camera feed"""
#         if not self.running:
#             return

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#             # Detect rectangle and show its corners
#             processed_image = self.detect_rectangle_orb(cv_image)

#             # Publish the processed image
#             try:
#                 processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
#                 self.image_pub.publish(processed_msg)
#             except CvBridgeError as e:
#                 rospy.logerr(f"Failed to convert image: {e}")

#             # Display the processed image in OpenCV window
#             cv2.imshow("ORB Rectangle Detection", processed_image)
#             key = cv2.waitKey(1)

#             if key == 27:  # Press 'ESC' to stop
#                 self.shutdown(None, None)

#         except Exception as e:
#             rospy.logerr(f"Failed to process image: {e}")

#     def shutdown(self, signum, frame):
#         """Handle shutdown properly"""
#         rospy.loginfo("Shutting down rectangle detection node.")
#         self.running = False
#         self.image_sub.unregister()
#         cv2.destroyAllWindows()
#         rospy.signal_shutdown("User Interruption")
#         sys.exit(0)

#     def run(self):
#         """Main function to move robot and detect rectangle"""
#         self.move_robot()
#         rospy.spin()


# if __name__ == '__main__':
#     try:
#         detector = Gen3RectangleDetection()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node interrupted.")


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

        # Initialize action client for robot movement
        self.action_client = actionlib.SimpleActionClient(
            '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for action server...")
        self.action_client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Initialize ORB detector
        self.orb = cv2.ORB_create(nfeatures=250)

        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

        # Publisher for processed image
        self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

        self.running = True  # Control flag

    def move_robot(self):
        """Moves the robot to a predefined position"""
        target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

        trajectory = JointTrajectory()
        trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

        trajectory.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self.action_client.send_goal(goal)

        rospy.loginfo("Sent goal to move the arm")
        self.action_client.wait_for_result()
        rospy.loginfo("Motion completed!")

    def detect_rectangle_orb(self, frame):
        """Detects keypoints using ORB and extracts exactly 4 rectangle corners."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ORB keypoints
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        if len(keypoints) < 4:
            rospy.logwarn("Not enough keypoints detected!")
            return frame

        # Convert keypoints to (x, y) array
        pts = np.array([kp.pt for kp in keypoints], dtype=np.float32)

        # Compute Convex Hull to find boundary points
        hull = cv2.convexHull(pts)

        if len(hull) >= 4:
            ordered_corners = self.extract_four_corners(hull.reshape(-1, 2))

            # Draw detected corners
            for (x, y) in ordered_corners:
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

            print("\nDetected Rectangle Corners (u, v) pixels:")
            for i, (u, v) in enumerate(ordered_corners):
                print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

        # Draw keypoints for visualization
        frame = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
        return frame

    def extract_four_corners(self, pts):
        """Filters only the four rectangle corners from the detected points."""
        if len(pts) < 4:
            rospy.logwarn("Not enough points detected!")
            return pts  # Return available points if less than 4

        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)

        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left

        return rect

    def image_callback(self, msg):
        """Callback function for processing live camera feed"""
        if not self.running:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Detect rectangle and show its corners
            processed_image = self.detect_rectangle_orb(cv_image)

            # Publish the processed image
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
                self.image_pub.publish(processed_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert image: {e}")

            # Display the processed image in OpenCV window
            cv2.imshow("ORB Rectangle Detection", processed_image)
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
        """Main function to move robot and detect rectangle"""
        self.move_robot()
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = Gen3RectangleDetection()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")

#ORB+RANSAC

#!/usr/bin/env python

# import rospy
# import actionlib
# import cv2
# import numpy as np
# import signal
# import sys
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class Gen3RectangleDetection:
#     def __init__(self):
#         rospy.init_node('gen3_rectangle_detection', anonymous=True)

#         # Handle shutdown signals
#         signal.signal(signal.SIGINT, self.shutdown)

#         # Initialize action client for robot movement
#         self.action_client = actionlib.SimpleActionClient(
#             '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )

#         rospy.loginfo("Waiting for action server...")
#         self.action_client.wait_for_server()
#         rospy.loginfo("Action server connected!")

#         # Initialize ORB detector
#         self.orb = cv2.ORB_create(nfeatures=500)

#         # Initialize feature matcher
#         self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#         # Initialize CV bridge for image conversion
#         self.bridge = CvBridge()

#         # Subscribe to the camera topic
#         self.image_sub = rospy.Subscriber("/my_gen3/rrbot/camera1/image_raw", Image, self.image_callback)

#         # Publisher for processed image
#         self.image_pub = rospy.Publisher("/edge_detected_image", Image, queue_size=1)

#         # Load reference image of the rectangle (assumed to be pre-saved)
#         self.reference_image = cv2.imread("/home/pc/Desktop/reference_rectangle.jpg", cv2.IMREAD_GRAYSCALE)
#         if self.reference_image is None:
#             rospy.logerr("Reference image not found! Please provide a template.")

#         # Extract keypoints and descriptors from reference image
#         self.ref_keypoints, self.ref_descriptors = self.orb.detectAndCompute(self.reference_image, None)

#         self.running = True  # Control flag

#     def move_robot(self):
#         """Moves the robot to a predefined position"""
#         target_joints = [-0.011, 0.822, 3.142, -0.410, -0.008, -1.750, 1.572]  # Example configuration

#         trajectory = JointTrajectory()
#         trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

#         point = JointTrajectoryPoint()
#         point.positions = target_joints
#         point.time_from_start = rospy.Duration(5.0)  # Move in 5 seconds

#         trajectory.points.append(point)

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory = trajectory
#         self.action_client.send_goal(goal)

#         rospy.loginfo("Sent goal to move the arm")
#         self.action_client.wait_for_result()
#         rospy.loginfo("Motion completed!")

#     def detect_rectangle_ransac(self, frame):
#         """Detects a rectangle using ORB + RANSAC-based homography."""
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Detect ORB keypoints in the current frame
#         keypoints, descriptors = self.orb.detectAndCompute(gray, None)

#         if descriptors is None or self.ref_descriptors is None:
#             rospy.logwarn("Descriptors missing!")
#             return frame

#         # Match features between reference image and current frame
#         matches = self.bf.match(self.ref_descriptors, descriptors)
#         matches = sorted(matches, key=lambda x: x.distance)  # Sort matches by distance

#         if len(matches) < 4:
#             rospy.logwarn("Not enough good matches for RANSAC!")
#             return frame

#         # Extract matched keypoints
#         ref_pts = np.float32([self.ref_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
#         frame_pts = np.float32([keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

#         # Compute Homography using RANSAC
#         H, mask = cv2.findHomography(ref_pts, frame_pts, cv2.RANSAC, 5.0)

#         if H is not None:
#             # Get the four corners of the reference rectangle
#             h, w = self.reference_image.shape
#             ref_corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
#             detected_corners = cv2.perspectiveTransform(ref_corners, H)

#             # Convert to integer coordinates
#             detected_corners = detected_corners.reshape(-1, 2)
#             ordered_corners = self.order_corners(detected_corners)

#             # Draw the rectangle
#             for i in range(4):
#                 cv2.circle(frame, tuple(ordered_corners[i].astype(int)), 5, (0, 0, 255), -1)
#             cv2.polylines(frame, [ordered_corners.astype(int)], isClosed=True, color=(255, 0, 0), thickness=3)

#             print("\nDetected Rectangle Corners (u, v) pixels:")
#             for i, (u, v) in enumerate(ordered_corners):
#                 print(f"Corner {i+1}: ({u:.2f}, {v:.2f})")

#         return frame

#     def order_corners(self, pts):
#         """Orders detected rectangle corners in top-left, top-right, bottom-right, bottom-left order."""
#         rect = np.zeros((4, 2), dtype="float32")
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)

#         rect[0] = pts[np.argmin(s)]  # Top-left
#         rect[1] = pts[np.argmin(diff)]  # Top-right
#         rect[2] = pts[np.argmax(s)]  # Bottom-right
#         rect[3] = pts[np.argmax(diff)]  # Bottom-left

#         return rect

#     def image_callback(self, msg):
#         """Callback function for processing live camera feed"""
#         if not self.running:
#             return

#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#             # Detect rectangle using ORB + RANSAC
#             processed_image = self.detect_rectangle_ransac(cv_image)

#             # Publish the processed image
#             try:
#                 processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
#                 self.image_pub.publish(processed_msg)
#             except CvBridgeError as e:
#                 rospy.logerr(f"Failed to convert image: {e}")

#             # Display the processed image in OpenCV window
#             cv2.imshow("ORB + RANSAC Rectangle Detection", processed_image)
#             key = cv2.waitKey(1)

#             if key == 27:  # Press 'ESC' to stop
#                 self.shutdown(None, None)

#         except Exception as e:
#             rospy.logerr(f"Failed to process image: {e}")

#     def shutdown(self, signum, frame):
#         """Handle shutdown properly"""
#         rospy.loginfo("Shutting down rectangle detection node.")
#         self.running = False
#         self.image_sub.unregister()
#         cv2.destroyAllWindows()
#         rospy.signal_shutdown("User Interruption")
#         sys.exit(0)

#     def run(self):
#         """Main function to move robot and detect rectangle"""
#         self.move_robot()
#         rospy.spin()


# if __name__ == '__main__':
#     try:
#         detector = Gen3RectangleDetection()
#         detector.run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Node interrupted.")


