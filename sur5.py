# #!/usr/bin/env python3

# import sys
# import time
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String

# from kortex_driver.srv import ExecuteActionRequest, ReadAction, ExecuteAction, ReadActionRequest
# from kortex_driver.msg import ActionEvent

# class ExampleMoveItTrajectories:
#     """ExampleMoveItTrajectories"""
#     def __init__(self):
#         super(ExampleMoveItTrajectories, self).__init__()
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('example_move_it_trajectories')

#         try:
#             self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
#             self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
#             self.robot_name = rospy.get_param('~robot_name', "kinova_arm")

#             arm_group_name = "arm"
#             self.robot = moveit_commander.RobotCommander("robot_description")
#             self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
#             self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, '/kinova_arm/robot_description', ns='/kinova_arm')

#             self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

#             # Kinova home action services
#             self.HOME_ACTION_IDENTIFIER = 2
#             read_action_full_name = f'/{self.robot_name}/base/read_action'
#             execute_action_full_name = f'/{self.robot_name}/base/execute_action'

#             rospy.wait_for_service(read_action_full_name)
#             rospy.wait_for_service(execute_action_full_name)

#             self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
#             self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

#             rospy.loginfo("Node initialized successfully.")

#         except Exception as e:
#             rospy.logerr(f"Initialization error: {e}")
#             self.is_init_success = False
#         else:
#             self.is_init_success = True

#     def move_to_home(self):
#         """Moves the robot to the predefined home position using Kinova's built-in action."""
#         self.last_action_notif_type = None
#         req = ReadActionRequest()

#         req.input.identifier = self.HOME_ACTION_IDENTIFIER
        
#         try:
#             res = self.read_action(req)
#         except rospy.ServiceException:
#             rospy.logerr("Failed to call ReadAction for homing")
#             return False

#         rospy.loginfo("Sending the robot home...")
#         try:
#             req_exec = ExecuteActionRequest()
#             req_exec.input = res.output
#             self.execute_action(req_exec)
#         except rospy.ServiceException:
#             rospy.logerr("Failed to call ExecuteAction for homing")
#             return False

#         rospy.sleep(5)  # Wait for the action to complete
#         rospy.loginfo("Robot reached home position.")
#         return True

#     def follow_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
#         """Moves the robot along a Cartesian path defined by waypoints."""
#         (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
#         if fraction == 1.0:
#             rospy.loginfo("Successfully computed Cartesian path")
#             return self.arm_group.execute(plan, wait=True)
#         else:
#             rospy.logerr("Failed to compute the full Cartesian path")
#             return False

#     def save_image(self, file_path):
#         """Publishes a request to save an image"""
#         rospy.loginfo(f"Requesting image save: {file_path}")
#         self.image_save_pub.publish(file_path)

# def main():
#     example = ExampleMoveItTrajectories()
#     success = example.is_init_success

#     if success:
#         # Move to home position first using Kinova's API
#         success &= example.move_to_home()

#         if not success:
#             rospy.logerr("Aborting since home position was not reached.")
#             return

#         # Define main target locations
#         locations = [
#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.5882, -0.3101, 0.1821),
#                                    orientation=geometry_msgs.msg.Quaternion(0.5711, 0.5473, 0.3788, 0.4805)),

#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.5951, 0.0828, 0.2202),
#                                    orientation=geometry_msgs.msg.Quaternion(0.5785, 0.5469, 0.3968, 0.4570)),

#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.7028, 0.4135, 0.1986),
#                                    orientation=geometry_msgs.msg.Quaternion(0.4849, 0.5998, 0.5073, 0.3844)),
#         ]

#         for i in range(len(locations) - 1):
#             start_pose = locations[i]
#             end_pose = locations[i + 1]

#             waypoints = []

#             mid_pose = geometry_msgs.msg.Pose()
#             mid_pose.position.x = (start_pose.position.x + end_pose.position.x) / 2
#             mid_pose.position.y = (start_pose.position.y + end_pose.position.y) / 2
#             mid_pose.position.z = max(start_pose.position.z, end_pose.position.z) + 0.1  
#             mid_pose.orientation = start_pose.orientation  

#             waypoints.append(start_pose)
#             waypoints.append(mid_pose)
#             waypoints.append(end_pose)

#             rospy.loginfo(f"Executing Cartesian path from Location {i+1} to {i+2}...")
#             success &= example.follow_cartesian_path(waypoints)

#             if success:
#                 rospy.loginfo(f"Successfully reached Location {i+2}")
#                 file_path = f"/home/pc/pic/location_{i+2}.jpg"
#                 example.save_image(file_path)
#                 rospy.sleep(5)  # Wait for 5 seconds at the location
#             else:
#                 rospy.logerr(f"Failed to reach Location {i+2}")
#                 break

#     if not success:
#         rospy.logerr("The example encountered an error.")

# if __name__ == '__main__':
#     main()




# #!/usr/bin/env python3

# import sys
# import time
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String

# from kortex_driver.srv import ExecuteActionRequest, ReadAction, ExecuteAction, ReadActionRequest
# from kortex_driver.msg import ActionEvent

# class ExampleMoveItTrajectories:
#     """ExampleMoveItTrajectories"""
#     def __init__(self):
#         super(ExampleMoveItTrajectories, self).__init__()
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('example_move_it_trajectories')

#         try:
#             self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
#             self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
#             self.robot_name = rospy.get_param('~robot_name', "kinova_arm")

#             arm_group_name = "arm"
#             self.robot = moveit_commander.RobotCommander("robot_description")
#             self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
#             self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, '/kinova_arm/robot_description', ns='/kinova_arm')

#             self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

#             # Kinova home action services
#             self.HOME_ACTION_IDENTIFIER = 2
#             read_action_full_name = f'/{self.robot_name}/base/read_action'
#             execute_action_full_name = f'/{self.robot_name}/base/execute_action'

#             rospy.wait_for_service(read_action_full_name)
#             rospy.wait_for_service(execute_action_full_name)

#             self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
#             self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

#             rospy.loginfo("Node initialized successfully.")

#         except Exception as e:
#             rospy.logerr(f"Initialization error: {e}")
#             self.is_init_success = False
#         else:
#             self.is_init_success = True

#     def follow_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
#         """Moves the robot along a Cartesian path defined by waypoints."""
#         (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
#         if fraction == 1.0:
#             rospy.loginfo("Successfully computed Cartesian path")
#             return self.arm_group.execute(plan, wait=True)
#         else:
#             rospy.logerr("Failed to compute the full Cartesian path")
#             return False

#     def save_image(self, file_path):
#         """Publishes a request to save an image"""
#         rospy.loginfo(f"Requesting image save: {file_path}")
#         self.image_save_pub.publish(file_path)

# def main():
#     example = ExampleMoveItTrajectories()
#     success = example.is_init_success

#     if success:
#         # Define main target locations
#         locations = [
#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.5882, -0.3101, 0.1821),
#                                    orientation=geometry_msgs.msg.Quaternion(0.5711, 0.5473, 0.3788, 0.4805)),

#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.5951, 0.0828, 0.2202),
#                                    orientation=geometry_msgs.msg.Quaternion(0.5785, 0.5469, 0.3968, 0.4570)),

#             geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.7028, 0.4135, 0.1986),
#                                    orientation=geometry_msgs.msg.Quaternion(0.4849, 0.5998, 0.5073, 0.3844)),
#         ]

#         for i in range(len(locations) - 1):
#             start_pose = locations[i]
#             end_pose = locations[i + 1]

#             waypoints = []

#             mid_pose = geometry_msgs.msg.Pose()
#             mid_pose.position.x = (start_pose.position.x + end_pose.position.x) / 2
#             mid_pose.position.y = (start_pose.position.y + end_pose.position.y) / 2
#             mid_pose.position.z = max(start_pose.position.z, end_pose.position.z) + 0.1  
#             mid_pose.orientation = start_pose.orientation  

#             waypoints.append(start_pose)
#             waypoints.append(mid_pose)
#             waypoints.append(end_pose)

#             rospy.loginfo(f"Executing Cartesian path from Location {i+1} to {i+2}...")
#             success &= example.follow_cartesian_path(waypoints)

#             if success:
#                 rospy.loginfo(f"Successfully reached Location {i+2}")
#                 file_path = f"/home/pc/pic/location_{i+2}.jpg"
#                 example.save_image(file_path)
#                 rospy.sleep(5)  # Wait for 5 seconds at the location
#             else:
#                 rospy.logerr(f"Failed to reach Location {i+2}")
#                 break

#     if not success:
#         rospy.logerr("The example encountered an error.")

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String  # Import standard ROS message

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # MoveIt setup
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, '/kinova_arm/robot_description', ns='/kinova_arm')

            # Publisher for image saving requests
            self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

            rospy.loginfo("Node initialized in namespace " + rospy.get_namespace())

        except Exception as e:
            rospy.logerr(f"Initialization error: {e}")
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_cartesian_pose(self, pose, tolerance):
        """Moves the robot to the specified pose."""
        self.arm_group.set_goal_position_tolerance(tolerance)
        self.arm_group.set_pose_target(pose)
        rospy.loginfo("Moving to target pose...")
        return self.arm_group.go(wait=True)

    def save_image(self, file_path):
        """Publishes a request to save an image"""
        rospy.loginfo(f"Requesting image save: {file_path}")
        self.image_save_pub.publish(file_path)

def main():
    example = ExampleMoveItTrajectories()
    success = example.is_init_success

    if success:
        locations = [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, -0.3101, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.0828, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.4135, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
        ]

        waypoints_1_to_2 = [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, -0.3101, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, -0.2, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, -0.1, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.0828, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            )
        ]

        waypoints_2_to_3 = [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.0828, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.2, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.3, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            ),
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.5882, 0.4135, 0.1821),
                orientation=geometry_msgs.msg.Quaternion(0.5001, 0.5000, 0.4999, 0.4999)
            )
        ]

        # Moving through locations and waypoints
        all_locations = [
            (locations[0], waypoints_1_to_2),
            (locations[1], waypoints_2_to_3),
            (locations[2], [])
        ]

        for idx, (location, waypoints) in enumerate(all_locations):
            rospy.loginfo(f"Reaching Location {idx+1}...")
            success &= example.reach_cartesian_pose(location, tolerance=0.01)

            if success:
                rospy.loginfo(f"Successfully reached Location {idx+1}")
                file_path = f"/home/pc/pic/location_{idx+1}.jpg"
                example.save_image(file_path)
                rospy.sleep(5)  # Wait for 5 seconds at the location

                # If there are waypoints, go through them
                for waypoint in waypoints:
                    rospy.loginfo(f"Moving to waypoint...")
                    success &= example.reach_cartesian_pose(waypoint, tolerance=0.01)
                    if success:
                        rospy.loginfo(f"Successfully reached waypoint.")
                        rospy.sleep(5)  # Wait for 5 seconds at the waypoint
                    else:
                        rospy.logerr(f"Failed to reach waypoint.")
                        break
            else:
                rospy.logerr(f"Failed to reach Location {idx+1}")
                break

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()



