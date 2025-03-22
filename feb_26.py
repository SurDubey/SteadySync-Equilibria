# #!/usr/bin/env python3

# import sys
# import time
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String

# class MoveRobot:
#     def __init__(self):
#         super(MoveRobot, self).__init__()
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_robot_waypoints')

#         arm_group_name = "arm"
#         self.robot = moveit_commander.RobotCommander("robot_description")
#         self.scene = moveit_commander.PlanningSceneInterface()
#         self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
#         self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

#         rospy.loginfo("MoveIt Node Initialized")

#     def move_to_joint_positions(self, joint_positions):
#         self.arm_group.set_joint_value_target(joint_positions)
#         return self.arm_group.go(wait=True)

#     def save_image(self, file_path):
#         rospy.loginfo(f"Requesting image save: {file_path}")
#         self.image_save_pub.publish(file_path)

#     def execute_waypoints(self, waypoints, final_position, image_path):
#         for i, wp in enumerate(waypoints):
#             rospy.loginfo(f"Moving to Waypoint {i+1}...")
#             if not self.move_to_joint_positions(wp):
#                 rospy.logerr(f"Failed at Waypoint {i+1}")
#                 return False

#         rospy.loginfo("Reached Final Position")
#         if self.move_to_joint_positions(final_position):
#             rospy.sleep(5)
#             self.save_image(image_path)
#             return True
#         return False

# def main():
#     robot = MoveRobot()

#     home_position = [0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708]
    
#     waypoints_top = [
#         [0.0175, 0.1047, 3.1241, -1.7977, 0.0, -0.2793, 1.5708],
#         [0.0, 0.2618, 3.1241, -1.3439, 0.0, -0.9774, 1.5708],
#         [0.0, 0.4712, 3.1241, -0.8552, 0.0, -1.7802, 1.5708],
#         [0.0, 0.6283, 3.1241, -0.7330, 0.0, -1.7453, 1.5708],
#         [0.0, 0.8727, 3.1416, -0.2269, 0.0, -1.9897, 1.5708],
#         [0.0, 0.7505, 3.1241, -0.6458, 0.0175, -1.7104, 1.5708],
#         [0.0, 1.0123, 3.1416, -0.1047, 0.0, -2.0071, 1.5708],
#         [0.0, 0.8378, 3.1416, -0.2967, 0.0, -1.8849, 1.5708]
#     ]
#     top_position = [0.0, 0.6981, 3.1241, -0.5759, 0.0, -1.7279, 1.5708]

#     waypoints_front = [
#         [0.0, 0.0873, 3.1416, -2.5133, 0.0, 1.0297, 1.5708],
#         [0.0, 0.4363, 3.1416, -2.4086, 0.0, 1.2741, 1.5708],
#         [0.0, 0.6458, 3.1416, -2.4086, 0.0, 1.2741, 1.5708],
#         [0.0, 1.0821, 3.1416, -2.4260, 0.0, 1.9024, 1.5708],
#         [0.0, 1.0821, 3.1416, -2.4609, 0.0, 1.9024, 1.5708],
#         [0.0, 1.0297, 3.1416, -2.5133, 0.0, 1.8675, 1.5708],
#         [0.0, 0.9774, 3.1416, -2.5133, 0.0, 1.7977, 1.5708]
#     ]
#     front_position = [0.0, 0.9425, 3.1416, -2.5133, 0.0, 1.7802, 1.5708]

#     if robot.move_to_joint_positions(home_position):
#         if robot.execute_waypoints(waypoints_top, top_position, "/home/pc/pic/top.jpg"):
#             robot.move_to_joint_positions(home_position)
#             robot.execute_waypoints(waypoints_front, front_position, "/home/pc/pic/front.jpg")
#     else:
#         rospy.logerr("Failed to reach HOME position")

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
from std_msgs.msg import String

class MoveRobot:
    def __init__(self):
        super(MoveRobot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_robot_positions')

        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
        self.image_save_pub = rospy.Publisher('/save_image_topic', String, queue_size=10)

        rospy.loginfo("MoveIt Node Initialized")

    def move_to_joint_positions(self, joint_positions):
        self.arm_group.set_joint_value_target(joint_positions)
        return self.arm_group.go(wait=True)

    def save_image(self, file_path):
        if file_path:
            rospy.loginfo(f"Requesting image save: {file_path}")
            self.image_save_pub.publish(file_path)

    def execute_sequence(self, positions, image_paths):
        for position, image_path in zip(positions, image_paths):
            rospy.loginfo(f"Moving to position: {position}")
            if self.move_to_joint_positions(position):
                rospy.sleep(5)
                self.save_image(image_path)
            else:
                rospy.logerr("Failed to reach position")
                return False
        return True

def main():
    robot = MoveRobot()

    home_position = [0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708]
    top_position = [0.0, 0.6981, 3.1241, -0.5759, 0.0, -1.7279, 1.5708]
    front_position = [0.0, 0.9425, 3.1416, -2.5133, 0.0, 1.7802, 1.5708]

    positions = [home_position, top_position, home_position, front_position, home_position]
    image_paths = [None, "/home/pc/pic/top.jpg", None, "/home/pc/pic/front.jpg", None]

    robot.execute_sequence(positions, image_paths)

if __name__ == '__main__':
    main()


