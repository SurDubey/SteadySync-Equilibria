#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import RobotCommander

class JointVelocityController:
    def __init__(self):
        rospy.init_node("joint_velocity_controller", anonymous=True)

        # MoveIt setup
        self.robot = RobotCommander()
        self.arm_group = self.robot.get_group("arm")

        # Subscribe to end-effector twist velocity
        rospy.Subscriber("/ee_twist", Float32MultiArray, self.ee_twist_callback)

        # Publisher to send joint velocity commands
        self.pub_joint_traj = rospy.Publisher(
            "/my_gen3/gen3_joint_trajectory_controller/command",
            JointTrajectory,
            queue_size=10
        )

        # Define joint names for Kinova Gen3
        self.joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6", "joint_7"
        ]

        # Joint velocity limits (modify as per Kinova's spec)
        self.joint_velocity_limit = 1.5  # rad/s (example safe limit)

        # Initialize variables
        self.latest_ee_twist = np.zeros((6, 1))
        self.running = True

        # Start control loop
        rospy.loginfo("Joint Velocity Controller Node Started")
        self.control_loop()

    def compute_jacobian_pinv(self):
        """Computes and caches the pseudo-inverse of the Jacobian matrix."""
        try:
            # Get current joint positions
            joint_positions = self.arm_group.get_current_joint_values()

            # Compute Jacobian (6x7)
            J = np.array(self.robot.compute_jacobian(joint_positions)).reshape(6, 7)

            # Compute pseudo-inverse (Moore-Penrose)
            J_pinv = np.linalg.pinv(J)

            return J_pinv
        except Exception as e:
            rospy.logerr(f"Jacobian computation failed: {e}")
            return None

    def ee_twist_callback(self, msg):
        """Receives EE velocity and stores it for processing in the control loop."""
        self.latest_ee_twist = np.array(msg.data).reshape(6, 1)  # Store latest twist

    def enforce_velocity_limits(self, joint_velocities):
        """Clamps joint velocities to safe limits."""
        return np.clip(joint_velocities, -self.joint_velocity_limit, self.joint_velocity_limit)

    def control_loop(self):
        """Continuously sends joint velocity commands at a fixed frequency."""
        rate = rospy.Rate(100)  # Control loop at 100 Hz (10ms update)

        while not rospy.is_shutdown() and self.running:
            J_pinv = self.compute_jacobian_pinv()
            if J_pinv is None:
                rospy.logwarn("Jacobian is invalid, skipping this cycle")
                rate.sleep()
                continue  # Skip this iteration

            # Compute joint velocities
            joint_velocities = np.dot(J_pinv, self.latest_ee_twist).flatten()

            # Apply velocity limits
            joint_velocities = self.enforce_velocity_limits(joint_velocities)

            # Publish joint velocities
            self.publish_joint_velocities(joint_velocities)

            rate.sleep()  # Maintain loop frequency

    def publish_joint_velocities(self, joint_velocities):
        """Publishes joint velocities to the robot."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.velocities = joint_velocities.tolist()
        point.time_from_start = rospy.Duration(0.1)  # Smooth updates every 100ms

        traj_msg.points.append(point)

        self.pub_joint_traj.publish(traj_msg)
        rospy.loginfo_throttle(1.0, f"Sent Joint Velocity Command: {joint_velocities}")

if __name__ == "__main__":
    try:
        node = JointVelocityController()
    except rospy.ROSInterruptException:
        pass

