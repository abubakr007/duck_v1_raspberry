#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, TwistStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import math
from tf_transformations import quaternion_matrix, quaternion_from_matrix, inverse_matrix, concatenate_matrices
#import tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_motion_planner_node")

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare and get parameters
        self.declare_parameter("look_ahead_distance", 0.5)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.look_ahead_distance = self.get_parameter("look_ahead_distance").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/duck_control/cmd_vel", 10)
        self.carrot_pose_pub = self.create_publisher(PoseStamped, "/pure_pursuit/carrot", 10)

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.global_plan = None

    def path_callback(self, path: Path):
        """
        Callback for the global path subscriber.
        Stores the path and updates its timestamp.
        """
        self.get_logger().info("Received a new path.")
        # Update the path header timestamp
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Update timestamps for each pose in the path for consistency
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = path.header.frame_id

        self.global_plan = path

    def control_loop(self):
        """
        Main control loop, executed at a fixed frequency.
        """
        if not self.global_plan or not self.global_plan.poses:
            # No path available, do nothing
            return

        # Get the robot's current pose in the odom frame.
        # Using rclpy.time.Time() (or time=0) gets the latest available transform.
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time())
        except Exception as ex:
            self.get_logger().warn(f"Could not transform from odom to base_footprint: {ex}")
            return

        # Transform the global plan into the robot's odometry frame for calculations
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform the global plan into the robot's frame.")
            return

        # Create a PoseStamped object for the robot's current position
        robot_pose = PoseStamped()
        robot_pose.header = robot_pose_transform.header
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        # Find the target "carrot" point on the path (this is in the 'odom' frame)
        carrot_pose_odom: PoseStamped = self.get_carrot_pose(robot_pose)

        # Calculate the distance to the carrot point
        dx = carrot_pose_odom.pose.position.x - robot_pose.pose.position.x
        dy = carrot_pose_odom.pose.position.y - robot_pose.pose.position.y
        distance_to_carrot = math.sqrt(dx ** 2 + dy ** 2)

        # Check if the goal has been reached
        if distance_to_carrot <= 0.15 and carrot_pose_odom == self.global_plan.poses[-1]:
            self.get_logger().info("Goal Reached!")
            self.global_plan.poses.clear()
            # Send a zero velocity command to stop the robot
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = 0.0
            cmd_vel.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_vel)
            return

        # --- KEY FIX IS HERE ---
        # Set the carrot's timestamp to match the timestamp of the robot's transform.
        # This ensures we ask TF2 for a transform at a time for which we know data exists.
        carrot_pose_odom.header.stamp = robot_pose_transform.header.stamp
        
        # Publish the carrot pose for visualization in RViz
        self.carrot_pose_pub.publish(carrot_pose_odom)

        # Transform the carrot pose from the 'odom' frame directly to the 'base_footprint' frame.
        try:
            carrot_pose_robot = self.tf_buffer.transform(
                carrot_pose_odom, "base_footprint", rclpy.duration.Duration(seconds=0.1))
        except Exception as ex:
            self.get_logger().error(f"Could not transform carrot pose to base_footprint: {ex}")
            return

        # Calculate curvature using the correctly transformed carrot pose
        curvature = self.get_curvature(carrot_pose_robot.pose)

        # --- Calculate and publish velocity command ---
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        
        # Linear velocity is scaled by distance to the carrot for smoother deceleration
        cmd_vel.twist.linear.x = min(self.max_linear_velocity, distance_to_carrot * 0.5)
        
        # Angular velocity is the product of linear velocity and curvature (v = wr, w = vk)
        cmd_vel.twist.angular.z =  curvature * cmd_vel.twist.linear.x
        
        # Clamp angular velocity to be safe
        cmd_vel.twist.angular.z =-1 * max(min(cmd_vel.twist.angular.z, self.max_angular_velocity), -self.max_angular_velocity)

        self.get_logger().info(f"Publishing cmd_vel: linear.x={cmd_vel.twist.linear.x:.2f}, angular.z={cmd_vel.twist.angular.z:.2f}", throttle_duration_sec=1)
        self.cmd_pub.publish(cmd_vel)



    def get_carrot_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        """
        Finds the "carrot" or lookahead point on the path.
        It first finds the closest point on the path to the robot and then
        finds the first point on the path that is at least the lookahead distance away.
        """
        # Default to the final goal point
        carrot_pose = self.global_plan.poses[-1]
        
        # Find the point on the path closest to the robot
        min_distance = float('inf')
        closest_index = 0
        
        for i, pose in enumerate(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # From the closest point, find the first point that is at or beyond the lookahead distance
        for i in range(closest_index, len(self.global_plan.poses)):
            pose = self.global_plan.poses[i]
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance >= self.look_ahead_distance:
                carrot_pose = pose
                break
        
        return carrot_pose

    def get_curvature(self, carrot_pose: Pose):
        """
        Calculates the curvature to the lookahead point.
        Assumes the carrot_pose is in the robot's coordinate frame.
        """
        carrot_dist_sq = carrot_pose.position.x ** 2 + carrot_pose.position.y ** 2
        if carrot_dist_sq > 0.001:  # Avoid division by zero
            curvature = 2.0 * carrot_pose.position.y / carrot_dist_sq
            # Clamp curvature to prevent extreme turning
            return max(min(curvature, 2.0), -2.0)
        else:
            return 0.0

    def transform_plan(self, target_frame: str) -> bool:
        """
        Transforms the entire global plan from its original frame to the target frame.
        """
        if self.global_plan.header.frame_id == target_frame:
            return True

        try:
            # Get the transform from the path's frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                target_frame, self.global_plan.header.frame_id, rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error(
                f"Couldn't transform plan from {self.global_plan.header.frame_id} to {target_frame}: {ex}")
            return False
        
        # Build a 4x4 transformation matrix from the transform message
        transform_matrix = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y
        transform_matrix[2][3] = transform.transform.translation.z

        # Transform each pose in the path
        for pose in self.global_plan.poses:
            # Build a 4x4 matrix for the pose
            pose_matrix = quaternion_matrix([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y
            pose_matrix[2][3] = pose.pose.position.z

            # Apply the transform: result = transform * pose
            transformed_pose_matrix = concatenate_matrices(transform_matrix, pose_matrix)
            
            # Extract the transformed pose back into the PoseStamped message
            quat = quaternion_from_matrix(transformed_pose_matrix)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            pose.pose.position.x = transformed_pose_matrix[0][3]
            pose.pose.position.y = transformed_pose_matrix[1][3]
            pose.pose.position.z = transformed_pose_matrix[2][3]
            
            # Update the header for the transformed pose
            pose.header.frame_id = target_frame

        # Update the path's header
        self.global_plan.header.frame_id = target_frame
        return True


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()