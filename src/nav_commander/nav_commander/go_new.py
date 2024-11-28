#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import tf_transformations


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.path_publisher = self.create_publisher(Path, '/plan', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/goal_points',
            self.pointcloud_callback,
            10
        )
        self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.navigator = BasicNavigator()

        self.odom_received = False

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_received = True

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to list of points
        target_points = []
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[1], point[0], point[2]
            target_points.append([x, y])

        self.get_logger().info(f'Received {len(target_points)} points from point cloud.')
        self.publish_trajectory(target_points)


    def get_quaternion_from_euler(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


    def publish_trajectory(self, target_points):
        

        if not self.odom_received: return

        path_msg = Path()

        # path_msg.header.frame_id = "base_link"
        # path_msg.header.stamp = self.get_clock().now().to_msg()

        path_poses = []

        # Convert to map frame

        position = np.array([self.odom.pose.pose.position.x, 
                            self.odom.pose.pose.position.y, 
                            self.odom.pose.pose.position.z])
        orientation = self.odom.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

        for pos in target_points:
            goal_in_base_link = np.array([pos[0], pos[1], 0.0])  
            goal_in_map_frame = np.dot(rotation_matrix, goal_in_base_link) + position
            pos[0] = goal_in_map_frame[0]
            pos[1] = goal_in_map_frame[1]

        target_points = target_points[::-1]
        # Iterate through target_points to create poses
        for i in range(1, len(target_points)):
            current_point = target_points[i]
            prev_point = target_points[i - 1]
            
            # Calculate the position
            x = current_point[0]
            y = current_point[1]
            z = 0.0

            print(x,y,z)

            # Calculate the yaw (tangent to the path)
            yaw = math.atan2(current_point[1] - prev_point[1], current_point[0] - prev_point[0])
            roll = 0.0
            pitch = 0.0
            qx, qy, qz, qw = self.get_quaternion_from_euler(roll, pitch, yaw)

            # Create and populate pose
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = z
            # pose.pose.orientation.w = qw
            # pose.pose.orientation.x = qx
            # pose.pose.orientation.y = qy
            # pose.pose.orientation.z = qz
            pose.pose.orientation.x = self.odom.pose.pose.orientation.x
            pose.pose.orientation.y = self.odom.pose.pose.orientation.y
            pose.pose.orientation.z = self.odom.pose.pose.orientation.z
            pose.pose.orientation.w = self.odom.pose.pose.orientation.w
            

            # Append to path
            path_msg.poses.append(pose)
            path_poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Published a new trajectory.')
        self.navigator.goThroughPoses(path_poses)
        while not self.navigator.isTaskComplete():
            print("navigating")
            self.odom_received = False
        



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = PathPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

