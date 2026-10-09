#!/usr/bin/env python3

import numpy as np


# Patch for deprecated numpy aliases
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'bool'):
    np.bool = bool

    
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from geometry_msgs.msg import Quaternion
import math

def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) into a Quaternion message."""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class EKFNode(Node):
    def __init__(self):
        super().__init__("EKFNode")


        self.subscription_odom = self.create_subscription(
            Odometry, "/calc_odom", self.odom_callback, 10
        )

        self.last_odom = (0,0,0)  # Store previous odometry state
        self.read_odom = False

        self.declare_parameter("path_min_dist", 0.05)
        self.path_min_dist = self.get_parameter("path_min_dist").value
        self.last_calc_xy = None

        self.calc_path_pub = self.create_publisher(Path, "/calc_robot_path", 10)
        self.calc_path_msg = Path()
        self.calc_path_msg.header.frame_id = "map"

        self.ekf_path_pub = self.create_publisher(Path, "/ekf_robot_path", 10)
        self.ekf_path_msg = Path()
        self.ekf_path_msg.header.frame_id = "map"

        self.pub_pose_with_cov = self.create_publisher(
            PoseWithCovarianceStamped, "/pose_with_covariance", 10
        )

    def odom_callback(self, data: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = data.pose.pose   # geometry_msgs/Pose

        # Un punto cada `path_min_dist` metros. Sin este filtro el Path crece a
        # 50 Hz y se republica entero en cada mensaje: se midieron 2.68 MB/s y
        # 12486 poses en dos minutos, con lo que RViz se arrastra y parece culpa
        # del filtro del alumno.
        xy = (data.pose.pose.position.x, data.pose.pose.position.y)
        if self.last_calc_xy is None or math.dist(xy, self.last_calc_xy) >= self.path_min_dist:
            self.last_calc_xy = xy
            self.calc_path_msg.poses.append(pose_stamped)
            self.calc_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.calc_path_pub.publish(self.calc_path_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    
    try:
        rclpy.spin(node)  # Keep ROS 2 running
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()