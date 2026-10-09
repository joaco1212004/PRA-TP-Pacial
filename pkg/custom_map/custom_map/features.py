import cv2
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class FeatureExtractor(Node):

  def __init__(self):
    super().__init__('feature_extractor')

    # Transient Local QoS profile required to receive latched map messages
    map_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    self.subscription = self.create_subscription(
        OccupancyGrid, '/map', self.map_callback, map_qos
    )
    self.publisher = self.create_publisher(PoseArray, '/landmarks', map_qos)

    self.get_logger().info(
        'FeatureExtractor node active. Waiting for /map...'
    )

  def map_callback(self, msg: OccupancyGrid):
    self.get_logger().info(
        f'Received map: {msg.info.width}x{msg.info.height} @'
        f' {msg.info.resolution}m/cell'
    )

    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))

    # Occupied cells (> 50) as white (255), free/unknown as black (0)
    img = np.uint8((data > 50) * 255)

    corners = cv2.goodFeaturesToTrack(
        img, maxCorners=30, qualityLevel=0.01, minDistance=20
    )

    pose_array = PoseArray()
    pose_array.header = msg.header

    if corners is not None:
      for c in corners:
        x, y = c.ravel()
        wx = msg.info.origin.position.x + x * msg.info.resolution
        wy = msg.info.origin.position.y + y * msg.info.resolution

        p = Pose()
        p.position.x = float(wx)
        p.position.y = float(wy)
        pose_array.poses.append(p)

      self.publisher.publish(pose_array)
      self.get_logger().info(
          f'Extracted and published {len(pose_array.poses)} landmarks to'
          ' /landmarks'
      )
    else:
      self.get_logger().warn('Map received, but 0 features were detected.')


def main(args=None):
  rclpy.init(args=args)
  node = FeatureExtractor()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()