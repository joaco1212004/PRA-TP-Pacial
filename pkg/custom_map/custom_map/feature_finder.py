import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PointStamped, Quaternion
import math
import numpy as np
import tf2_ros
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

def do_transform_point(point_stamped, transform_stamped):
    # Extract translation
    t = transform_stamped.transform.translation
    translation = np.array([t.x, t.y, t.z])

    # Extract yaw angle from quaternion (assuming only Z rotation matters)
    q = transform_stamped.transform.rotation
    # yaw = atan2(2*(wz), 1 - 2*z^2)
    yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    # Build 2D rotation matrix
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rot_matrix = np.array([
        [cos_yaw, -sin_yaw, 0.0],
        [sin_yaw,  cos_yaw, 0.0],
        [0.0,      0.0,     1.0]
    ])

    # Point to numpy
    p = np.array([point_stamped.point.x,
                  point_stamped.point.y,
                  point_stamped.point.z])

    # Apply transform
    p_transformed = rot_matrix.dot(p) + translation

    # Fill result
    result = PointStamped()
    result.header = transform_stamped.header
    result.point.x, result.point.y, result.point.z = p_transformed
    return result

class LandmarkObserver(Node):

  def __init__(self):
    super().__init__('landmark_observer')

    self.declare_parameter('fov', 180.0)

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    landmarks_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    self.sub_scan = self.create_subscription(
        LaserScan, '/scan', self.scan_callback, 10
    )
    self.sub_map = self.create_subscription(
        PoseArray, '/landmarks', self.map_callback, landmarks_qos
    )
    self.pub_obs = self.create_publisher(PoseArray, '/observed_landmarks', 10)

    self.map_landmarks = None
    self.get_logger().info(
        'LandmarkObserver node active. Waiting for /landmarks...'
    )

  def map_callback(self, msg: PoseArray):
    self.map_landmarks = msg
    self.get_logger().info(
        f'Received {len(msg.poses)} landmarks from /landmarks'
    )

  def scan_callback(self, msg: LaserScan):
    if self.map_landmarks is None or len(self.map_landmarks.poses) == 0:
      return

    try:
      transform = self.tf_buffer.lookup_transform(
          target_frame='base_footprint',
          source_frame=self.map_landmarks.header.frame_id,
          time=rclpy.time.Time(),
      )
    except Exception:
      return

    obs_array = PoseArray()
    obs_array.header = msg.header
    obs_array.header.frame_id = 'base_footprint'

    # Retrieve FOV parameter and calculate angular bounds centered at forward (+X)
    fov_deg = self.get_parameter('fov').get_parameter_value().double_value
    half_fov = math.radians(fov_deg) / 2.0

    # Effective angle range bounded by both FOV parameter and LiDAR scan range
    fov_min = max(msg.angle_min, -half_fov)
    fov_max = min(msg.angle_max, half_fov)
    r_min = msg.range_min
    r_max = msg.range_max

    detected_poses = []

    for landmark in self.map_landmarks.poses:
      pt = PointStamped()
      pt.header = self.map_landmarks.header
      pt.point = landmark.position

      pt_robot = do_transform_point(pt, transform)
      lx = pt_robot.point.x
      ly = pt_robot.point.y

      r_landmark = math.hypot(lx, ly)
      theta_landmark = math.atan2(ly, lx)

      if fov_min <= theta_landmark <= fov_max and r_min <= r_landmark <= r_max:
        # theta_landmark esta en base_footprint, pero msg.ranges esta en la
        # terna del laser, que cuelga de base_link rotada 180 grados en yaw (ver
        # rosmaster_x3.urdf.xacro, y el /tf_static del robot real). Indexar el
        # scan con el angulo de base_footprint valida cada landmark del frente
        # contra el rayo que apunta hacia atras: se pierde el 93% de las
        # detecciones y algunas de las que pasan son de landmarks ocluidos.
        theta_scan = math.atan2(math.sin(theta_landmark - math.pi),
                                math.cos(theta_landmark - math.pi))
        idx = int((theta_scan - msg.angle_min) / msg.angle_increment)
        idx = np.clip(idx, 0, len(msg.ranges) - 1)

        neighbor_idxs = range(
            max(0, idx - 2), min(len(msg.ranges), idx + 3)
        )
        r_measured = np.array([msg.ranges[i] for i in neighbor_idxs])
        r_measured = r_measured[np.isfinite(r_measured)]

        if (
            len(r_measured) > 0
            and np.min(np.abs(r_measured - r_landmark)) < 0.2
        ):
          # Noise parameters
            range_noise_ratio = 0.01  # 2% of distance
            angle_noise_std = math.radians(3.0)  # 1 degree angular error

            # Apply range-dependent noise in polar coordinates
            r_noisy = r_landmark + np.random.normal(0, range_noise_ratio * r_landmark)
            theta_noisy = theta_landmark + np.random.normal(0, angle_noise_std)

            # Reproject to Cartesian coordinates in base_footprint frame
            pose = Pose()
            pose.position.x = float(r_noisy * math.cos(theta_noisy))
            pose.position.y = float(r_noisy * math.sin(theta_noisy))
            pose.position.z = 0.0
            detected_poses.append(pose)

    # Fill start of array with observed detections
    obs_array.poses = detected_poses

    # Pad remaining indices with 0-filled Pose objects up to total map landmarks
    total_landmarks = len(self.map_landmarks.poses)
    missing_count = total_landmarks - len(detected_poses)
    for _ in range(missing_count):
      obs_array.poses.append(Pose())

    self.pub_obs.publish(obs_array)

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
