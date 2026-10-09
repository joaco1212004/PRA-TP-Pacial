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
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
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

class Odom3Node(Node):
    def __init__(self):
        super().__init__("lidar_tf")

        # ATENCION: Se puede setear el numero de particulas desde el launch file
        # De esta manera podemos variar el numero de particulas sin tener que cambiar
        # el codigo cada vez. Por defecto se usan 1000.

        # TODO: Su codigo debera tomar este parametro y usarlo para inicializar el
        # filtro de particulas con la cantidad especificada de particulas.
        self.declare_parameter("num_particles", 1000)
        num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value


        self.subscription_odom = self.create_subscription(
            Odometry, "/calc_odom", self.odom_callback, 10
        )

        # QoS de sensor (BEST_EFFORT), no el perfil por defecto. El /scan del
        # rosbag del robot real (seccion 3) se grabo BEST_EFFORT, y un
        # subscriptor RELIABLE es incompatible con ese publicador: no recibe un
        # solo mensaje. ROS lo avisa una vez con un WARN de "incompatible QoS"
        # que se pierde entre la salida del launch. Este perfil recibe de los
        # dos: del bag y del /scan RELIABLE de Gazebo.
        self.subscription_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.likelyhood_map = self.create_subscription(
            OccupancyGrid, "/likelihood_map", self.map_callback, map_qos)

        self.map_received = False
        self.odom_received = False
        self.last_odom = (0.0, 0.0, 0.0)  # x, y, theta

        # Create a publisher for PointCloud2
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/particle_cloud", 10
        )

        self.declare_parameter("path_min_dist", 0.05)
        self.path_min_dist = self.get_parameter("path_min_dist").value
        self.last_calc_xy = None

        self.calc_path_pub = self.create_publisher(Path, "/calc_robot_path", 10)
        self.calc_path_msg = Path()
        self.calc_path_msg.header.frame_id = "map"

        self.particle_path_pub = self.create_publisher(Path, "/particle_robot_path", 10)
        self.particle_path_msg = Path()
        self.particle_path_msg.header.frame_id = "map"

    def map_callback(self, msg):
        if not self.map_received:
            self.map_data = msg
            self.map_received = True
            self.grid = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            self.get_logger().info('Map and inflated map processed.')


    def create_pointcloud2(self, points, intensities=None):
        """Create a PointCloud2 message from numpy array."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # or whatever your frame is

        # points is a numpy array of shape (N, 2) for x, y
        # we must add the z coordinate (set to 0) and intensity (if provided)

        points = np.hstack([points, np.zeros((points.shape[0], 1))])  # Add z=0
        
        # `intensities != None` sobre un array de numpy devuelve otro array, y
        # el `and` que sigue lo evalua como booleano: ValueError. Con `is not
        # None` la comprobacion vuelve a ser sobre el objeto.
        if intensities is not None and np.max(intensities) > 0:
            # sin /= : el operador in-place modificaria el array del llamador
            intensities = intensities / np.max(intensities)
            intensities = intensities.reshape(-1, 1).astype(np.float32)
            points_with_intensity = np.hstack([points.astype(np.float32), intensities])
        else:
            points_with_intensity = np.hstack([points.astype(np.float32), np.zeros((points.shape[0], 1), dtype=np.float32)])

        msg.height = 1
        msg.width = points_with_intensity.shape[0]

        
        # Define the point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 3 floats * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Convert points to bytes
        msg.data = points_with_intensity.tobytes()
        
        return msg


    def odom_callback(self, data: Odometry):
        if not self.odom_received:
            self.odom_received = True
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
    
            # Extract quaternion (w, x, y, z) format
            q_w = data.pose.pose.orientation.w
            q_x = data.pose.pose.orientation.x
            q_y = data.pose.pose.orientation.y
            q_z = data.pose.pose.orientation.z
            
            current_rotation = R.from_quat([q_x, q_y, q_z, q_w])
            theta = current_rotation.as_euler('xyz', degrees=False)[2]  # Extract yaw

            self.last_odom = (x, y, theta)
            return


        # PATH PUBLISHING #################################
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
        ####################################################

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # Extract quaternion (w, x, y, z) format
        q_w = data.pose.pose.orientation.w
        q_x = data.pose.pose.orientation.x
        q_y = data.pose.pose.orientation.y
        q_z = data.pose.pose.orientation.z
        
        current_rotation = R.from_quat([q_x, q_y, q_z, q_w])
        theta = current_rotation.as_euler('xyz', degrees=False)[2]  # Extract yaw

        # TODO: Implementar el modelo de movimiento basado en odometría para actualizar las partículas
        # Pueden graficar las particulas usando la funcion create_pointcloud2() que ya esta implementada en este archivo.
        # Esa funcion espera recibir las particulas como un arreglo de numpy de tamaño (N, 2) donde N es el numero de
        # particulas y cada particula tiene la forma (x, y). Tambien pueden pasar un arreglo de intensidades para
        # graficar las particulas con un color distinto segun su peso. Si lo desean tambien pueden graficar la
        # trayectoria de las particulas con el publicador particle_path_pub. Deberan elegir como seleccionar el punto
        # que representa la trayectoria de las particulas, por ejemplo pueden usar la particula con mayor peso o el
        # promedio de todas las particulas.


    def scan_callback(self, data):
        # TODO: Implementar el modelo de sensor basado en laser para actualizar los pesos de las particulas
        # Deberan asignarle un peso a cada particula basado en la probabilidad de que el laser haya medido 
        # lo que midio dado el mapa de verosimilitud. Luego deberan implementar el resampling de las particulas
        # basado en sus pesos. Nuevamente, pueden graficar las particulas usando la funcion create_pointcloud2()
        # que ya esta implementada y recibe particulas y pesos como arreglos de numpy.
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = Odom3Node()
    
    try:
        rclpy.spin(node)  # Keep ROS 2 running
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()