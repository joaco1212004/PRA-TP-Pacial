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

        # -------------------------------------------------------------
        # Estado interno del filtro EKF.
        # -------------------------------------------------------------

        # Vector de estado estimado: [x, y, theta].
        # Se inicializa cuando el usuario selecciona una pose en RViz.
        self.mu = None

        # Matriz de covarianza del estado estimado.
        self.Sigma = None

        # Covarianza del ruido del modelo de movimiento.
        self.Q = np.diag([0.02, 0.02, 0.02])

        # Suscripción a la posición inicial seleccionada en RViz.
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10
        )

        # Última posición agregada a la trayectoria estimada.
        self.last_ekf_xy = None
    
    def initial_pose_callback(self, data: PoseWithCovarianceStamped):
        """
        Inicializa el belief del EKF utilizando la pose seleccionada
        en RViz mediante la herramienta 2D Pose Estimate.
        """

        # Obtener la posición inicial.
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # Convertir el cuaternión recibido a orientación yaw.
        q = data.pose.pose.orientation

        theta = R.from_quat(
            [q.x, q.y, q.z, q.w]
        ).as_euler("xyz")[2]

        # Inicializar el vector de estado.
        self.mu = np.array(
            [x, y, theta],
            dtype=float
        )

        # Incertidumbre inicial:
        # 0.5 metros en x e y, y 10 grados en orientación.
        #
        # La matriz de covarianza contiene varianzas, por eso
        # elevamos al cuadrado los desvíos estándar.
        self.Sigma = np.diag([
            0.5 ** 2,
            0.5 ** 2,
            np.deg2rad(10.0) ** 2
        ])

        # Reiniciar la trayectoria del EKF al seleccionar
        # una nueva posición inicial.
        self.ekf_path_msg = Path()
        self.ekf_path_msg.header.frame_id = "map"
        self.last_ekf_xy = None

        # Limpiar la trayectoria anterior en RViz.
        self.ekf_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.ekf_path_pub.publish(self.ekf_path_msg)

        # Publicar el belief inicial.
        self.publish_belief()

        self.get_logger().info(
            "EKF inicializado correctamente."
        )


    def publish_belief(self):
        """
        Publica la estimación del EKF y su matriz de covarianza.

        También actualiza la trayectoria correspondiente
        a la posición estimada por el filtro.
        """

        if self.mu is None or self.Sigma is None:
            return

        x, y, theta = self.mu

        # -------------------------------------------------------------
        # Publicar PoseWithCovarianceStamped.
        # -------------------------------------------------------------

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Posición estimada.
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Orientación estimada.
        msg.pose.pose.orientation = yaw_to_quaternion(
            float(theta)
        )

        # El mensaje ROS utiliza una matriz de covarianza 6x6:
        # [x, y, z, roll, pitch, yaw].
        #
        # Nuestro EKF trabaja únicamente con [x, y, yaw].
        # Copiamos sus covarianzas a las posiciones correspondientes.
        covariance = [0.0] * 36

        indices = [0, 1, 5]

        for i in range(3):
            for j in range(3):
                covariance[
                    indices[i] * 6 + indices[j]
                ] = float(self.Sigma[i, j])

        msg.pose.covariance = covariance

        self.pub_pose_with_cov.publish(msg)

        # -------------------------------------------------------------
        # Publicar trayectoria estimada por el EKF.
        # -------------------------------------------------------------

        current_xy = (float(x), float(y))

        if (
            self.last_ekf_xy is None
            or math.dist(current_xy, self.last_ekf_xy)
            >= self.path_min_dist
        ):

            pose_stamped = PoseStamped()

            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose

            self.ekf_path_msg.poses.append(pose_stamped)
            self.ekf_path_msg.header.stamp = msg.header.stamp

            self.last_ekf_xy = current_xy

            self.ekf_path_pub.publish(self.ekf_path_msg)

    def odom_callback(self, data: Odometry):
        """
        Ejecuta el paso de predicción del EKF utilizando
        los desplazamientos obtenidos mediante odometría.
        """

        # -------------------------------------------------------------
        # Publicación de la trayectoria calculada por odometría.
        # Se conserva la implementación original.
        # -------------------------------------------------------------

        pose_stamped = PoseStamped()

        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = data.pose.pose

        xy = (
            data.pose.pose.position.x,
            data.pose.pose.position.y
        )

        if (
            self.last_calc_xy is None
            or math.dist(xy, self.last_calc_xy)
            >= self.path_min_dist
        ):
            self.last_calc_xy = xy

            self.calc_path_msg.poses.append(pose_stamped)
            self.calc_path_msg.header.stamp = self.get_clock().now().to_msg()

            self.calc_path_pub.publish(self.calc_path_msg)

        # -------------------------------------------------------------
        # Obtener la posición actual de la odometría.
        # -------------------------------------------------------------

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        q = data.pose.pose.orientation

        theta = R.from_quat(
            [q.x, q.y, q.z, q.w]
        ).as_euler("xyz")[2]

        # -------------------------------------------------------------
        # Primera lectura de odometría.
        #
        # Se utiliza como referencia para calcular los
        # desplazamientos entre mediciones consecutivas.
        # -------------------------------------------------------------

        if not self.read_odom:

            self.last_odom = (x, y, theta)
            self.read_odom = True

            return

        # Recuperar la odometría anterior.
        prev_x, prev_y, prev_theta = self.last_odom

        # Actualizar la referencia para el siguiente callback.
        self.last_odom = (x, y, theta)

        # No ejecutar la predicción hasta que el usuario
        # haya inicializado el belief desde RViz.
        if self.mu is None:
            return

        # -------------------------------------------------------------
        # Calcular el desplazamiento observado.
        # -------------------------------------------------------------

        dx = x - prev_x
        dy = y - prev_y

        # Diferencia angular normalizada a [-pi, pi].
        delta_theta = math.atan2(
            math.sin(theta - prev_theta),
            math.cos(theta - prev_theta)
        )

        # Transformar el desplazamiento al sistema local
        # de referencia de la odometría anterior.
        delta_x_local = (
            math.cos(prev_theta) * dx
            + math.sin(prev_theta) * dy
        )

        delta_y_local = (
            -math.sin(prev_theta) * dx
            + math.cos(prev_theta) * dy
        )

        # Si no hubo desplazamiento apreciable,
        # no es necesario ejecutar una nueva predicción.
        if (
            abs(delta_x_local) < 1e-5
            and abs(delta_y_local) < 1e-5
            and abs(delta_theta) < 1e-5
        ):
            return

        # -------------------------------------------------------------
        # Predicción del estado.
        #
        # Aplicamos el desplazamiento local utilizando
        # la orientación estimada por el EKF.
        # -------------------------------------------------------------

        x_est, y_est, theta_est = self.mu

        new_x = (
            x_est
            + math.cos(theta_est) * delta_x_local
            - math.sin(theta_est) * delta_y_local
        )

        new_y = (
            y_est
            + math.sin(theta_est) * delta_x_local
            + math.cos(theta_est) * delta_y_local
        )

        new_theta = theta_est + delta_theta

        # Normalizar la orientación estimada.
        new_theta = math.atan2(
            math.sin(new_theta),
            math.cos(new_theta)
        )

        # -------------------------------------------------------------
        # Jacobiano Gt del modelo de movimiento.
        # -------------------------------------------------------------

        Gt = np.eye(3)

        Gt[0, 2] = (
            -math.sin(theta_est) * delta_x_local
            -math.cos(theta_est) * delta_y_local
        )

        Gt[1, 2] = (
            math.cos(theta_est) * delta_x_local
            -math.sin(theta_est) * delta_y_local
        )

        # -------------------------------------------------------------
        # Predicción de la covarianza.
        #
        # Sigma_t = Gt * Sigma_(t-1) * Gt.T + Qt
        # -------------------------------------------------------------

        self.Sigma = (
            Gt @ self.Sigma @ Gt.T
            + self.Q
        )

        # Mantener la simetría de la matriz de covarianza.
        self.Sigma = (
            self.Sigma + self.Sigma.T
        ) / 2.0

        # -------------------------------------------------------------
        # Actualizar el belief estimado.
        # -------------------------------------------------------------

        self.mu = np.array([
            new_x,
            new_y,
            new_theta
        ])

        # Publicar la posición y covarianza actualizadas.
        self.publish_belief()

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