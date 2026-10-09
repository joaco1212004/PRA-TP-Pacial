#!/usr/bin/env python3
"""
Publica /real_robot_path: el camino VERDE que dibuja particulas.rviz.

La consigna define el exito de la seccion 1.5 como "el camino rojo sigue el
camino verde", y rviz/particulas.rviz tiene el display escuchando
/real_robot_path -- pero ningun nodo del paquete lo publicaba, asi que el verde
nunca aparecia y el criterio no se podia observar.

El ground truth de Gazebo sale por /ground_truth/odom. Ese topico existe en el
bring-up de Fortress; en Gazebo Classic (el backend de macOS) no hay plugin de
pose verdadera, asi que este nodo lo dice por pantalla en vez de quedarse mudo.
Con el rosbag del robot real tampoco hay ground truth, y es correcto que no lo
haya: es justamente lo que la seccion 3 pide comentar.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class GroundTruthPath(Node):

    def __init__(self):
        super().__init__('ground_truth_path')

        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        # Distancia minima entre puntos: sin esto el Path crece a 50 Hz y el
        # mensaje se vuelve enorme despues de un par de minutos.
        self.declare_parameter('path_min_dist', 0.05)
        self.declare_parameter('warn_period', 10.0)

        topic = self.get_parameter('ground_truth_topic').value
        self.path_min_dist = self.get_parameter('path_min_dist').value

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.last_xy = None
        self.received = 0

        self.pub = self.create_publisher(Path, '/real_robot_path', 10)
        self.create_subscription(Odometry, topic, self.truth_callback, 10)
        self.create_timer(self.get_parameter('warn_period').value, self.check)

        self.get_logger().info(f'Publicando /real_robot_path a partir de {topic}')

    def truth_callback(self, msg: Odometry):
        self.received += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None and math.dist((x, y), self.last_xy) < self.path_min_dist:
            return
        self.last_xy = (x, y)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = pose.header.stamp
        self.pub.publish(self.path_msg)

    def check(self):
        if self.received:
            return
        self.get_logger().warn(
            'Sin datos de ground truth: el camino verde de RViz va a quedar vacio. '
            'Es lo esperado en Gazebo Classic (macOS) y con el rosbag del robot '
            'real, que no tienen pose verdadera. En Fortress, verifique que '
            '/ground_truth/odom este publicando.')


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
