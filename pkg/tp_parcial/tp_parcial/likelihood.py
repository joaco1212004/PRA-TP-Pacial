import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import distance_transform_edt

class LikelihoodMapPublisher(Node):
    def __init__(self):
        super().__init__('likelihood_map_publisher')
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, '/likelihood_map', qos)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )

        self.declare_parameter("sigma", 1.0)
        self.sigma = self.get_parameter("sigma").get_parameter_value().double_value
    
    def map_callback(self, msg):
        prob_msg = OccupancyGrid()
        prob_msg.header = msg.header
        prob_msg.info = msg.info


        # TODO: Completar el codigo para que el mensaje contenga la data del mapa de probabilidades
        # prob_msg.data = mapa de probabilidades
        
        # Procuren utilizar el valor de self.sigma para calcular la probabilidad de cada celda del mapa de ocupación.

        # ROS espera que la data sea un array unidimensional de enteros de 8 bits con valores entre 0 y 100
        # msg.data contiene la data del mapa de ocupación como un array de enteros de 8 bits con 0 para celdas libres
        # 100 para celdas ocupadas y -1 para celdas desconocidas

        # Convertimos el mapa recibido de un array 1D a una matriz 2D.
        # OccupancyGrid se ordena como grid[y, x].
        occupancy_grid = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            (msg.info.height, msg.info.width)
        )

        # Las celdas ocupadas tienen valor 100.
        occupied = occupancy_grid == 100

        # Calculamos para cada celda la distancia al obstáculo más cercano.
        # distance_transform_edt calcula distancia hasta el cero más cercano,
        # por eso usamos ~occupied:
        #   obstáculo -> 0
        #   resto     -> 1
        distance = distance_transform_edt(~occupied)

        # Campo de verosimilitud gaussiano.
        # Sobre el obstáculo la distancia es 0 y la probabilidad es máxima.
        # La probabilidad disminuye al alejarnos del obstáculo.
        likelihood = np.exp(
            -(distance ** 2) / (2.0 * self.sigma ** 2)
        )

        # OccupancyGrid espera valores enteros entre 0 y 100.
        likelihood_grid = np.clip(
            np.round(likelihood * 100.0),
            0,
            100
        ).astype(np.int8)

        # Volvemos al formato unidimensional requerido por ROS.
        prob_msg.data = likelihood_grid.flatten().tolist()

        self.pub.publish(prob_msg)
        self.get_logger().info("Published likelihood map")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
