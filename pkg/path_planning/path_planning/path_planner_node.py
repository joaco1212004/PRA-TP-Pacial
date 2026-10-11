#!/usr/bin/env python3
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.map = None
        self.odom = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        # Utilizamos la posición corregida por el filtro de partículas
        # para determinar el punto inicial de la planificación.
        self.declare_parameter('localization_source', 'particles')
        source = self.get_parameter('localization_source').value
        if source not in ('particles', 'ekf'):
            raise ValueError('localization_source debe ser particles o ekf')
        pose_type = PoseWithCovarianceStamped if source == 'ekf' else Odometry
        pose_topic = '/pose_with_covariance' if source == 'ekf' else '/particle_odom'
        self.odom_sub = self.create_subscription(
            pose_type, pose_topic, self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.get_logger().info('Planner iniciado.')

    def map_callback(self, msg):
        self.map = msg

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        if self.map is None:
            self.get_logger().warn('Todavía no se recibió /map.')
            return
        if self.odom is None:
            self.get_logger().warn('Todavía no se recibió la localización.')
            return

        self.plan(msg)

    def world_to_map(self, x, y):
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        res = self.map.info.resolution

        mx = int((x - ox) / res)
        my = int((y - oy) / res)

        if not (0 <= mx < self.map.info.width and
                0 <= my < self.map.info.height):
            return None

        return (mx, my)

    def map_to_world(self, mx, my):
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        res = self.map.info.resolution

        return (
            ox + (mx + 0.5) * res,
            oy + (my + 0.5) * res
        )

    def plan(self, goal_msg):
        start = self.world_to_map(
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y
        )

        goal = self.world_to_map(
            goal_msg.pose.position.x,
            goal_msg.pose.position.y
        )

        if start is None:
            self.get_logger().warn('La posición actual está fuera del mapa.')
            return

        if goal is None:
            self.get_logger().warn('El objetivo está fuera del mapa.')
            return

        self.get_logger().info(f'Planificando de {start} a {goal}.')

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # [COMPLETAR] Implementar algún algoritmo de planificación de caminos
        # Debe devolver una lista de tuplas (x, y) en coordenadas de mapa, o None si no se encuentra un camino
        
        width = self.map.info.width
        height = self.map.info.height

        # -------------------------------------------------------------
        # Inflación de obstáculos para planificación segura.
        # -------------------------------------------------------------

        # Convertimos el OccupancyGrid a una matriz bidimensional.
        occupancy_grid = np.array(
            self.map.data,
            dtype=np.int16
        ).reshape(height, width)

        # Consideramos obstáculos las celdas ocupadas (>= 50)
        # y las desconocidas (-1), por seguridad.
        obstacles = (occupancy_grid >= 50) | (occupancy_grid == -1)

        # Distancia mínima de seguridad desde el centro del robot
        # hasta los obstáculos, expresada en metros.
        # Valor inicial para evaluar experimentalmente.
        safety_radius = 0.20

        # Agregamos un borde ocupado para tratar también los límites
        # del mapa como obstáculos.
        padded_obstacles = np.pad(
            obstacles,
            1,
            mode="constant",
            constant_values=True
        )

        # Calculamos la distancia al obstáculo más cercano.
        # El argumento sampling convierte las distancias a metros.
        distance_map = distance_transform_edt(
            ~padded_obstacles,
            sampling=self.map.info.resolution
        )[1:-1, 1:-1]

        # Una celda es transitable únicamente si está libre
        # y respeta el margen de seguridad establecido.
        safe_cells = (~obstacles) & (distance_map >= safety_radius)

        self.get_logger().info(
            f"Planificación con margen de seguridad de {safety_radius:.2f} m."
        )

        def is_free(cell):
            """
            Comprueba si una celda es transitable considerando
            la inflación de obstáculos.

            Una celda debe encontrarse dentro del mapa y mantener
            una distancia mínima de seguridad respecto de paredes,
            obstáculos y regiones desconocidas.
            """

            x, y = cell

            # Verificar que la celda esté dentro del mapa.
            if not (0 <= x < width and 0 <= y < height):
                return False

            # Consultar la matriz de celdas seguras.
            # En NumPy se indexa como [fila, columna] = [y, x].
            return bool(safe_cells[y, x])


        def get_neighbors(cell):
            """
            Devuelve los vecinos libres usando conectividad de 8 vecinos.
            """
            x, y = cell
            neighbors = []

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:

                    if dx == 0 and dy == 0:
                        continue

                    neighbor = (x + dx, y + dy)

                    if not is_free(neighbor):
                        continue

                    # Si el movimiento es diagonal, evitamos atravesar
                    # la esquina de dos obstáculos.
                    if dx != 0 and dy != 0:
                        if not is_free((x + dx, y)):
                            continue
                        if not is_free((x, y + dy)):
                            continue

                    neighbors.append(neighbor)

            return neighbors


        # Antes de planificar verificamos que inicio y objetivo sean transitables.
        if not is_free(start):
            self.get_logger().warn('La posición inicial no está en una celda libre.')
            return

        if not is_free(goal):
            self.get_logger().warn('El objetivo no está en una celda libre.')
            return


        # Cola de prioridad de A*.
        # Cada elemento contiene:
        # (costo_estimado_total, celda)
        open_set = []

        heapq.heappush(open_set, (0.0, start))

        # Costo acumulado desde el inicio hasta cada celda.
        g_cost = {start: 0.0}

        # Predecesor de cada celda, utilizado luego para reconstruir el camino.
        predecessor = {}

        # Celdas que ya fueron completamente expandidas.
        closed = set()

        path = None


        while open_set:

            _, current = heapq.heappop(open_set)

            # Una misma celda puede aparecer más de una vez en la cola.
            # Si ya fue expandida, la ignoramos.
            if current in closed:
                continue

            # Si llegamos al objetivo, reconstruimos el camino.
            if current == goal:

                path = [current]

                while current != start:
                    current = predecessor[current]
                    path.append(current)

                path.reverse()
                break

            closed.add(current)

            for neighbor in get_neighbors(current):

                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]

                # Movimiento horizontal/vertical: costo 1.
                # Movimiento diagonal: costo sqrt(2).
                movement_cost = math.sqrt(dx**2 + dy**2)

                new_cost = g_cost[current] + movement_cost

                # Si encontramos una forma más barata de llegar al vecino,
                # actualizamos su costo y su predecesor.
                if (
                    neighbor not in g_cost
                    or new_cost < g_cost[neighbor]
                ):
                    g_cost[neighbor] = new_cost
                    predecessor[neighbor] = current

                    # Heurística euclídea hasta el objetivo.
                    heuristic = math.sqrt(
                        (goal[0] - neighbor[0])**2
                        + (goal[1] - neighbor[1])**2
                    )

                    total_cost = new_cost + heuristic

                    heapq.heappush(
                        open_set,
                        (total_cost, neighbor)
                    )

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if path is None:
            self.get_logger().warn('No se encontró un camino.')
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map.header.frame_id

        for mx, my in path:
            x, y = self.map_to_world(mx, my)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

        self.get_logger().info(
            f'Camino publicado: {len(path)} puntos.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()