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
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        # Cantidad de partículas configurable desde el launch.
        self.declare_parameter("num_particles", 1000)
        self.num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value


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

        # Almacenan las poses [x, y, theta] y los pesos de las partículas.
        self.particles = None
        self.weights = None

        # Parámetros de ruido para el modelo de movimiento holonómico.
        # Son valores iniciales de prueba; luego podremos ajustarlos.
        self.motion_alpha = np.array([
            0.05, 0.02, 0.005,
            0.05, 0.05, 0.05
        ])

        # Create a publisher for PointCloud2
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/particle_cloud", 10
        )

        # Recibe la posición inicial indicada con 2D Pose Estimate en RViz.
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10
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

        # Publicador de la pose estimada por el filtro de partículas.
        # Utilizamos Odometry para facilitar su integración con el TP2.
        self.particle_odom_pub = self.create_publisher(
            Odometry, "/particle_odom", 10
        )

        # Última posición agregada a la trayectoria del filtro.
        self.last_particle_xy = None

    def map_callback(self, msg):
        if not self.map_received:
            self.map_data = msg
            self.map_received = True
            self.grid = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            self.get_logger().info('Map and inflated map processed.')

    def initial_pose_callback(self, msg):
        """
        Inicializa las partículas alrededor de la pose seleccionada en RViz,
        incorporando incertidumbre en posición y orientación.
        """

        # Posición inicial seleccionada.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convertir la orientación de cuaternión a ángulo theta.
        q = msg.pose.pose.orientation

        theta = R.from_quat([
            q.x, q.y, q.z, q.w
        ]).as_euler('xyz', degrees=False)[2]

        # Desviaciones estándar de la distribución inicial.
        sigma_x = 0.20
        sigma_y = 0.20
        sigma_theta = np.deg2rad(10.0)

        # Cada partícula representa una posible pose [x, y, theta].
        self.particles = np.zeros((self.num_particles, 3))

        # Generar posiciones aleatorias alrededor de la pose inicial.
        self.particles[:, 0] = np.random.normal(
            x, sigma_x, self.num_particles
        )

        self.particles[:, 1] = np.random.normal(
            y, sigma_y, self.num_particles
        )

        self.particles[:, 2] = np.random.normal(
            theta, sigma_theta, self.num_particles
        )

        # Mantener los ángulos en el intervalo [-pi, pi].
        self.particles[:, 2] = (
            self.particles[:, 2] + np.pi
        ) % (2 * np.pi) - np.pi

        # Inicialmente todas las partículas tienen el mismo peso.
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Publicar la nube de partículas para visualizarla en RViz.
        cloud = self.create_pointcloud2(
            self.particles[:, :2],
            self.weights
        )

        self.pointcloud_pub.publish(cloud)

        self.get_logger().info(
            f"Inicializadas {self.num_particles} partículas."
        )

        # Reiniciar la trayectoria al seleccionar una nueva pose inicial.
        self.particle_path_msg.poses.clear()
        self.last_particle_xy = None

        # Limpiar la trayectoria anterior en RViz.
        self.particle_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.particle_path_pub.publish(self.particle_path_msg)

        # Publicar también la nueva pose inicial estimada.
        self.publish_particle_pose()

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

    def publish_particle_pose(self):
        """
        Calcula y publica la pose estimada del robot a partir de
        la distribución de partículas.

        La posición se obtiene mediante un promedio ponderado
        y la orientación mediante un promedio circular.
        """

        # No publicar hasta tener partículas inicializadas.
        if self.particles is None or self.weights is None:
            return

        # Normalizar los pesos para garantizar que sumen uno.
        weights = np.asarray(self.weights, dtype=float)
        total_weight = np.sum(weights)

        if total_weight <= 0 or not np.isfinite(total_weight):
            return

        weights = weights / total_weight

        # Posición estimada mediante promedio ponderado.
        x_est = np.sum(weights * self.particles[:, 0])
        y_est = np.sum(weights * self.particles[:, 1])

        # Promedio circular de las orientaciones.
        # Evita errores al promediar ángulos cercanos a -pi y pi.
        theta_est = np.arctan2(
            np.sum(weights * np.sin(self.particles[:, 2])),
            np.sum(weights * np.cos(self.particles[:, 2]))
        )

        # Crear el mensaje de odometría con la pose corregida.
        estimated_odom = Odometry()

        estimated_odom.header.stamp = self.get_clock().now().to_msg()
        estimated_odom.header.frame_id = "map"
        estimated_odom.child_frame_id = "base_link"

        estimated_odom.pose.pose.position.x = float(x_est)
        estimated_odom.pose.pose.position.y = float(y_est)
        estimated_odom.pose.pose.position.z = 0.0

        estimated_odom.pose.pose.orientation = yaw_to_quaternion(
            float(theta_est)
        )

        # Publicar la estimación para utilizarla posteriormente
        # en Path Planning y Pure Pursuit.
        self.particle_odom_pub.publish(estimated_odom)

        # Crear un punto de la trayectoria estimada.
        pose_stamped = PoseStamped()
        pose_stamped.header = estimated_odom.header
        pose_stamped.pose = estimated_odom.pose.pose

        current_xy = (float(x_est), float(y_est))

        # Agregar un nuevo punto cuando el robot se haya
        # desplazado una distancia suficiente.
        if (
            self.last_particle_xy is None
            or math.dist(current_xy, self.last_particle_xy)
            >= self.path_min_dist
        ):
            self.last_particle_xy = current_xy

            self.particle_path_msg.poses.append(pose_stamped)
            self.particle_path_msg.header.stamp = estimated_odom.header.stamp

            # Publicar el recorrido estimado por las partículas.
            self.particle_path_pub.publish(self.particle_path_msg)

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

        # MOVIMIENTO DE PARTÍCULAS

        # Guardamos la odometría anterior y actualizamos su valor.
        prev_x, prev_y, prev_theta = self.last_odom
        self.last_odom = (x, y, theta)

        # Si todavía no se indicó la pose inicial en RViz,
        # no hay partículas para actualizar.
        if self.particles is None:
            return

        # Desplazamiento medido por odometría.
        dx = x - prev_x
        dy = y - prev_y

        # Diferencia angular normalizada a [-pi, pi].
        delta_theta = np.arctan2(
            np.sin(theta - prev_theta),
            np.cos(theta - prev_theta)
        )

        # Convertimos el desplazamiento global al sistema
        # de referencia local del robot.
        delta_x_local = (
            np.cos(prev_theta) * dx
            + np.sin(prev_theta) * dy
        )

        delta_y_local = (
            -np.sin(prev_theta) * dx
            + np.cos(prev_theta) * dy
        )

        # Si no hubo movimiento, no hace falta actualizar.
        if (
            abs(delta_x_local) < 1e-9
            and abs(delta_y_local) < 1e-9
            and abs(delta_theta) < 1e-9
        ):
            return

        # Parámetros del modelo de ruido.
        a1, a2, a3, a4, a5, a6 = self.motion_alpha

        # Varianzas del movimiento holonómico.
        var_x = (
            a1 * delta_x_local**2
            + a2 * delta_y_local**2
            + a3 * delta_theta**2
        )

        var_y = (
            a2 * delta_x_local**2
            + a1 * delta_y_local**2
            + a3 * delta_theta**2
        )

        var_theta = (
            a4 * delta_x_local**2
            + a5 * delta_y_local**2
            + a6 * delta_theta**2
        )

        # Generamos un movimiento aleatorio independiente
        # para cada partícula.
        noisy_dx = delta_x_local + np.random.normal(
            0.0, np.sqrt(var_x), self.num_particles
        )

        noisy_dy = delta_y_local + np.random.normal(
            0.0, np.sqrt(var_y), self.num_particles
        )

        noisy_dtheta = delta_theta + np.random.normal(
            0.0, np.sqrt(var_theta), self.num_particles
        )

        # Orientación actual de cada partícula.
        particle_theta = self.particles[:, 2]

        # Transformamos los movimientos locales de cada partícula
        # al sistema global utilizando su propia orientación.
        self.particles[:, 0] += (
            np.cos(particle_theta) * noisy_dx
            - np.sin(particle_theta) * noisy_dy
        )

        self.particles[:, 1] += (
            np.sin(particle_theta) * noisy_dx
            + np.cos(particle_theta) * noisy_dy
        )

        # Actualizamos la orientación de las partículas.
        self.particles[:, 2] = (
            particle_theta + noisy_dtheta + np.pi
        ) % (2 * np.pi) - np.pi

        # Publicamos las nuevas posiciones en RViz.
        cloud = self.create_pointcloud2(
            self.particles[:, :2],
            self.weights
        )

        self.pointcloud_pub.publish(cloud)


    def scan_callback(self, data):
        # TODO: Implementar el modelo de sensor basado en laser para actualizar los pesos de las particulas
        # Deberan asignarle un peso a cada particula basado en la probabilidad de que el laser haya medido 
        # lo que midio dado el mapa de verosimilitud. Luego deberan implementar el resampling de las particulas
        # basado en sus pesos. Nuevamente, pueden graficar las particulas usando la funcion create_pointcloud2()
        # que ya esta implementada y recibe particulas y pesos como arreglos de numpy.

        """
        Actualiza el peso de las partículas comparando las mediciones
        del LIDAR con el campo de verosimilitud y aplica resampling.
        """

        # Esperar a tener el mapa y las partículas inicializadas.
        if not self.map_received or self.particles is None:
            return

        # Leer las distancias medidas por el LIDAR.
        ranges = np.asarray(data.ranges, dtype=float)

        if len(ranges) == 0:
            return

        # Utilizar aproximadamente 12 rayos para reducir el costo
        # computacional sin procesar todas las mediciones del LIDAR.
        indices = np.linspace(
            0, len(ranges) - 1,
            min(12, len(ranges)),
            dtype=int
        )

        distances = ranges[indices]

        # Descartar mediciones inválidas o sin impacto detectado.
        valid = (
            np.isfinite(distances)
            & (distances >= data.range_min)
            & (distances < data.range_max)
        )

        distances = distances[valid]
        indices = indices[valid]

        if len(distances) == 0:
            return

        # Ángulos de los rayos dentro de la terna del LIDAR.
        beam_angles = data.angle_min + indices * data.angle_increment

        # Obtener la posición y orientación de todas las partículas.
        px = self.particles[:, 0]
        py = self.particles[:, 1]
        theta = self.particles[:, 2]

        # El LIDAR está desplazado 0.043 m en X respecto del robot
        # y rotado 180 grados. El desplazamiento en Z no afecta
        # este cálculo sobre el mapa bidimensional.
        laser_x = px + 0.043 * np.cos(theta)
        laser_y = py + 0.043 * np.sin(theta)

        # Dirección global de cada medición para cada partícula.
        global_angles = (
            theta[:, None]
            + np.pi
            + beam_angles[None, :]
        )

        # Calcular los puntos donde impactarían los rayos.
        hit_x = (
            laser_x[:, None]
            + distances[None, :] * np.cos(global_angles)
        )

        hit_y = (
            laser_y[:, None]
            + distances[None, :] * np.sin(global_angles)
        )

        # Convertir coordenadas del mundo a índices del mapa.
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        cell_x = np.floor(
            (hit_x - origin_x) / resolution
        ).astype(int)

        cell_y = np.floor(
            (hit_y - origin_y) / resolution
        ).astype(int)

        # Verificar qué puntos pertenecen al mapa.
        height, width = self.grid.shape

        inside = (
            (cell_x >= 0)
            & (cell_x < width)
            & (cell_y >= 0)
            & (cell_y < height)
        )

        # Probabilidad pequeña para mediciones fuera del mapa.
        probabilities = np.full(hit_x.shape, 0.02)

        # Consultar el campo de verosimilitud.
        # Sus valores están entre 0 y 100.
        probabilities[inside] = (
            self.grid[cell_y[inside], cell_x[inside]] / 100.0
        )

        # Conservamos una probabilidad mínima para tolerar diferencias
        # entre el mapa y las mediciones reales del sensor.
        probabilities = 0.15 + 0.85 * probabilities

        # Calcular los pesos en escala logarítmica para evitar
        # problemas numéricos al combinar muchos rayos.
        log_weights = (
            np.log(np.maximum(self.weights, 1e-300))
            + np.sum(np.log(probabilities), axis=1)
        )

        # Normalizar los pesos.
        log_weights -= np.max(log_weights)

        weights = np.exp(log_weights)
        weights /= np.sum(weights)

        self.weights = weights

        # Calcular el número efectivo de partículas.
        # Si es muy pequeño, unas pocas partículas concentran
        # prácticamente toda la probabilidad.
        effective_particles = 1.0 / np.sum(self.weights ** 2)

        # Aplicar resampling sistemático cuando sea necesario.
        if effective_particles < 0.5 * len(self.particles):

            n = len(self.particles)

            # Construir la distribución acumulada de pesos.
            cumulative = np.cumsum(self.weights)
            cumulative[-1] = 1.0

            # Seleccionar partículas proporcionalmente a sus pesos.
            positions = (
                np.arange(n) + np.random.uniform()
            ) / n

            selected = np.searchsorted(cumulative, positions)

            # Copiar las partículas seleccionadas para evitar memoria compartida.
            self.particles = self.particles[selected].copy()

            # Agregar una pequeña dispersión después del resampling.
            # Esto evita que muchas copias de una misma partícula queden
            # exactamente superpuestas y se pierda diversidad.
            self.particles[:, 0] += np.random.normal(0.0, 0.03, n)
            self.particles[:, 1] += np.random.normal(0.0, 0.03, n)
            self.particles[:, 2] += np.random.normal(
                0.0, np.deg2rad(2.0), n
            )

            # Normalizar los ángulos al intervalo [-pi, pi].
            self.particles[:, 2] = (
                self.particles[:, 2] + np.pi
            ) % (2 * np.pi) - np.pi

            # Después del resampling, todas las partículas tienen igual peso.
            self.weights = np.full(n, 1.0 / n)

        # Publicar las partículas actualizadas en RViz.
        cloud = self.create_pointcloud2(
            self.particles[:, :2],
            self.weights
        )

        self.pointcloud_pub.publish(cloud)

        # Publicar la pose corregida después de procesar el LIDAR.
        self.publish_particle_pose()
        

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