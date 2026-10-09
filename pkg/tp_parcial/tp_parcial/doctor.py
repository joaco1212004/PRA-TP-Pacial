#!/usr/bin/env python3
"""
Chequeo previo del entorno del TP. Corralo ANTES de escribir codigo.

Cada defecto que costo tiempo en este TP tiene la misma forma: un topico que no
existe, un QoS incompatible, una terna rotada, un paquete que no esta instalado.
Ninguno imprime un error: el alumno ve "no pasa nada" y busca el problema en su
algoritmo. Este nodo convierte esas fallas mudas en una lista con causa y arreglo.

    ros2 launch custom_map cafe.launch.py rviz:=False     # en otra terminal
    ros2 run tp_parcial doctor

Con `--section 2` chequea ademas lo que necesita el EKF (landmarks y la cadena
de TF que los hace posibles).
"""

import argparse
import importlib
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

VERDE, ROJO, AMAR, GRIS, FIN = '\033[32m', '\033[31m', '\033[33m', '\033[90m', '\033[0m'
OK, FALLA, AVISO = f'{VERDE}  ok  {FIN}', f'{ROJO} FALLA{FIN}', f'{AMAR} aviso{FIN}'

# Tiempo que se espera por un topico antes de darlo por ausente. El LIDAR puede
# ir a 5 Hz si Gazebo cae a render por software, asi que conviene ser generoso.
ESPERA = 8.0


class Doctor(Node):

    def __init__(self, seccion):
        super().__init__('doctor')
        self.seccion = seccion
        self.fallas = []
        self.avisos = []

    # ------------------------------------------------------------- utilidades
    def linea(self, estado, titulo, detalle='', arreglo=''):
        print(f'{estado}  {titulo}')
        if detalle:
            print(f'        {GRIS}{detalle}{FIN}')
        if arreglo:
            print(f'        {AMAR}-> {arreglo}{FIN}')

    def bien(self, t, d=''):
        self.linea(OK, t, d)

    def mal(self, t, d='', arreglo=''):
        self.linea(FALLA, t, d, arreglo)
        self.fallas.append(t)

    def ojo(self, t, d='', arreglo=''):
        self.linea(AVISO, t, d, arreglo)
        self.avisos.append(t)

    def espera_mensaje(self, tipo, topico, qos=10, espera=ESPERA):
        """Devuelve (n_publicadores, primer_mensaje o None)."""
        import time
        caja = {}
        sub = self.create_subscription(tipo, topico, lambda m: caja.setdefault('m', m), qos)
        t0 = time.time()
        while time.time() - t0 < espera and 'm' not in caja:
            rclpy.spin_once(self, timeout_sec=0.1)
        n = self.count_publishers(topico)
        self.destroy_subscription(sub)
        return n, caja.get('m')

    # ------------------------------------------------------------- chequeos
    def dependencias(self):
        print(f'\n{GRIS}--- dependencias de Python ---{FIN}')
        for mod, paquete in (('numpy', 'python3-numpy'),
                             ('scipy', 'python3-scipy'),
                             ('cv2', 'python3-opencv')):
            try:
                importlib.import_module(mod)
                self.bien(f'import {mod}')
            except ImportError:
                critico = mod != 'cv2' or self.seccion == 2
                (self.mal if critico else self.ojo)(
                    f'falta {mod}',
                    'cv2 lo usa custom_map/features.py, que publica /landmarks'
                    if mod == 'cv2' else 'lo importan los esqueletos del TP',
                    f'sudo apt install ros-$ROS_DISTRO-{paquete}  (o el equivalente de tu entorno)')

    def topicos_base(self):
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        print(f'\n{GRIS}--- el simulador y el mapa ---{FIN}')

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        latched.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        n, m = self.espera_mensaje(OccupancyGrid, '/map', latched)
        if m:
            self.bien('/map publica',
                      f'{m.info.width}x{m.info.height} @ {m.info.resolution} m/celda')
        else:
            self.mal('/map no publica', f'{n} publicadores',
                     'el map_server es un nodo de ciclo de vida: si quedo en unconfigured '
                     'no publica nunca. Revisalo con `ros2 lifecycle get /map_server` '
                     '(tiene que decir active). Suele pasar si el lanzamiento '
                     'anterior no bajo limpio.')

        n, m = self.espera_mensaje(LaserScan, '/scan', qos_profile_sensor_data)
        if m:
            self.bien('/scan publica',
                      f'{len(m.ranges)} rayos, frame {m.header.frame_id}')
        else:
            self.mal('/scan no publica', f'{n} publicadores',
                     'si hay 1 publicador y 0 mensajes, mira el QoS: suscribite con '
                     'qos_profile_sensor_data. Si hay 0 publicadores, el sensor del '
                     'simulador no arranco.')

        n, m = self.espera_mensaje(Odometry, '/calc_odom')
        if m and n == 1:
            self.bien('/calc_odom publica', '1 publicador, que es lo correcto')
        elif n > 1:
            self.mal('/calc_odom tiene mas de un publicador', f'{n} publicadores',
                     'hay dos nodos integrando odometria a la vez. Revisa que '
                     'calculated_odometry.py no se lance dos veces.')
        else:
            self.mal('/calc_odom no publica', f'{n} publicadores',
                     'es la odometria que DERIVA, la que el TP pide corregir. '
                     'La publica calculated_odometry.py. Si no esta, el filtro no '
                     'recibe movimiento y no pasa nada, sin ningun error.')

        n, _ = self.espera_mensaje(Odometry, '/ground_truth/odom', espera=3.0)
        if n:
            self.bien('/ground_truth/odom disponible', 'el camino verde de RViz va a funcionar')
        else:
            self.ojo('/ground_truth/odom ausente',
                     'sin pose verdadera no se puede evaluar el criterio "el camino rojo '
                     'sigue al verde"', 'es lo esperado con el rosbag del robot real')

    def ternas(self):
        import tf2_ros
        print(f'\n{GRIS}--- cadena de TF ---{FIN}')
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self)
        import time
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # la terna del laser: el error mas caro del TP si se omite
        for laser in ('laser_frame', 'laser_link'):
            try:
                t = buf.lookup_transform('base_link', laser, rclpy.time.Time())
                q = t.transform.rotation
                yaw = math.degrees(math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))
                tr = t.transform.translation
                if abs(abs(yaw) - 180.0) < 1.0:
                    self.bien(f'base_link -> {laser}',
                              f'({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f}) yaw {yaw:.1f} grados '
                              '- acordate de sumarle pi al angulo de cada rayo')
                else:
                    self.ojo(f'base_link -> {laser} con yaw {yaw:.1f}',
                             'el TP asume 180 grados')
                break
            except Exception:
                continue
        else:
            self.mal('no existe la terna del laser',
                     'ni laser_frame ni laser_link cuelgan de base_link')

        try:
            buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.bien('map -> base_footprint resuelve')
        except Exception as e:
            self.mal('map -> base_footprint NO resuelve', str(e)[:90],
                     'base_footprint quedo huerfano. En Linux suele ser que falta '
                     'ros-$ROS_DISTRO-gz-ros2-control: sin el no arranca el '
                     'controller_manager, no hay /joint_states, y la odometria de '
                     'ruedas no publica el TF. landmark_observer se come la excepcion '
                     'y /observed_landmarks queda en cero mensajes, sin decir nada.')

    def seccion_dos(self):
        from geometry_msgs.msg import PoseArray
        print(f'\n{GRIS}--- seccion 2: landmarks ---{FIN}')
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        latched.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        n, m = self.espera_mensaje(PoseArray, '/landmarks', latched)
        if m:
            self.bien('/landmarks publica', f'{len(m.poses)} landmarks en el mapa')
        else:
            self.mal('/landmarks no publica', f'{n} publicadores',
                     'lo publica custom_map/features.py, que importa cv2. Si cv2 no '
                     'esta, el nodo muere al arrancar y el mensaje se pierde entre '
                     'la salida del launch.')

        n, m = self.espera_mensaje(PoseArray, '/observed_landmarks')
        if m:
            vistos = sum(1 for p in m.poses
                         if abs(p.position.x) > 1e-9 or abs(p.position.y) > 1e-9)
            self.bien('/observed_landmarks publica', f'{vistos} detectados de {len(m.poses)}')
            if vistos == 0:
                self.ojo('ningun landmark detectado en este instante',
                         'puede ser normal si el robot mira a una pared; movelo y reintenta')
        else:
            self.mal('/observed_landmarks no publica', f'{n} publicadores',
                     'si hay publicador y cero mensajes, landmark_observer no puede hacer '
                     'lookup_transform de base_footprint: revisa el chequeo de TF de arriba. '
                     'La excepcion la come un except y no se ve.')

    def resumen(self):
        print()
        if self.fallas:
            print(f'{ROJO}{len(self.fallas)} problema(s) a resolver '
                  f'antes de empezar:{FIN}')
            for f in self.fallas:
                print(f'  - {f}')
        else:
            print(f'{VERDE}Entorno OK.{FIN}')
        if self.avisos:
            print(f'{AMAR}{len(self.avisos)} aviso(s), no bloquean:{FIN}')
            for a in self.avisos:
                print(f'  - {a}')
        return 1 if self.fallas else 0


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--section', type=int, default=1, choices=(1, 2),
                    help='2 chequea ademas los landmarks del EKF')
    conocidos, resto = ap.parse_known_args()

    rclpy.init(args=resto)
    d = Doctor(conocidos.section)
    print(f'\n{GRIS}Chequeo del entorno del TP (seccion {conocidos.section}). '
          f'Necesita el simulador corriendo.{FIN}')
    codigo = 1
    try:
        d.dependencias()
        d.topicos_base()
        d.ternas()
        if conocidos.section == 2:
            d.seccion_dos()
        codigo = d.resumen()
    except KeyboardInterrupt:
        pass
    finally:
        d.destroy_node()
        rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
