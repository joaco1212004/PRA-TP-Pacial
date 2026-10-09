# TP Parcial — Localización y Movimiento (I-402)

Este paquete trae los esqueletos del trabajo. La consigna completa está en el PDF;
acá está lo necesario para que el entorno funcione y para no perder horas en
problemas que no son del algoritmo.

---

## 1. Setup — elegí tu máquina

El TP corre en dos entornos distintos y **no se mezclan**. Usá el que corresponda.

### macOS → pixi + RoboStack + Gazebo Classic

Acá **no se usa `apt` ni `rosdep`**: todas las dependencias vienen del `pixi.toml`
del repositorio de la materia. Verificá que incluya estas dos, y agregalas si no
están:

```toml
[feature.classic.dependencies]
scipy = "*"       # lo importan los esqueletos
py-opencv = "*"   # lo importa custom_map/features.py, que publica /landmarks
```

```bash
pixi install
pixi run -e classic colcon build --symlink-install
```

Todo comando de acá en adelante va prefijado con `pixi run -e classic`.

> En macOS el simulador es **Gazebo Classic**, no Fortress: Fortress no puede abrir
> su ventana ni correr sensores renderizados en Mac, así que no habría LIDAR. Los
> launch eligen el backend solos según el sistema operativo.

### Ubuntu 22.04 → ROS 2 Humble nativo + Gazebo Fortress

Acá **no se usa pixi**. El `pixi.toml` del repositorio es solo para macOS: ignoralo.

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Además hacen falta estos paquetes, que `rosdep` no siempre arrastra:

```bash
sudo apt install -y ros-humble-gz-ros2-control ros-humble-topic-tools \
                    ros-humble-joint-state-publisher ros-humble-ros2-controllers
```

**`gz-ros2-control` no es opcional.** Sin él no arranca el `controller_manager`, no
hay `/joint_states`, la odometría de ruedas no publica su transformada, y
`base_footprint` queda colgando fuera del árbol de TF. Consecuencia: la Sección 2
no recibe **ninguna** observación de landmarks. Y nada de eso imprime un error.

Si tenés GPU y Gazebo va lento, probablemente tu usuario no puede abrir el
dispositivo de render:

```bash
sudo usermod -aG render,video $USER   # hay que volver a iniciar sesión
```

---

## 2. Antes de escribir una línea de código

```bash
# terminal 1
ros2 launch custom_map cafe.launch.py rviz:=False
# terminal 2
ros2 run tp_parcial doctor            # o: doctor --section 2
```

El `doctor` revisa las nueve cosas que, cuando fallan, **no producen ningún error**:
dependencias de Python, `/map`, `/scan` y su QoS, `/calc_odom`, la terna del LIDAR,
la cadena de TF y los landmarks. Cada falla viene con su causa y su arreglo.

Si el `doctor` no da verde, arreglá eso primero: cualquier síntoma raro que veas
después va a venir de ahí y no de tu filtro.

---

## 3. Las tres secciones

En macOS, prefijá cada comando con `pixi run -e classic`.

### Sección 1 — Filtro de partículas

```bash
ros2 launch custom_map cafe.launch.py rviz:=False
ros2 launch tp_parcial particle_filter.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

En RViz, marcá la pose inicial con **2D Pose Estimate**: hasta que no lo hagas, no
hay partículas y no pasa nada (es lo esperado, y el nodo lo avisa por consola).

**Lo que tenés que ver**, manejando el robot:

| color | tópico | qué es |
|---|---|---|
| 🟢 verde | `/real_robot_path` | dónde está el robot de verdad |
| 🔵 azul | `/calc_robot_path` | lo que cree la odometría — **se despega** |
| 🔴 rojo | `/particle_robot_path` | tu filtro — **tiene que seguir al verde** |

Variantes para el análisis:

```bash
ros2 launch tp_parcial particle_filter.launch.py sigma:=2.0
ros2 launch tp_parcial particle_filter.launch.py num_particles:=500
```

### Sección 2 — EKF con landmarks

```bash
ros2 launch custom_map cafe_features.launch.py rviz:=False     # + fov:=360.0
ros2 launch tp_parcial ekf.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Mirá `/observed_landmarks` (naranja) en RViz: es lo único que te dice si el sensor
está viendo algo. Si está vacío todo el tiempo, corré `doctor --section 2`.

### Sección 3 — Rosbag del robot real

```bash
ros2 launch tp_parcial tu_launch.py odom_topic:=/odom num_particles:=1000
ros2 launch custom_map real.launch.py
ros2 bag play ruta/a/tu_bag --clock
```

Dos diferencias con la simulación:

- **`odom_topic:=/odom`**, porque en el robot real no existe `/calc_odom`.
- El `/scan` del bag está grabado **BEST_EFFORT**. Un subscriptor con el perfil por
  defecto (RELIABLE) es incompatible y **no recibe un solo mensaje**. Por eso el
  esqueleto usa `qos_profile_sensor_data`; si lo cambiás, se rompe. ROS avisa con un
  WARN de *incompatible QoS* que se pierde entre la salida del launch.

El robot arranca y termina más o menos en el mismo lugar, así que la distancia entre
tu pose final y el origen es tu medida de calidad.

---

## 4. Cuando "no pasa nada"

Casi ningún problema de este TP produce un error. Esta tabla es el atajo:

| Síntoma | Causa probable | Cómo confirmarlo |
|---|---|---|
| Las partículas nunca aparecen | falta el **2D Pose Estimate** | el nodo lo avisa cada 10 s |
| El campo de verosimilitud no sale | `map_server` quedó en `unconfigured` | `ros2 lifecycle get /map_server` → tiene que decir `active` |
| El filtro no se mueve | te suscribiste al tópico de odometría equivocado | `ros2 topic hz /calc_odom` |
| El filtro converge **espejado** | te olvidaste de la rotación de 180° del LIDAR | `ros2 run tf2_ros tf2_echo base_link laser_frame` |
| La §2 nunca corrige | falta `gz-ros2-control` (Ubuntu) | `ros2 run tf2_ros tf2_echo map base_footprint` |
| El bag reproduce pero el filtro no recibe | QoS del `/scan` | `ros2 topic info -v /scan` |
| Todo anduvo y de golpe dejó de andar | quedaron nodos del lanzamiento anterior | `ros2 node list` — buscá nombres repetidos |

**El último es el más frecuente.** `ros2 launch` no siempre baja a sus hijos: si ves
nodos duplicados, bajá todo y relanzá antes de buscar el problema en tu código.

---

## 5. La terna del LIDAR

Está en la consigna, pero se pasa por alto y es el error más caro del TP:

```
base_link → laser_frame:  (0.043, 0, 0.110) m,  yaw 180°
```

Un rayo con ángulo `a` en la terna del láser apunta a **`a + π`** visto desde el
robot, y su origen está 4.3 cm adelante del eje. Si lo omitís, el filtro converge a
una pose espejada **sin emitir un solo error**.

La grilla se indexa `grid[y, x]` —primero fila (eje Y), después columna— con la celda
(0,0) en la esquina **inferior izquierda**.

---

## 6. Puntos de partida razonables

No son la respuesta, son para que no arranques de cero:

| Parámetro | Valor | Por qué |
|---|---|---|
| `sigma` del campo | **0.1 – 0.2 m** | por debajo, la nube se empobrece; por encima de 1 m el campo deja de discriminar |
| `num_particles` | **≥ 300** | entre 50 y 200 funciona *casi siempre*, que es peor que fallar siempre |
| dispersión inicial | 0.3 – 0.6 m, 20° | el filtro solo converge a algo que esté dentro de la nube inicial |
| `sigma` con el rosbag | **más grande** (≈ 0.35 m) | el mapa del laboratorio se hizo con SLAM y tiene error propio |
