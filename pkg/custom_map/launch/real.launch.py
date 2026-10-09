#!/usr/bin/env python3

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_custom = get_package_share_directory("custom_map")
    default_map = os.path.join(pkg_custom, "maps", "my_map3.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
        }],
    )

    # map_server is a lifecycle node: without something to configure and
    # activate it, it starts, waits, and never publishes /map.
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_server_lifecycle_manager",
        output="screen",
        parameters=[{
            # use_sim_time FALSE a proposito, aunque el map_server si lo use.
            #
            # El lifecycle manager secuencia las transiciones con temporizadores.
            # Con use_sim_time=true esos temporizadores corren contra /clock, y
            # /clock lo publica el rosbag -- que la consigna manda arrancar
            # DESPUES de este launch. Sin reloj, el manager configura el
            # map_server y se queda esperando para siempre: /map nunca publica,
            # el campo de verosimilitud nunca sale, y el filtro no corrige.
            # Nada de eso imprime un error.
            #
            # El manager solo necesita tiempo de pared para ordenar transiciones;
            # el map_server conserva su use_sim_time y sigue estampando bien.
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    # Deliberately fixed, see the module docstring. Because it is static and
    # nothing else publishes map -> odom, odom keeps exactly one parent.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Open the Gazebo window (Classic only)"),
        map_server,
        map_lifecycle,
        map_to_odom,
    ])
