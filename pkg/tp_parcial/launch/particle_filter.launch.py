#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tp_parcial')
    rviz_config_file = LaunchConfiguration('rviz_config', 
                                           default=os.path.join(pkg_share, 'rviz', 'particulas.rviz'))
    
    # Declaración de argumentos
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar el reloj simulado/bag si es true'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/calc_odom',
        description='Nombre del tópico de odometría (/odom para bag/real, /calc_odom para sim)'
    )

    # Para poder correr headless o en CI sin que arranque RViz.
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Abrir RViz2'
    )

    num_particles_arg = DeclareLaunchArgument(
        "num_particles",
        default_value="1000",
        description="Number of particles for the filter"
    )

    sigma_arg = DeclareLaunchArgument(
        "sigma",
        default_value="1.0",
        description="Sigma for the likelihood function"
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')

    likelihood = Node(
        package='tp_parcial',
        executable='likelihood',
        name='likelihood',
        output='screen',
        parameters=[{
            "sigma": LaunchConfiguration("sigma"),
            "use_sim_time": use_sim_time
        }]
    )

    particle_localization = Node(
        package='tp_parcial',
        executable='particle_localization',
        name='particle_localization',
        output='screen',
        parameters=[{
            "num_particles": LaunchConfiguration("num_particles"),
            "use_sim_time": use_sim_time
        }],
        remappings=[
            ('/calc_odom', odom_topic)
        ]
    )
    
    # Publica /real_robot_path, el camino VERDE que espera particulas.rviz. Sin
    # el, el criterio de exito de la consigna ("el camino rojo sigue el camino
    # verde") no se puede observar, porque ningun nodo publicaba ese topico.
    ground_truth_path = Node(
        package='tp_parcial',
        executable='ground_truth_path',
        name='ground_truth_path',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # RViz2
    start_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(rviz_arg)
    ld.add_action(num_particles_arg)
    ld.add_action(sigma_arg)
    
    ld.add_action(start_rviz2_cmd)
    ld.add_action(likelihood)
    ld.add_action(particle_localization)
    ld.add_action(ground_truth_path)

    return ld