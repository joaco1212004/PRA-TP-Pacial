#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    pkg_share = get_package_share_directory('tp_parcial')
    # RViz configuration (can reuse the one from slam_toolbox setup)
    rviz_config_file = LaunchConfiguration('rviz_config', 
                                           default=os.path.join(pkg_share, 'rviz', 'ekf.rviz'))

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar el reloj simulado/bag si es true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # El EKF se suscribe a /calc_odom, la odometria que deriva. El argumento
    # queda expuesto para poder apuntarlo a /odom con el rosbag del robot real,
    # donde /calc_odom no existe.
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/calc_odom',
        description='Nombre del tópico de odometría (/calc_odom en simulación, /odom con el rosbag)'
    )

    # --- Define nodes ---
    ekf_localization = Node(
        package='tp_parcial',
        executable='ekf_localization',
        name='ekf_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/calc_odom', LaunchConfiguration('odom_topic'))]
    )

    # Mismo motivo que en particle_filter.launch.py: el criterio de exito de la
    # seccion 2.3 ("se corrige la posicion para estar mas cerca de la posicion
    # real") necesita que la posicion real este dibujada.
    ground_truth_path = Node(
        package='tp_parcial',
        executable='ground_truth_path',
        name='ground_truth_path',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    
    # RViz2
    start_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(start_rviz2_cmd)
    
    ld.add_action(ekf_localization)
    ld.add_action(ground_truth_path)

    return ld 