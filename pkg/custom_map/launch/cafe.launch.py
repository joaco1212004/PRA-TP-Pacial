#!/usr/bin/env python3
"""
Gazebo in the custom cafe, the recorded map, and RViz showing both.

This is the only launch file in the package and the only thing anyone has to
run:

    ros2 launch custom_map cafe.launch.py

It picks the simulator that works on the machine it is started on. Gazebo
Fortress on macOS initialises ogre2 on a secondary thread and Cocoa only allows
window creation on the main thread, so there it has no GUI and no rendering
sensor -- `/scan` never publishes. Gazebo Classic opens its window and its CPU
raycast lidar needs no rendering at all. So macOS gets Classic and everything
else gets Fortress. Override with `backend:=classic` or `backend:=fortress`.

`map -> odom` is a fixed identity transform, on purpose. The robot's pose in the
map is therefore its raw odometry, which drifts -- that is the point, so the
drift can be watched against /ground_truth/odom and corrected by hand. Nothing
here estimates a pose, and nothing here should: no AMCL, no SLAM, no Nav2.

Nothing in this package requires the rest of the workspace to be rebuilt. The
Fortress simulation is included from yahboom_rosmaster_gazebo's installed share
directory, and the model path is extended here rather than in the environment
activation script.
"""
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


CLASSIC_LAUNCH = os.path.join(
    "yahboom_robostack_M_silycon", "launch", "simulation.launch.py")


def _resolve_classic_launch(pkg_custom):
    """
    Find the Gazebo Classic bring-up, which is not an installed ROS package.

    It lives in a plain folder at the top of the repository: the folder name
    contains a capital letter, which is not a legal ROS package name, so it
    cannot be found with get_package_share_directory and has to be located on
    disk. Copying it in here instead would fork the macOS specifics it carries
    (the .dylib plugin names, the spawn timeout), which is exactly the kind of
    drift this package is trying to avoid.
    """
    candidates = []
    pixi_root = os.environ.get("PIXI_PROJECT_ROOT")
    if pixi_root:
        candidates.append(os.path.join(pixi_root, CLASSIC_LAUNCH))
    # install/custom_map/share/custom_map -> the workspace root, four levels up
    candidates.append(os.path.join(
        os.path.abspath(os.path.join(pkg_custom, "..", "..", "..", "..")), CLASSIC_LAUNCH))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        "Could not find the Gazebo Classic bring-up. Looked in:\n  "
        + "\n  ".join(candidates)
        + "\nStart this from inside the repository's `classic` pixi environment "
          "(pixi run -e classic ...), or pass backend:=fortress.")


def _rviz(pkg_custom, use_sim_time, want_rviz):
    """Open RViz on this package's config: Map display, fixed frame map."""
    if not want_rviz:
        return LogInfo(msg="rviz:=false, not opening RViz")
    # Late enough that /map and the robot description are already up, so the
    # Map and RobotModel displays populate on the first frame.
    return TimerAction(period=8.0, actions=[Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_custom, "rviz", "gazebo.rviz")],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
        output="screen",
    )])


def _simulator(context):
    pkg_custom = get_package_share_directory("custom_map")
    backend = context.launch_configurations.get("backend", "auto")
    if backend == "auto":
        backend = "classic" if platform.system() == "Darwin" else "fortress"

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    gui = context.launch_configurations.get("gui", "true")
    # Read before the include runs. IncludeLaunchDescription's launch_arguments
    # are SetLaunchConfiguration actions that leak into this scope, so telling
    # the simulation `rviz:=false` below would otherwise turn off the RViz
    # started here too. Grouping the include to contain the leak is worse: the
    # simulation defers its own RViz in a TimerAction, and a scoped group is
    # popped long before that timer fires, so `rviz:=false` never reaches it.
    want_rviz = context.launch_configurations.get("rviz", "true").lower() \
        in ("true", "1", "yes")
    models = os.path.join(pkg_custom, "models")

    if backend == "classic":
        world = os.path.join(pkg_custom, "worlds", "cafe_classic.world")
        return [
            # Classic resolves model:// through this. Setting it here keeps the
            # environment activation script untouched.
            AppendEnvironmentVariable("GAZEBO_MODEL_PATH", models),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_resolve_classic_launch(pkg_custom)),
                launch_arguments={
                    "world": world,
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                    # RViz is started here instead, on this package's config.
                    "rviz": "false",
                }.items(),
            ),
            # /calc_odom: la odometria que DERIVA, integrando el /cmd_vel crudo
            # (antes de que el watchdog le inyecte el sesgo de la base mecanum
            # sin calibrar). Es la que el TP pide corregir. El bring-up de
            # Fortress ya la lanza; el de Classic no, asi que en macOS el topico
            # no existia y el filtro se quedaba sin entrada -- sin ningun error.
            Node(
                package="yahboom_rosmaster_gazebo",
                executable="calculated_odometry.py",
                name="cmd_vel_odometry",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time == "true",
                    "publish_rate": 50.0,
                    "odom_frame_id": "odom",
                    "base_frame_id": "calc_base",
                    "timeout": 0.5,
                }],
            ),
            _rviz(pkg_custom, use_sim_time, want_rviz),
        ]

    pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")
    world = os.path.join(pkg_custom, "worlds", "cafe.world")
    return [
        # Fortress 6 reads the IGN_ name; everything after it reads the GZ_ one.
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", models),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz, "launch", "rosmaster_gazebo_fortress.launch.py")),
            launch_arguments={
                "world": world,
                "use_sim_time": use_sim_time,
                "rviz": "false",
            }.items(),
        ),
        _rviz(pkg_custom, use_sim_time, want_rviz),
    ]


def generate_launch_description():
    pkg_custom = get_package_share_directory("custom_map")
    default_map = os.path.join(pkg_custom, "maps", "my_map.yaml")

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
            "use_sim_time": use_sim_time,
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
        DeclareLaunchArgument("backend", default_value="auto",
                              description="auto, classic (macOS) or fortress"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Open the Gazebo window (Classic only)"),
        OpaqueFunction(function=_simulator),
        map_server,
        map_lifecycle,
        map_to_odom,
    ])
