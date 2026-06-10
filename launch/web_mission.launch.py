#!/usr/bin/env python3
"""One-command web-driven sock-fetch mission stack (simulation).

Composes the whole M6 mission stack so a fetch can be driven entirely from the
web UI (click the map in "Fetch sock" mode -> RunMission goal -> live status):

  1. mobile_grasp.launch.py (jetank_ros_main)
       gazebo (+ros2_control controllers) + MoveIt move_group + stereo perception
       + sock detector (auto configure+activate) + segmentation + grasp_server
       + base_approach + mobile_grasp_coordinator (hosts the pick Trigger service).
  2. nav2  (navigation_full.launch.py, jetank_navigation, mode:=nav2)
       NavigateToPose + AMCL + map_server against ``map``.
  3. mission_coordinator (jetank_mission)
       hosts ~/run_mission: NAVIGATE_TO_SITE -> SEARCH -> PICK ->
       NAVIGATE_TO_DEPOSIT -> DEPOSIT.
  4. web_control_node (+ cmd_vel_bridge) via web_control.launch.py sim:=true
       the browser UI; the bridge muxes teleop + Nav2 -> the sim controller.

Avoiding double-starts (gazebo/move_group/controllers/RSP):
  - mobile_grasp.launch.py ALREADY starts gazebo + gz_ros2_control controllers +
    move_group + robot_state_publisher. nav2 is added via navigation_full.launch.py
    with ``use_sim_time:=true``, which gates ALL hardware bringup (a 2nd
    robot_state_publisher, the motor driver, IMU, RPLidar) behind
    ``UnlessCondition(use_sim_time)`` and in ``mode:=nav2`` only brings up
    nav2_bringup (NavigateToPose + AMCL + map_server). We also pass ``rviz:=false``
    so it doesn't open a 2nd RViz (mobile_grasp owns MoveIt's RViz via use_rviz).
  - The startup is staggered with TimerActions: gazebo/move_group/perception
    (mobile_grasp's own internal stagger) first; nav2 at +20 s once gz clock and
    controllers are up; mission_coordinator + web at +28 s once nav2 is alive.

Usage::

    ros2 launch jetank_mission web_mission.launch.py \
        map:=$HOME/maps/sock_arena.yaml

Args:
  world           (sock_arena)                  Gazebo world (mobile_grasp).
  map             ($HOME/maps/jetank_map.yaml)  nav2 map yaml.
  model_path_sim  (/home/koen/models/sock_sim.pt)  YOLO sim model (mobile_grasp).
  use_rviz        (true)                        MoveIt RViz (mobile_grasp); nav2
                                                RViz stays off to avoid a 2nd one.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    model_path_sim = LaunchConfiguration("model_path_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    gui = LaunchConfiguration("gui")

    ros_main = FindPackageShare("jetank_ros_main")
    navigation = FindPackageShare("jetank_navigation")
    web_control = FindPackageShare("jetank_web_control")
    mission = FindPackageShare("jetank_mission")

    sim_time = {"use_sim_time": True}

    # Force Fast-DDS to UDPv4-only (disable SHM). On this dev box the SHM
    # transport intermittently wedges (open_and_lock_file failed) when the full
    # stack starts, so the late-launching nav2 nodes fail to allocate SHM ports
    # and never activate -> AMCL never localizes -> Nav2 rejects every goal.
    # Setting this first propagates to every node in this stack (incl. included
    # sub-launches). See config/fastdds_udp_only.xml.
    udp_only_profile = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([mission, "config", "fastdds_udp_only.xml"]),
    )

    def inc(pkg, rel, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", rel])
            ),
            launch_arguments=launch_args.items(),
        )

    # --- 1. Pick stack (gazebo + controllers + move_group + perception + grasp).
    # mobile_grasp internally staggers gz -> move_group -> perception -> pipeline.
    mobile_grasp = inc(
        ros_main, "mobile_grasp.launch.py",
        world=world, model_path_sim=model_path_sim, use_rviz=use_rviz, gui=gui,
    )

    # --- 2. nav2 (NavigateToPose + AMCL + map_server). use_sim_time:=true gates
    # off ALL hardware bringup + the 2nd robot_state_publisher (UnlessCondition);
    # mode:=nav2 brings up only nav2_bringup. rviz:=false: mobile_grasp owns RViz.
    # +50s: mobile_grasp's own stagger now runs to ~46s (gazebo+controllers must
    # settle before move_group/perception); nav2 starts after that to avoid a
    # startup race that left controller_manager dead under simultaneous load.
    nav2 = TimerAction(period=50.0, actions=[
        inc(navigation, "navigation_full.launch.py",
            mode="nav2", map=map_file, use_sim_time="true", rviz="false"),
    ])

    # --- 3. mission_coordinator (hosts ~/run_mission).
    coordinator = Node(
        package="jetank_mission",
        executable="mission_coordinator",
        name="mission_coordinator",
        parameters=[sim_time],
        output="screen",
    )

    # --- 4. web UI (+ cmd_vel_bridge). web_control.launch.py sim:=true wires the
    # raw-Image camera, use_sim_time, the /cmd_vel_teleop topic and the bridge.
    web = inc(web_control, "web_control.launch.py", sim="true")

    # Start the coordinator + web once nav2 has had time to come up.
    mission_and_web = TimerAction(period=60.0, actions=[coordinator, web])

    return LaunchDescription([
        udp_only_profile,
        DeclareLaunchArgument("world", default_value="sock_arena"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                os.path.expanduser("~"), "maps", "sock_arena.yaml"),
            description="Full path to the nav2 map yaml file."),
        DeclareLaunchArgument(
            "model_path_sim", default_value="/home/koen/models/sock_sim.pt"),
        # Headless defaults: this stack is driven from the browser (:8080), so the
        # Gazebo GUI + MoveIt RViz are off by default to cut load and let the
        # controller_manager come up cleanly. Pass gui:=true use_rviz:=true to see them.
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo GUI client (false => server-only)."),
        mobile_grasp,
        nav2,
        mission_and_web,
    ])
