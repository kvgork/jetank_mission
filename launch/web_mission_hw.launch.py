#!/usr/bin/env python3
"""Hardware sock-fetch mission stack.

One-command bringup for the full JeTank hardware fetch-sock mission, driven
entirely from the web UI (click map in "Fetch sock" mode -> RunMission goal
-> live status feed in the browser).

Hardware wiring
---------------
The topic contract is frozen across sim and hardware (see plans/sim2real-hardware-mission.md):

  web teleop    --Twist-->        /cmd_vel_teleop  --\\
  base_approach --TwistStamped--> /cmd_vel_manip   ---> [cmd_vel_bridge HW]
  mission_coord --TwistStamped--> /cmd_vel_manip   --/    --Twist--> /cmd_vel --> motor

  Nav2 velocity_smoother ---------------Twist------------------> /cmd_vel --> motor
        (owns /cmd_vel during NAVIGATE; arbiter is idle/silent then)

The mission FSM sequences NAVIGATE vs SEARCH/PICK so only one active base-motion
source publishes to /cmd_vel at a time:
  - During NAVIGATE:  Nav2 velocity_smoother owns /cmd_vel (arbiter is silent
                      because manip_topic is idle and nav_topic='' is disabled).
  - During SEARCH/PICK: base_approach/mission_coordinator publish to
                      /cmd_vel_manip; the arbiter forwards it as plain Twist to
                      /cmd_vel. Nav2 is idle (no active goal).

Teleop-during-Nav2 overlap caveat: manual web teleop during an active Nav2 goal
produces two simultaneous publishers on /cmd_vel (Nav2 smoother + arbiter
forwarding teleop). The arbiter cannot suppress Nav2's output. This is an
accepted manual-override edge case; for safety, avoid teleop during autonomous
navigation. See the bringup runbook for verification steps.

Key HW differences from sim (web_mission.launch.py)
----------------------------------------------------
- use_sim_time:=false throughout (all nodes see wall clock).
- unified.launch.py with hardware:=serial loads the JetankSerialHardware
  ros2_control backend so the arm actually actuates (not mock silent-success).
- The stereo camera runs via the real CSI pipeline, not ros_topics mode.
  NVMM hardware decode requires running against the host GStreamer libs outside
  pixi (see CLAUDE.md gotcha). The disparity/camera_info topics are identical
  to the sim path (/stereo_camera/disparity, /stereo_camera/left/camera_info),
  so sock_segmentation_server requires NO topic remaps.
- Nav2 is brought up through unified.launch.py (navigation_mode:=nav2 with
  map_file). Do NOT add a separate navigation_full.launch.py include — that
  would double-start nav2_bringup, motor driver, IMU, and RPLidar.
- cmd_vel arbiter is a standalone cmd_vel_bridge Node with output_stamped=False
  (plain Twist) and nav_topic='' (Nav2 owns /cmd_vel; arbiter must not subscribe
  its own output topic to avoid a feedback loop).
- Camera frame_id override: stereo_camera_config.yaml default is camera_*_link
  (non-optical); disparity reprojection needs the optical frame for correct 3D
  centroids. unified.launch.py defaults left_frame_id/right_frame_id to the
  optical frames and forwards them to stereo_camera.launch.py's launch args,
  which override frames.left_frame_id/frames.right_frame_id on the node. No
  on-Jetson config edit is needed — the optical frames are wired by default.

Timer periods (hardware-tunable greenfield estimates)
-----------------------------------------------------
These are NOT validated on real hardware. Adjust after first bringup:
  unified at t=0:   urdf, motor, camera, imu, lidar, moveit, nav2.
                    On real HW, camera init + lidar spin-up + nav2 AMCL can
                    take 20-40 s. Monitor /scan, /odom, /amcl_pose.
  detector at +30s: model load on Jetson (~5-15 s for .pt; faster for .engine).
  pipeline at +40s: segmentation, grasp_server, base_approach, coordinator.
  lc_configure at +50s, lc_activate at +55s: lifecycle for /sock_detector.
  mission+web at +60s: mission_coordinator, web UI, arbiter.
  Tune down once real startup times are profiled.

Usage
-----
  ros2 launch jetank_mission web_mission_hw.launch.py \\
      map:=$HOME/maps/sock_arena.yaml \\
      model_path_real:=$HOME/models/sock_real.pt

Args
----
  map:            Full path to nav2 map YAML (required). Default:
                  ~/maps/sock_arena.yaml.
  model_path_real Path to trained real-camera YOLO model. Default:
                  ~/models/sock_real.pt.
  confidence:     Detection confidence threshold. Default: 0.3.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Launch argument configurations ----
    map_file = LaunchConfiguration("map")
    model_path_real = LaunchConfiguration("model_path_real")
    confidence = LaunchConfiguration("confidence")

    # ---- Package share references ----
    ros_main = FindPackageShare("jetank_ros_main")
    detection = FindPackageShare("jetank_detection")
    manipulation = FindPackageShare("jetank_manipulation")
    web_control_pkg = FindPackageShare("jetank_web_control")
    mission_pkg = FindPackageShare("jetank_mission")

    # Hardware nodes run with wall clock (use_sim_time=False).
    hw_time = {"use_sim_time": False}

    # Force Fast-DDS to UDPv4-only (disable SHM). Mirrors web_mission.launch.py.
    # SHM intermittently wedges (open_and_lock_file failed) when the full stack
    # starts simultaneously; late-launching nodes fail to allocate SHM ports.
    # This env var propagates to every node in the launch tree.
    udp_only_profile = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([mission_pkg, "config", "fastdds_udp_only.xml"]),
    )

    def inc(pkg, rel, **launch_args):
        """Shorthand IncludeLaunchDescription helper (mirrors web_mission pattern)."""
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", rel])
            ),
            launch_arguments=launch_args.items(),
        )

    # =========================================================================
    # 1. Core hardware stack via unified.launch.py
    #    Brings up: urdf, motor controller, stereo_camera (real CSI path),
    #               IMU, RPLidar, moveit_bringup (serial backend), nav2_bringup.
    #    web_control is disabled here — we bring up the web node + arbiter
    #    separately in step 4 so we can inject the HW-specific arbiter params.
    #
    #    hardware:=serial   -> JetankSerialHardware ros2_control backend (real
    #                          servos). Without this, arm moves are mock
    #                          silent-success and no servo physically moves.
    #    left_frame_id / right_frame_id: declared in unified.launch.py for
    #    documentation; actual frame override requires editing
    #    stereo_camera_config.yaml on the Jetson until jetank_perception exposes
    #    these as proper launch args (see module docstring above).
    # =========================================================================
    unified = inc(
        ros_main, "unified.launch.py",
        use_sim_time="false",
        enable_moveit="true",
        enable_navigation="true",
        navigation_mode="nav2",
        map_file=map_file,
        enable_web_control="false",
        hardware="serial",
        left_frame_id="camera_left_optical_frame",
        right_frame_id="camera_right_optical_frame",
    )

    # =========================================================================
    # 2. Detection (hardware camera model, continuous streaming detections).
    #    detect_real.launch.py pins sim:=false so the node loads model_path_real.
    #    continuous:=true keeps the detector publishing /detections/socks so the
    #    segmentation server always has fresh detections during SEARCH.
    #
    #    Timer: +30s to allow the stereo camera CSI pipeline to initialise and
    #    the model to load on the Jetson GPU. Adjust after profiling.
    # =========================================================================
    detector = TimerAction(period=30.0, actions=[
        inc(
            detection, "detect_real.launch.py",
            model_path_real=model_path_real,
            continuous="true",
            confidence=confidence,
        )
    ])

    # =========================================================================
    # 3. Pick pipeline.
    #    sock_segmentation_server: subscribes
    #      /stereo_camera/disparity      (published by stereo_camera_node in ns)
    #      /stereo_camera/left/camera_info   (same)
    #      /detections/socks             (published by sock_detector)
    #    These match the real stereo_camera_node's published topics exactly —
    #    no remaps required.
    #
    #    base_approach_node + mission_coordinator: use cmd_vel_topic=/cmd_vel_manip
    #    (TwistStamped). The HW arbiter forwards /cmd_vel_manip -> /cmd_vel (Twist)
    #    when the manip source is fresh. Default topic in both nodes is
    #    /diff_drive_controller/cmd_vel (sim); overriding here via param only —
    #    NO code changes to either node.
    #
    #    mobile_grasp_coordinator: triggers grasp_server + base_approach in
    #    sequence. No cmd_vel param needed (it delegates to base_approach).
    #
    #    Timer: +40s (after detector loads and camera pipeline is stable).
    # =========================================================================
    seg = Node(
        package="jetank_perception",
        executable="sock_segmentation_server",
        name="sock_segmentation_server",
        parameters=[hw_time],
        output="screen",
    )
    grasp = Node(
        package="jetank_manipulation",
        executable="grasp_server",
        name="grasp_server",
        parameters=[hw_time],
        output="screen",
    )
    # base_approach_node: override cmd_vel_topic to /cmd_vel_manip (TwistStamped).
    # The HW arbiter subscribes this and republishes as plain Twist on /cmd_vel.
    approach = Node(
        package="jetank_manipulation",
        executable="base_approach_node",
        name="base_approach_node",
        parameters=[hw_time, {"cmd_vel_topic": "/cmd_vel_manip"}],
        output="screen",
    )
    grasp_coordinator = Node(
        package="jetank_manipulation",
        executable="mobile_grasp_coordinator",
        name="mobile_grasp_coordinator",
        parameters=[hw_time],
        output="screen",
    )
    pipeline = TimerAction(period=40.0, actions=[seg, grasp, approach, grasp_coordinator])

    # =========================================================================
    # Lifecycle: configure + activate /sock_detector.
    # Manual `ros2 lifecycle set` mirrors the proven sim mobile_grasp.launch.py —
    # no competing lifecycle_manager touches /sock_detector (nav2's manager has an
    # explicit node_names list of nav2 nodes only; moveit manages controllers).
    # Ordering guarantee: activate (+55s) precedes mission_coordinator + web UI
    # (+60s), so no RunMission/PICK goal can be issued before the detector is
    # active. Timer periods mirror mobile_grasp relative to pipeline start:
    #   pipeline at +40s -> configure at +50s -> activate at +55s.
    # Adjust if the detector model load time on real HW exceeds the +30->+55s budget.
    # =========================================================================
    lc_configure = TimerAction(period=50.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/sock_detector", "configure"],
            output="screen",
        )
    ])
    lc_activate = TimerAction(period=55.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/sock_detector", "activate"],
            output="screen",
        )
    ])

    # =========================================================================
    # 4. Mission coordinator + web UI + HW cmd_vel arbiter.
    #
    #    mission_coordinator: cmd_vel_topic=/cmd_vel_manip so SEARCH rotation
    #    commands go to the arbiter, not the old sim diff_drive controller topic.
    #
    #    web_control.launch.py sim:=false: web UI subscribes
    #    /stereo_camera/left/image_raw/compressed (compressed on real camera),
    #    publishes teleop Twist on /cmd_vel_teleop (cmd_vel_topic arg).
    #    The web_control.launch.py sim-mode bridge (condition=IfCondition(sim))
    #    does NOT start — we supply our own HW-specific arbiter below.
    #
    #    HW cmd_vel arbiter (cmd_vel_bridge Node):
    #      output_stamped = False    -> plain Twist on output_topic (motor expects Twist)
    #      output_topic   = /cmd_vel -> motor's real input topic
    #      teleop_topic   = /cmd_vel_teleop  -> web UI Twist
    #      manip_topic    = /cmd_vel_manip   -> base_approach + mission_coord TwistStamped
    #      nav_topic      = ''               -> DISABLED; Nav2 owns /cmd_vel directly
    #                                          during NAVIGATE. If nav_topic were set to
    #                                          /cmd_vel, the arbiter would subscribe its
    #                                          own output (feedback loop).
    #      use_sim_time   = False
    #
    #    Timer: +60s to allow nav2 AMCL to localise before mission goals are accepted.
    # =========================================================================
    coordinator = Node(
        package="jetank_mission",
        executable="mission_coordinator",
        name="mission_coordinator",
        parameters=[hw_time, {"cmd_vel_topic": "/cmd_vel_manip"}],
        output="screen",
    )

    web = inc(
        web_control_pkg, "web_control.launch.py",
        sim="false",
        cmd_vel_topic="/cmd_vel_teleop",
    )

    # HW arbiter: muxes teleop (Twist) + manip (TwistStamped) -> /cmd_vel (Twist).
    # nav_topic='' disables the nav subscription (Nav2 owns /cmd_vel directly).
    arbiter = Node(
        package="jetank_web_control",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        parameters=[{
            "use_sim_time": False,
            "output_stamped": False,
            "output_topic": "/cmd_vel",
            "teleop_topic": "/cmd_vel_teleop",
            "manip_topic": "/cmd_vel_manip",
            "nav_topic": "",
        }],
        output="screen",
    )

    mission_and_web = TimerAction(period=60.0, actions=[coordinator, web, arbiter])

    # =========================================================================
    # Launch description
    # =========================================================================
    return LaunchDescription([
        # Fast-DDS SHM workaround (same as sim stack)
        udp_only_profile,

        # ---- Declare args ----
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                os.path.expanduser("~"), "maps", "sock_arena.yaml"
            ),
            description=(
                "Full path to the nav2 map YAML (required). "
                "Example: $HOME/maps/sock_arena.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "model_path_real",
            default_value=os.path.join(
                os.path.expanduser("~"), "models", "sock_real.pt"
            ),
            description=(
                "Path to the trained real-camera YOLO sock model (.pt or .engine). "
                "Collect a real-camera dataset and train before first mission run."
            ),
        ),
        DeclareLaunchArgument(
            "confidence",
            default_value="0.3",
            description="Detection confidence threshold (0.0-1.0). Tune on real data.",
        ),

        # ---- Hardware stack ----
        unified,

        # ---- Detection (staggered: camera must be up first) ----
        detector,

        # ---- Pick pipeline (staggered: detector must be up first) ----
        pipeline,

        # ---- Lifecycle management for sock_detector ----
        lc_configure,
        lc_activate,

        # ---- Mission + web + arbiter (staggered: nav2 AMCL must localise first) ----
        mission_and_web,
    ])
