# jetank_mission

Thin, web-driven sock-fetch **mission orchestrator** for the JeTank. It owns no
navigation, perception, motion or grasp logic of its own — it coordinates the
existing modular packages purely through their ROS interfaces. A click on the
web map becomes a `RunMission` goal; a single cancellable FSM drives the robot to
the site, finds and picks a sock, carries it to a deposit area, and drops it.

For the full cross-package architecture (perception chain, grasp sub-FSM,
velocity mux, web UI), see [`docs/fetch-sock-mission.md`](docs/fetch-sock-mission.md).

## 🤖 What it is

`mission_coordinator` is a `MultiThreadedExecutor` node hosting one action server,
`~/run_mission` (`jetank_mission/action/RunMission`). The web control node sends
it goals; it delegates every actual capability to other packages:

- **navigation** → Nav2 `NavigateToPose`
- **the pick** → `jetank_manipulation`'s `mobile_grasp_coordinator` (a second thin
  FSM behind a `Trigger` service — this package never touches MoveIt or the arm)
- **the release** → the gripper action controller

## 🔁 The FSM

```
RunMission goal (site: map-frame PoseStamped, search_timeout)
   │
   ▼  NAVIGATE_TO_SITE   nav2 NavigateToPose → goal.site
   ▼  SEARCH             rotate (TwistStamped @ search_omega), watch
   │                     /detections/socks; once a detection scores ≥ min_score,
   │                     turn to centre its bbox within center_tol_frac of the
   │                     image centre, then succeed. Timeout → fail "no sock found".
   ▼  PICK               Trigger /mobile_grasp_coordinator/execute_sock_grasp
   │                     (delegates the whole detect→approach→grasp to jetank_manipulation)
   ▼  NAVIGATE_TO_DEPOSIT read ~/.jetank/deposit_pose.json → nav2 NavigateToPose
   │                     (missing/unset → fail "no deposit area set")
   ▼  DEPOSIT            GripperCommand open (position=gripper_open_position) to release
   ▼  DONE               Result success=true
```

Each transition publishes `RunMission.Feedback{state=<NAME>}` and a **latched**
`/mission/status` (`std_msgs/String`, transient-local QoS, so a late-joining UI
gets the current state). Any missing server / timeout / sub-failure → stop the
base, publish `FAILED` feedback, return `Result{success=false, outcome=<reason>}`.
Cancellation is honoured *between* states via `goal_handle.is_cancel_requested`
(stops the base, emits `CANCELLED`).

> **Why SEARCH centres the sock.** The 2D detector fires while a sock sits at the
> image edge (u≈0), where stereo disparity is invalid and the 3D segmentation
> reprojects 0 points → PICK fails. SEARCH rotates the base to bring the bbox
> centre into the stereo-valid region before declaring the sock found. (v1:
> rotate-in-place only — no translation/exploration.)

## 🚀 One-command launch

```bash
ros2 launch jetank_mission web_mission.launch.py \
    map:=$HOME/maps/sock_arena.yaml gui:=true use_rviz:=true
```

Headless by default (drive it from the browser at **`http://<host>:8080`**).

| Arg | Default | Meaning |
|---|---|---|
| `map` | `~/maps/sock_arena.yaml` | nav2 map yaml |
| `world` | `sock_arena` | Gazebo world (passed to mobile_grasp) |
| `model_path_sim` | `/home/koen/models/sock_sim.pt` | YOLO sim model |
| `use_rviz` | `false` | MoveIt RViz (mobile_grasp); nav2 RViz stays off |
| `gui` | `false` | Gazebo GUI client (false ⇒ server-only) |

**Staggered bringup** (`TimerAction`s avoid a startup race that otherwise kills
`controller_manager` under simultaneous load):

1. **t=0** — `mobile_grasp.launch.py` (`jetank_ros_main`): Gazebo + ros2_control
   controllers + MoveIt `move_group` + stereo perception + sock detector +
   segmentation + `grasp_server` + `base_approach` + `mobile_grasp_coordinator`.
2. **t=50 s** — `navigation_full.launch.py mode:=nav2 use_sim_time:=true rviz:=false`
   (`jetank_navigation`): `NavigateToPose` + AMCL + map_server only. `use_sim_time`
   gates off ALL hardware bringup (motor I2C, RPLidar, IMU, 2nd robot_state_publisher).
3. **t=60 s** — `mission_coordinator` + `web_control.launch.py sim:=true` (web UI +
   `cmd_vel_bridge`).

**Fast-DDS UDP-only profile.** The launch sets `FASTRTPS_DEFAULT_PROFILES_FILE`
to [`config/fastdds_udp_only.xml`](config/fastdds_udp_only.xml) first, so it
propagates to every node in the stack. The default Fast-DDS shared-memory (SHM)
transport intermittently wedges (`open_and_lock_file failed`) under full-stack
load, leaving late-launching Nav2 nodes unable to allocate SHM ports → AMCL never
activates → Nav2 rejects every goal. UDPv4 loopback sidesteps the whole class.

## 📐 `RunMission` action

```
# Goal
geometry_msgs/PoseStamped site    # map-frame pick site (from the web map click)
float32 search_timeout            # seconds to search (0 = use param default)
---
# Result
bool success
string outcome                    # human-readable end state
---
# Feedback
string state                      # current FSM state
```

## ⚙️ Parameters (`mission_coordinator`)

| Param | Default | Meaning |
|---|---|---|
| `min_score` | `0.3` | detection score gate for SEARCH |
| `search_timeout` | `20.0` | SEARCH default timeout (s) when goal value is 0 |
| `search_omega` | `0.5` | scan-rotate yaw rate (rad/s) while no sock seen |
| `center_omega` | `0.35` | yaw rate while turning to centre a seen sock |
| `image_width` | `640` | image width (px) for the centring geometry |
| `center_tol_frac` | `0.12` | centred when `|bbox_x − width/2| ≤ frac·width` |
| `cmd_vel_topic` | `/diff_drive_controller/cmd_vel` | SEARCH rotation output (TwistStamped) |
| `base_frame` | `base_link` | frame_id stamped on the SEARCH twist |
| `nav_action` | `/navigate_to_pose` | Nav2 action |
| `pick_service` | `/mobile_grasp_coordinator/execute_sock_grasp` | the pick FSM trigger |
| `gripper_action` | `/gripper_controller/gripper_cmd` | DEPOSIT release |
| `gripper_open_position` | `0.04` | open width (m) |
| `deposit_file` | `~/.jetank/deposit_pose.json` | persisted deposit pose (JSON `{"x":..,"y":..}`) |
| `detections_topic` | `/detections/socks` | `vision_msgs/Detection2DArray` (latest-cached) |
| `nav_timeout_s` / `pick_timeout_s` / `deposit_nav_timeout_s` / `gripper_timeout_s` / `server_wait_timeout_s` | 120 / 120 / 120 / 15 / 5 | per-step timeouts (s) |

## 🔌 ROS interfaces

All consumed from other packages — the coordinator hosts only `~/run_mission` and
publishes `/mission/status`.

| Direction | Interface | Type | Peer |
|---|---|---|---|
| **Server** | `~/run_mission` | action `RunMission` | Web UI (goal source) |
| **Pub** | `/mission/status` | topic `std_msgs/String` (latched) | Web UI |
| **Pub** | `/diff_drive_controller/cmd_vel` | topic `geometry_msgs/TwistStamped` | diff-drive controller (SEARCH) |
| **Sub** | `/detections/socks` | topic `vision_msgs/Detection2DArray` | sock detector |
| **Action client** | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 |
| **Action client** | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | gripper controller |
| **Service client** | `/mobile_grasp_coordinator/execute_sock_grasp` | `std_srvs/srv/Trigger` | `jetank_manipulation` pick FSM |

## 🧪 Build & test

`jetank_mission` is an **ament_cmake** package (the build type is `ament_cmake`
because it generates the `RunMission` action via `rosidl_generate_interfaces`; the
Python module + executable are installed directly from `CMakeLists.txt`).

```bash
pixi run build              # or: colcon build --packages-select jetank_mission
```

Tests are pure, ROS-free logic tests (`test/test_mission_fsm.py`) over the
module-level helpers — `parse_deposit_file`, `detection_passes`,
`resolve_search_timeout`, `site_frame_or_default`, `expand_user_path` — with heavy
ROS deps stubbed when absent, so they run in a bare env as well as post-build.

```bash
# under colcon (ament_cmake_pytest)
colcon test --packages-select jetank_mission && colcon test-result --verbose

# standalone (bare env, deps stubbed)
pixi run -- bash -c 'cd src/jetank_mission && python -m pytest test/ -q'
```

## 🗺️ Drive a fetch from the browser

1. Open `http://<host>:8080`; wait for the stack (~60 s).
2. (Once) set a deposit area: deposit mode → click the map (persists to
   `~/.jetank/deposit_pose.json`).
3. "Fetch sock" mode → click the pick site → `RunMission` starts.
4. Watch the live status line cycle NAVIGATE_TO_SITE → SEARCH → PICK →
   NAVIGATE_TO_DEPOSIT → DEPOSIT → DONE.

---

*The coordinator is intentionally thin — extend behaviour by changing the modular
nodes behind their ROS interfaces, not by adding logic to the FSM. Full
architecture: [`docs/fetch-sock-mission.md`](docs/fetch-sock-mission.md).*
