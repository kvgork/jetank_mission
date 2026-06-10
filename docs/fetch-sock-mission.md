# Fetch-Sock Mission — Implementation Reference

How the JeTank's web-driven *fetch-sock* mission is built, end to end: click a
point on the map in the browser → the robot drives there, finds a sock, picks it
up, carries it to a deposit area, and drops it.

This document is the integration-level map. It spans **five** ROS 2 packages and
explains how the thin coordinators wire together pre-existing navigation,
perception, and manipulation pieces. Each piece is reachable on its own ROS
interface; the mission layer owns *no* navigation/perception/motion logic of its
own — it is orchestration only.

- **Audience:** anyone extending the mission, debugging a failed fetch, or
  porting it to hardware.
- **Scope:** simulation (Gazebo Fortress) is the validated target; the same
  graph runs on hardware once the drivers replace the sim sources.

---

## 1. The one-paragraph summary

A pick-site click on the web map becomes a `map`-frame `PoseStamped`. The
`mission_coordinator` (package `jetank_mission`) runs a five-state machine:
**NAVIGATE_TO_SITE → SEARCH → PICK → NAVIGATE_TO_DEPOSIT → DEPOSIT**. The PICK
state delegates the whole pick to a *second* thin FSM, the
`mobile_grasp_coordinator` (package `jetank_manipulation`), which runs
**SEGMENT → REACH_CHECK → [APPROACH] → GRASP**. Navigation is Nav2; perception is
a stereo→disparity→3D-segmentation chain; grasping is a preset MoveIt2 joint
sequence on the 4-DOF arm. All driving velocity funnels through a single
`cmd_vel_bridge` mux onto the diff-drive controller.

---

## 2. Package responsibilities

| Package | Role in the mission |
|---|---|
| `jetank_mission` | Top-level FSM (`mission_coordinator`), `RunMission` action, `web_mission.launch.py` (one-command stack), UDP-only Fast-DDS profile. |
| `jetank_web_control` | Browser UI: map-click → mission goal, live status poll, deposit-set; `cmd_vel_bridge` velocity mux. |
| `jetank_navigation` | Nav2 bringup (`NavigateToPose` + AMCL + map_server) via `navigation_full.launch.py mode:=nav2`. |
| `jetank_detection` | 2D sock detector (`/detections/socks`), `SegmentSocks` action + `SockCloud` msg definitions. |
| `jetank_perception` | `sock_segmentation_server`: disparity → reproject → ground-removal → cluster → 3D centroid. |
| `jetank_manipulation` | `mobile_grasp_coordinator` (pick FSM), `base_approach_node` (drive-to-standoff), `grasp_server` (preset MoveIt2 grasp). |

---

## 3. Top-level state machine (`mission_coordinator`)

Source: `jetank_mission/jetank_mission/mission_coordinator.py`
Action: `~/run_mission` (`jetank_mission/action/RunMission`)

```
RunMission goal (site: map-frame PoseStamped, search_timeout)
        │
        ▼
┌──────────────────┐  nav2 NavigateToPose → goal.site
│ NAVIGATE_TO_SITE │
└──────────────────┘
        │ ok
        ▼
┌──────────────────┐  rotate in place (TwistStamped @ search_omega),
│      SEARCH      │  watch /detections/socks, stop on score ≥ min_score
└──────────────────┘  (timeout → fail "no sock found")
        │ sock in view
        ▼
┌──────────────────┐  Trigger /mobile_grasp_coordinator/execute_sock_grasp
│       PICK       │  (delegates to the grasp FSM, §4)
└──────────────────┘
        │ ok
        ▼
┌────────────────────┐  read ~/.jetank/deposit_pose.json → nav2 NavigateToPose
│ NAVIGATE_TO_DEPOSIT│  (missing/unset → fail "no deposit area set")
└────────────────────┘
        │ ok
        ▼
┌──────────────────┐  GripperCommand open (position=gripper_open_position)
│     DEPOSIT      │
└──────────────────┘
        │ ok
        ▼
       DONE  (Result success=true)
```

**Design contract.** The coordinator is a *thin, cancellable* FSM. Each
transition publishes `RunMission.Feedback{state=<NAME>}` and a latched
`/mission/status` (`std_msgs/String`, transient-local QoS so a late-joining UI
gets the current state). Any missing server, timeout, or sub-failure →
`_fail()`: stop the base, publish `FAILED` feedback, return
`Result{success=false, outcome=<reason>}`. Cancellation is honoured *between*
states via `goal_handle.is_cancel_requested`.

**Concurrency idiom.** `ActionClient`s + a `ServiceClient` on a
`ReentrantCallbackGroup` under a `MultiThreadedExecutor`. Each sub-call sends the
goal, then `_spin_until(future, timeout)` blocks until the future resolves or the
per-step timeout elapses.

**Pure, unit-tested helpers** (no ROS, see `test/test_mission_fsm.py`):
`parse_deposit_file`, `detection_passes`, `resolve_search_timeout`,
`site_frame_or_default`, `expand_user_path`.

### 3.1 `RunMission` action

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

### 3.2 Parameters (`mission_coordinator`)

| Param | Default | Meaning |
|---|---|---|
| `min_score` | `0.3` | detection score gate for SEARCH |
| `search_timeout` | `20.0` | SEARCH default timeout (s) if goal value is 0 |
| `search_omega` | `0.5` | rotate-in-place yaw rate (rad/s) |
| `cmd_vel_topic` | `/diff_drive_controller/cmd_vel` | SEARCH rotation output |
| `nav_action` | `/navigate_to_pose` | Nav2 action |
| `pick_service` | `/mobile_grasp_coordinator/execute_sock_grasp` | the pick FSM trigger |
| `gripper_action` | `/gripper_controller/gripper_cmd` | DEPOSIT release |
| `gripper_open_position` | `0.04` | open width (m) |
| `deposit_file` | `~/.jetank/deposit_pose.json` | persisted deposit pose |
| `detections_topic` | `/detections/socks` | `vision_msgs/Detection2DArray` |
| `nav_timeout_s` / `pick_timeout_s` / `deposit_nav_timeout_s` / `gripper_timeout_s` / `server_wait_timeout_s` | 120 / 120 / 120 / 15 / 5 | per-step timeouts |

> **SEARCH is v1 rotate-in-place only.** It spins until a 2D detection passes
> `min_score` or the timeout trips. It does not translate or explore.

---

## 4. The pick sub-FSM (`mobile_grasp_coordinator`)

Source: `jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py`
Trigger: `~/execute_sock_grasp` (`std_srvs/srv/Trigger`)

```
Trigger
   │
   ▼
┌──────────┐  /segment_socks (SegmentSocks) → SockCloud (nearest sock, base_link)
│ SEGMENT  │  build top-down grasp pose in base_link, then REMEMBER it in `odom`
└──────────┘  (world-fixed) while the sock is still in view.
   │ found
   ▼
┌────────────┐  horiz dist(centroid, arm_base_xy) ≤ arm_reach ?
│ REACH_CHECK│  yes → skip APPROACH ; no → APPROACH
└────────────┘
   │ out of reach
   ▼
┌──────────┐  /approach_target (ApproachTarget) → drive base to `standoff` ahead
│ APPROACH │  of the sock centroid (snapshotted into odom; world-fixed).
└──────────┘
   │ arrived
   ▼
┌──────────┐  /grasp_object (GraspObject)
│  GRASP   │   preset mode (default): empty target_pose → grasp_server runs its
└──────────┘   tuned joint sequence. pose mode: recover remembered odom pose into
   │           base_link at LATEST TF, send as target_pose.
   ▼
  DONE  (Trigger response success=true)
```

### 4.1 The "remember-in-odom" open-loop trick

The camera is **rigidly mounted on the arm** and points forward with no tilt
(see `jetank_description/README.md` and the IMU-on-arm note in `CLAUDE.md`).
Consequence: once the base drives up to a floor sock at the ~0.18 m standoff, the
sock drops **below the camera FOV** and can no longer be seen. So the pipeline
**captures the grasp pose while the sock is still visible**, transforms it from
`base_link` into the world-fixed `odom` frame, and stores it. After APPROACH
moves the base, the GRASP step transforms that stored `odom` pose **back** into
`base_link` *at the latest TF* — open-loop tracking through the drive. This is
why `world_frame` (default `odom`) exists and why GRASP never re-segments.

### 4.2 REACH_CHECK geometry

`reachable = hypot(cx - arm_base_x, cy - arm_base_y) ≤ arm_reach`, with
`arm_base_xy = [0.06, 0.0]` (arm mount offset in `base_link`) and
`arm_reach = 0.22 m`. The 4-DOF arm reaches ~0.22–0.25 m, so almost any sock
spotted from the search pose is "out of reach" → APPROACH runs to centre the sock
at the standoff before the preset grasp fires.

### 4.3 Why GRASP defaults to **preset**, not Cartesian pose

A free-form Cartesian floor-grasp pose is **infeasible** on this arm: at floor
level the wrist self-collides with the arm-mounted camera, so OMPL cannot sample
a valid IK state. `grasp_mode="preset"` (default) instead fires `grasp_server`'s
SRDF named-target joint sequence
(`ready → grasp_pre → open → grasp_reach → close → ready → home`), which plans as
joint goals and is RViz-validated. `grasp_mode="pose"` exists for elevated /
genuinely reachable targets.

`grasp_server` joint presets (`grasp_server.py`): `grasp_reach` ≈ S2=100°, S3=-15°;
`grasp_pre` ≈ raised approach S2=70°.

### 4.4 Parameters (`mobile_grasp_coordinator`)

| Param | Default | Meaning |
|---|---|---|
| `min_score` | `0.3` | segmentation detection gate |
| `max_range` | `3.0` | depth clip (m) passed to SegmentSocks |
| `arm_reach` | `0.22` | reachable-envelope radius (m) |
| `arm_base_xy` | `[0.06, 0.0]` | arm mount in base_link |
| `approach_standoff` | `0.18` | stop distance ahead of sock (m) |
| `grasp_mode` | `preset` | `preset` (joint seq) or `pose` (Cartesian) |
| `target_frame` | `base_link` | segmentation/grasp frame |
| `world_frame` | `odom` | world-fixed frame to remember the grasp pose |
| `segment_timeout_s` / `approach_timeout_s` / `grasp_timeout_s` | 10 / 30 / 60 | per-step timeouts |

---

## 5. Perception chain (SEGMENT internals)

Server: `jetank_perception/src/sock_segmentation_server.cpp`
Action: `/segment_socks` (`jetank_detection/action/SegmentSocks`)

The server snapshots a synchronized triple — **disparity image** (`32FC1`),
**camera_info**, and the latest **2D detections** — then per detection:

1. **Sync gates** — reject if `disparity age > max_age` (1.0 s) or
   `|disparity_stamp − detections_stamp| > max_sync_dt` (0.5 s).
2. **Reproject** — for each detection ROI, reproject finite, in-range disparity
   pixels to 3D (`max_range` clip). Drop if `< min_points` (30).
3. **Ground removal** (`remove_ground=true`) — PCL RANSAC plane fit,
   `ground_distance_threshold = 0.02 m`. Drop if post-ground `< min_points`.
4. **Cluster** — Euclidean clustering at `cluster_tolerance = 0.05 m`; keep the
   largest cluster. Drop if `< min_points`.
5. **Result** — the surviving blob nearest `base_link` becomes the returned
   `SockCloud{cloud, centroid, dimensions, label, score}`, transformed into the
   goal's `target_frame`. `found=false` if no blob survives.

```
# SegmentSocks (jetank_detection)
# Goal:   string target_frame, float32 min_score, float32 max_range, bool publish_debug
# Result: bool found, SockCloud sock
# Feedback: uint16 processed, uint16 total

# SockCloud.msg
sensor_msgs/PointCloud2 cloud
geometry_msgs/PointStamped centroid     # in target_frame
geometry_msgs/Vector3 dimensions        # AABB size (m)
string label
float32 score
```

**Ground removal — height gate (default).** `ground_filter="height"`: RANSAC fits
the flat floor plane *in the optical frame* (no `base_link`/TF dependency), then
KEEPS points more than `ground_margin` (0.012 m) above it on the camera side via
signed plane distance. This survives a sock nearly coplanar with the floor at
range, where the legacy `ground_filter="ransac"` inlier-removal deleted the whole
blob (2507 → 0 points). With the height gate: 2507 → 176 points kept → sock found.

> **SEGMENT robustness:** SEARCH now centres the sock first (so it is in the
> stereo-valid region, not at the edge where disparity reprojects 0 points), and
> the height gate keeps a low/flat sock the plane-removal used to erase. A sock
> still needs to be within ~range and not fully occluded.

---

## 6. Driving the base (APPROACH + the velocity mux)

### 6.1 `base_approach_node`

Source: `jetank_manipulation/jetank_manipulation/base_approach_node.py`
Action: `/approach_target` (`jetank_manipulation/action/ApproachTarget`)

```
# Goal:   geometry_msgs/PointStamped target, float32 standoff, float32 timeout
# Result: bool success, float32 final_distance, string message
# Feedback: float32 distance, float32 heading_error
```

Snapshots the target into `odom` (world-fixed, so the base can actually reach
it), then a proportional controller: **rotate-to-face** first (angular only)
until heading error is small, then **drive forward** until
`dist ≤ standoff + arrive_tol`. The forward term `k_lin·(dist − standoff)` decays
to ~0 at the standoff, so the base asymptotes from above and the `arrive_tol`
band (0.03 m) is what actually trips arrival. Publishes the command as a
`TwistStamped` **directly on `/diff_drive_controller/cmd_vel`** (param
`cmd_vel_topic`).

### 6.2 `cmd_vel_bridge` — the single velocity owner

Source: `jetank_web_control/jetank_web_control/cmd_vel_bridge.py`

Three things want to drive the base: **web teleop** (joystick/WASD →
`/cmd_vel_teleop`), **Nav2** (`/cmd_vel`), and `base_approach` (direct). The
bridge muxes teleop + nav and republishes a `TwistStamped` on
`/diff_drive_controller/cmd_vel`:

- teleop fresh **and non-zero** → teleop wins (manual override priority);
- else nav fresh and non-zero → Nav2 passes through;
- else → emit a brief zero **stop-burst** (~0.3 s), then **go silent**.

> **Why it goes silent (critical).** `base_approach` publishes to the *same*
> controller topic directly. If the bridge floods continuous idle zeros, they
> interleave with base_approach's drive commands at the controller and cancel the
> motion — the base stutters in place, APPROACH times out, and **every fetch
> mission fails at PICK**. Going silent when idle lets `base_approach` own the
> topic; the diff-drive controller's own `cmd_vel_timeout` keeps the base stopped
> once every publisher is quiet. See `memory: jetank-cmdvel-bridge-base-approach-conflict`.

---

## 7. Web UI flow

Source: `jetank_web_control/jetank_web_control/web_control_node.py`

HTTP endpoints used by the fetch mission:

| Endpoint | Purpose |
|---|---|
| `POST /mission/goal` | `{ix,iy}` map-pixel → `RunMission` goal (fetch) |
| `POST /mission/deposit` | `{ix,iy}` → store + persist deposit pose |
| `GET /mission/deposit` | deposit pose `{x,y}` or `{set:false}` |
| `GET /mission/status` | latest mission status `{status, active}` |
| `POST /mission/cancel` | cancel the active goal |
| `POST /grab` | standalone `GraspObject` (grasp without a mission) |

**Map click → world.** The map panel renders `/map.png` with
`object-fit: contain`, so the rendered image is letterboxed inside the panel box.
The click handler `clickToPixel` maps the click through the **same** geometry
the overlay uses — `scale = min(w/natW, h/natH)`, content centred — so the full
map is clickable on both axes (an earlier bug let only the middle band of the
letterboxed axis be selected). The pixel is then converted to a map-frame point
by `map_pixel_to_world`, which accounts for the **vertically-flipped** served PNG
(`grid_row = (height-1) - iy`) and uses cell-centre conversion
`wx = origin_x + (ix+0.5)·res`, `wy = origin_y + (grid_row+0.5)·res`.

**Live status.** After sending a goal the UI polls `/mission/status` every 1 s and
renders the FSM state until a terminal token (`DONE/FAILED/CANCELLED/IDLE`). The
coordinator's `/mission/status` is latched (transient-local), so the value is
also available to late joiners.

**Deposit pose** is persisted as `{"x":..,"y":..}` JSON to
`~/.jetank/deposit_pose.json`, written by the web node and read back by the
coordinator's NAVIGATE_TO_DEPOSIT step.

---

## 8. Launch & run

One-command stack (simulation):

```bash
ros2 launch jetank_mission web_mission.launch.py map:=$HOME/maps/sock_arena.yaml
```

It composes, with staggered `TimerAction`s to avoid a startup race:

1. **t=0** — `mobile_grasp.launch.py` (`jetank_ros_main`): Gazebo + ros2_control
   controllers + MoveIt `move_group` + stereo perception + sock detector
   (auto-configure+activate) + `sock_segmentation_server` + `grasp_server` +
   `base_approach_node` + `mobile_grasp_coordinator`.
2. **t=50 s** — `navigation_full.launch.py mode:=nav2 use_sim_time:=true rviz:=false`
   (`jetank_navigation`): `NavigateToPose` + AMCL + map_server only. `use_sim_time`
   gates off ALL hardware bringup (motor I2C, RPLidar, IMU, a 2nd
   robot_state_publisher) via `UnlessCondition`.
3. **t=60 s** — `mission_coordinator` + `web_control.launch.py sim:=true` (web UI
   + `cmd_vel_bridge`).

Launch args: `world` (`sock_arena`), `map` (`~/maps/sock_arena.yaml`),
`model_path_sim` (`/home/koen/models/sock_sim.pt`), `use_rviz` (`false`),
`gui` (`false`). Headless by default — drive it from the browser at
**`http://<host>:8080`**.

**Fast-DDS UDP-only profile.** The launch sets
`FASTRTPS_DEFAULT_PROFILES_FILE` to `config/fastdds_udp_only.xml` because the SHM
transport intermittently wedges under the full-stack load, leaving late-launching
Nav2 nodes unable to allocate SHM ports → AMCL never activates → goals rejected.

### How to drive a fetch from the browser

1. Open `http://<host>:8080`. Wait for the stack (~60 s).
2. (Once) set a deposit area: deposit mode → click the map. Persists to
   `~/.jetank/deposit_pose.json`.
3. "Fetch sock" mode → click the pick site on the map → `RunMission` starts.
4. Watch the live status line cycle NAVIGATE_TO_SITE → SEARCH → PICK →
   NAVIGATE_TO_DEPOSIT → DEPOSIT → DONE.

---

## 9. Interface map (one glance)

| From | Interface | Type | To |
|---|---|---|---|
| Web UI | `/mission_coordinator/run_mission` | action `RunMission` | mission_coordinator |
| mission_coordinator | `/navigate_to_pose` | action `NavigateToPose` | Nav2 |
| mission_coordinator | `/detections/socks` | topic `Detection2DArray` | detector (read) |
| mission_coordinator | `/mobile_grasp_coordinator/execute_sock_grasp` | service `Trigger` | grasp FSM |
| mission_coordinator | `/gripper_controller/gripper_cmd` | action `GripperCommand` | gripper |
| mission_coordinator | `/mission/status` | topic `String` (latched) | Web UI (read) |
| mobile_grasp_coordinator | `/segment_socks` | action `SegmentSocks` | sock_segmentation_server |
| mobile_grasp_coordinator | `/approach_target` | action `ApproachTarget` | base_approach_node |
| mobile_grasp_coordinator | `/grasp_object` | action `GraspObject` | grasp_server |
| base_approach / bridge / search | `/diff_drive_controller/cmd_vel` | topic `TwistStamped` | diff-drive controller |
| Web teleop | `/cmd_vel_teleop` | topic `Twist` | cmd_vel_bridge |
| Nav2 | `/cmd_vel` | topic `Twist` | cmd_vel_bridge |

---

## 10. Known issues & gotchas

- **`arm_controller` must spawn active.** GRASP fails with
  `error_code=-4 CONTROL_FAILED` and the controller logs
  *"Can't accept new action goals. Controller is not running"* when the
  `gz_ros2_control` `arm_controller` is **inactive**. The gz spawner
  (`gazebo_sim` → `gazebo_headless`) defaults `start_arm_active=false`;
  `mobile_grasp.launch.py` must pass `start_arm_active:=true` so MoveIt can drive
  the arm (fixed). To recover a live session without relaunch:
  `ros2 control set_controller_state arm_controller active`.
- **`NameError` at PICK→DONE (FIXED).** `mobile_grasp_coordinator._on_trigger`
  previously built its success message from `gx,gy,gz` that were never assigned —
  a latent crash on the first successful grasp. Now reports the sock centroid
  `(cx,cy,cz)`, which is always in scope.
- **APPROACH-stalls = the cmd_vel mux conflict** (§6.2). If missions time out at
  APPROACH with no base motion, confirm the bridge goes silent when idle (it must
  not flood zeros onto the controller topic).
- **SEGMENT `found=false`** for edge/far socks (§5) — position-dependent.
- **Camera FOV** — floor socks leave the fixed forward camera's view at close
  range; the remember-in-odom open loop (§4.1) is the workaround, *not* a bug.
- **Two maps / double Nav2** — clicking a separate "start navigation" button while
  the mission stack is up spawns a second Nav2 on a different map; the competing
  AMCL/map_server corrupts localization. In the mission stack, Nav2 is already up
  — only use "Fetch sock".
- **AMCL localization tuning** lives staged in
  `jetank_ros_main/plans/mapping-nav-separation-plan.md` (Phase 2): sharp
  likelihood field + tight seed covariance.

---

## 11. Source index

| Concern | File |
|---|---|
| Top FSM | `jetank_mission/jetank_mission/mission_coordinator.py` |
| Top FSM tests | `jetank_mission/test/test_mission_fsm.py` |
| One-command launch | `jetank_mission/launch/web_mission.launch.py` |
| UDP-only DDS | `jetank_mission/config/fastdds_udp_only.xml` |
| Pick FSM | `jetank_manipulation/jetank_manipulation/mobile_grasp_coordinator.py` |
| Drive-to-standoff | `jetank_manipulation/jetank_manipulation/base_approach_node.py` |
| Preset grasp | `jetank_manipulation/jetank_manipulation/grasp_server.py` |
| 3D segmentation | `jetank_perception/src/sock_segmentation_server.cpp` |
| Web UI + bridge | `jetank_web_control/jetank_web_control/web_control_node.py`, `cmd_vel_bridge.py` |
| Nav2 bringup | `jetank_navigation/launch/navigation_full.launch.py`, `config/nav2/nav2_params.yaml` |
| Action/msg defs | `RunMission.action` (mission), `SegmentSocks.action` + `SockCloud.msg` (detection), `ApproachTarget.action` + `GraspObject.action` (manipulation) |

---

*Generated from source inspection of the workspace. The mission and grasp
coordinators are intentionally thin — extend behaviour by changing the modular
nodes behind their ROS interfaces, not by adding logic to the FSMs.*
