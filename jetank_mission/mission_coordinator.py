#!/usr/bin/env python3
"""Mission coordinator for the JeTank web map-click sock-fetch mission (M2-M5).

A *thin*, cancellable state machine that ORCHESTRATES the existing modular pieces
purely via their ROS interfaces — it owns no navigation, perception, motion or
grasp logic of its own. It hosts a single action server ``~/run_mission`` (type
``jetank_mission/action/RunMission``) which the web control node drives: a
pick-site click on the web map becomes a goal ``PoseStamped`` in the ``map``
frame, and the coordinator runs:

  1. NAVIGATE_TO_SITE   ActionClient nav2 ``NavigateToPose`` (``nav_action`` param,
                        default ``/navigate_to_pose``) to ``goal.site`` (already a
                        map-frame PoseStamped). Nav failure -> fail.
  2. SEARCH             Rotate in place: publish ``TwistStamped`` on
                        ``cmd_vel_topic`` (default ``/diff_drive_controller/cmd_vel``)
                        at ``search_omega`` rad/s while watching ``/detections/socks``
                        (``vision_msgs/Detection2DArray``, latest-cached). Stop (zero
                        Twist) as soon as a detection with ``score >= min_score``
                        appears. Timeout = ``goal.search_timeout`` (if >0) else the
                        ``search_timeout`` param -> stop + fail "no sock found".
                        (v1: rotate-in-place only.)
  3. PICK               ServiceClient ``std_srvs/srv/Trigger`` on
                        ``/mobile_grasp_coordinator/execute_sock_grasp``
                        (``pick_service`` param). That service runs the whole
                        detect->approach->preset-grasp. Failure -> fail.
  4. NAVIGATE_TO_DEPOSIT Read the persisted deposit pose from
                        ``~/.jetank/deposit_pose.json`` (``deposit_file`` param,
                        JSON ``{"x":..,"y":..}`` written by web_control M1). Missing
                        / unset -> fail "no deposit area set". Else ``NavigateToPose``
                        to ``(x, y)`` in ``map`` (orientation w=1).
  5. DEPOSIT            Open the gripper to release:
                        ActionClient ``control_msgs/action/GripperCommand`` on
                        ``/gripper_controller/gripper_cmd`` (``gripper_action`` param),
                        ``position=gripper_open_position`` (default 0.04),
                        ``max_effort=5.0``.
  6. REPORT/DONE        Result ``success=True``, ``outcome`` summarising; return to
                        idle for the next goal.

Any step's missing server / timeout / failure -> publish a terminal feedback,
return ``Result(success=False, outcome=<reason>)``, leave the base stopped. The
server honours cancellation between states (``goal_handle.is_cancel_requested``).

Uses ActionClients + a ServiceClient on a ReentrantCallbackGroup under a
MultiThreadedExecutor (the same idiom as ``mobile_grasp_coordinator.py``): each
sub-call sends the goal then spins the executor until the future resolves or a
per-step timeout elapses.

Pure transition logic is factored into module-level helpers
(``parse_deposit_file``, ``detection_passes``, ``resolve_search_timeout``,
``site_frame_or_default``) so it is unit-testable with no ROS — see
``test/test_mission_fsm.py``.

Usage:
  ros2 run jetank_mission mission_coordinator --ros-args -p use_sim_time:=true
  ros2 action send_goal /mission_coordinator/run_mission \
      jetank_mission/action/RunMission \
      "{site: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, \
        orientation: {w: 1.0}}}, search_timeout: 0.0}"
"""

import json
import os
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand
from vision_msgs.msg import Detection2DArray

from jetank_mission.action import RunMission


# ---------------------------------------------------------------------------
# Pure, ROS-free helpers (unit-tested in test/test_mission_fsm.py)
# ---------------------------------------------------------------------------


def parse_deposit_file(text):
    """Parse the deposit-pose JSON ``{"x":..,"y":..}`` written by web_control.

    Returns ``(x, y)`` floats on success, or ``None`` when the text is empty,
    not valid JSON, or missing/non-numeric ``x``/``y`` keys. Never raises.
    """
    if text is None:
        return None
    text = text.strip()
    if not text:
        return None
    try:
        data = json.loads(text)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    try:
        x = float(data["x"])
        y = float(data["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return (x, y)


def detection_passes(msg, min_score):
    """Return True if *msg* (Detection2DArray) holds a detection scoring >= min_score.

    Tolerant of the message shape: each detection's ``results`` list carries
    ``ObjectHypothesisWithPose`` rows whose ``hypothesis.score`` (Humble) — or
    legacy ``.score`` — is the confidence. Empty / malformed -> False. Never raises.
    """
    if msg is None:
        return False
    detections = getattr(msg, "detections", None)
    if not detections:
        return False
    for det in detections:
        results = getattr(det, "results", None)
        if not results:
            continue
        for res in results:
            score = None
            hyp = getattr(res, "hypothesis", None)
            if hyp is not None:
                score = getattr(hyp, "score", None)
            if score is None:
                score = getattr(res, "score", None)
            if score is None:
                continue
            try:
                if float(score) >= float(min_score):
                    return True
            except (TypeError, ValueError):
                continue
    return False


def resolve_search_timeout(goal_timeout, default_timeout):
    """Per-goal search timeout: the goal value if > 0, else the param default."""
    try:
        gt = float(goal_timeout)
    except (TypeError, ValueError):
        gt = 0.0
    if gt > 0.0:
        return gt
    return float(default_timeout)


def site_frame_or_default(frame_id, default="map"):
    """Frame for the site goal: the PoseStamped's frame if set, else *default*."""
    if frame_id is None:
        return default
    frame_id = frame_id.strip()
    return frame_id if frame_id else default


def expand_user_path(path):
    """Expand ``~`` and env vars in *path* (pure; mirrors os.path semantics)."""
    return os.path.expanduser(os.path.expandvars(path))


# ---------------------------------------------------------------------------
# Mission coordinator node (the FSM)
# ---------------------------------------------------------------------------


class MissionCoordinator(Node):
    """Hosts ~/run_mission: NAVIGATE_TO_SITE -> SEARCH -> PICK ->
    NAVIGATE_TO_DEPOSIT -> DEPOSIT -> REPORT."""

    def __init__(self) -> None:
        super().__init__("mission_coordinator")

        # --- Parameters --------------------------------------------------
        self.declare_parameter("min_score", 0.3)
        self.declare_parameter("search_timeout", 20.0)
        self.declare_parameter("search_omega", 0.5)
        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter(
            "pick_service", "/mobile_grasp_coordinator/execute_sock_grasp"
        )
        self.declare_parameter("gripper_action", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open_position", 0.04)
        self.declare_parameter("deposit_file", "~/.jetank/deposit_pose.json")
        self.declare_parameter("detections_topic", "/detections/socks")
        # Per-step timeouts (s).
        self.declare_parameter("nav_timeout_s", 120.0)
        self.declare_parameter("pick_timeout_s", 120.0)
        self.declare_parameter("deposit_nav_timeout_s", 120.0)
        self.declare_parameter("gripper_timeout_s", 15.0)
        self.declare_parameter("server_wait_timeout_s", 5.0)

        self._cb_group = ReentrantCallbackGroup()

        # --- Clients -----------------------------------------------------
        self._nav_client = ActionClient(
            self, NavigateToPose,
            self.get_parameter("nav_action").value,
            callback_group=self._cb_group,
        )
        self._gripper_client = ActionClient(
            self, GripperCommand,
            self.get_parameter("gripper_action").value,
            callback_group=self._cb_group,
        )
        self._pick_client = self.create_client(
            Trigger,
            self.get_parameter("pick_service").value,
            callback_group=self._cb_group,
        )

        # --- cmd_vel (SEARCH rotate-in-place) ----------------------------
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, self.get_parameter("cmd_vel_topic").value, 10
        )

        # --- detections subscription (latest-cached) ---------------------
        self._latest_detection = None
        self._detection_sub = self.create_subscription(
            Detection2DArray,
            self.get_parameter("detections_topic").value,
            self._on_detection,
            10,
            callback_group=self._cb_group,
        )

        # --- /mission/status (latched, for the UI) -----------------------
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/mission/status", latched_qos
        )

        # --- Action server ----------------------------------------------
        self._action_server = ActionServer(
            self,
            RunMission,
            "~/run_mission",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self._publish_status("IDLE")
        self.get_logger().info(
            "mission_coordinator ready. Send goals to ~/run_mission."
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_detection(self, msg):
        self._latest_detection = msg

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_cb(self, _goal_request):
        self.get_logger().info("RunMission goal received.")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().info("RunMission cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """Run the FSM. Each transition publishes feedback(state=<NAME>); each
        terminal path returns a Result and leaves the base stopped."""
        goal = goal_handle.request
        self.get_logger().info("RunMission executing.")

        # ---- 1. NAVIGATE_TO_SITE ---------------------------------------
        if self._cancelled(goal_handle):
            return self._canceled(goal_handle)
        self._enter_state(goal_handle, "NAVIGATE_TO_SITE")
        site = self._site_goal(goal.site)
        timeout_s = float(self.get_parameter("nav_timeout_s").value)
        ok, msg = self._navigate(site, timeout_s)
        if not ok:
            return self._fail(goal_handle, f"NAVIGATE_TO_SITE failed: {msg}")

        # ---- 2. SEARCH --------------------------------------------------
        if self._cancelled(goal_handle):
            return self._canceled(goal_handle)
        self._enter_state(goal_handle, "SEARCH")
        ok, msg = self._search(goal_handle, goal.search_timeout)
        if not ok:
            # _search has already stopped the base.
            return self._fail(goal_handle, f"SEARCH failed: {msg}")

        # ---- 3. PICK ----------------------------------------------------
        if self._cancelled(goal_handle):
            return self._canceled(goal_handle)
        self._enter_state(goal_handle, "PICK")
        ok, msg = self._pick()
        if not ok:
            return self._fail(goal_handle, f"PICK failed: {msg}")

        # ---- 4. NAVIGATE_TO_DEPOSIT ------------------------------------
        if self._cancelled(goal_handle):
            return self._canceled(goal_handle)
        self._enter_state(goal_handle, "NAVIGATE_TO_DEPOSIT")
        deposit = self._deposit_goal()
        if deposit is None:
            return self._fail(
                goal_handle,
                "NAVIGATE_TO_DEPOSIT failed: no deposit area set "
                f"({self.get_parameter('deposit_file').value}).",
            )
        timeout_s = float(self.get_parameter("deposit_nav_timeout_s").value)
        ok, msg = self._navigate(deposit, timeout_s)
        if not ok:
            return self._fail(goal_handle, f"NAVIGATE_TO_DEPOSIT failed: {msg}")

        # ---- 5. DEPOSIT -------------------------------------------------
        if self._cancelled(goal_handle):
            return self._canceled(goal_handle)
        self._enter_state(goal_handle, "DEPOSIT")
        ok, msg = self._deposit_open_gripper()
        if not ok:
            return self._fail(goal_handle, f"DEPOSIT failed: {msg}")

        # ---- 6. REPORT / DONE ------------------------------------------
        self._enter_state(goal_handle, "DONE")
        goal_handle.succeed()
        result = RunMission.Result()
        result.success = True
        result.outcome = (
            "Mission complete: navigated to site, found and picked a sock, "
            "navigated to the deposit area and released it."
        )
        self.get_logger().info(f"RunMission DONE: {result.outcome}")
        return result

    # ------------------------------------------------------------------
    # FSM steps (each returns gracefully on failure)
    # ------------------------------------------------------------------

    def _navigate(self, pose: PoseStamped, timeout_s: float):
        """Send a nav2 NavigateToPose goal; return (ok, message)."""
        wait_s = float(self.get_parameter("server_wait_timeout_s").value)
        if not self._nav_client.wait_for_server(timeout_sec=wait_s):
            return False, (
                f"nav action '{self.get_parameter('nav_action').value}' "
                "not available."
            )
        goal = NavigateToPose.Goal()
        goal.pose = pose
        result = self._send_and_wait(self._nav_client, goal, timeout_s, "navigate")
        if result is None:
            return False, "nav goal rejected or timed out."
        # nav2 reports success purely by a returned result; NavigateToPose.Result
        # is empty, so a non-None wrapped result with SUCCEEDED status is success.
        return True, "reached goal."

    def _search(self, goal_handle, goal_search_timeout):
        """Rotate in place watching /detections/socks; (ok, message).

        Stops the base (zero Twist) before returning on every path.
        """
        min_score = float(self.get_parameter("min_score").value)
        omega = float(self.get_parameter("search_omega").value)
        default_timeout = float(self.get_parameter("search_timeout").value)
        timeout_s = resolve_search_timeout(goal_search_timeout, default_timeout)

        # Clear any stale detection so we only react to fresh ones during SEARCH.
        self._latest_detection = None

        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return False, "cancelled."
                if detection_passes(self._latest_detection, min_score):
                    self.get_logger().info("SEARCH: sock detected; stopping.")
                    return True, "sock in view."
                if self.get_clock().now().nanoseconds > deadline:
                    return False, f"no sock found within {timeout_s:.1f}s."
                self._publish_twist(omega)
                time.sleep(0.05)
        finally:
            self._publish_twist(0.0)
        return False, "search aborted."

    def _pick(self):
        """Call the mobile-grasp Trigger service; return (ok, message)."""
        wait_s = float(self.get_parameter("server_wait_timeout_s").value)
        if not self._pick_client.wait_for_service(timeout_sec=wait_s):
            return False, (
                f"pick service '{self.get_parameter('pick_service').value}' "
                "not available."
            )
        timeout_s = float(self.get_parameter("pick_timeout_s").value)
        future = self._pick_client.call_async(Trigger.Request())
        if not self._spin_until(future, timeout_s):
            return False, "pick service call timed out."
        response = future.result()
        if response is None:
            return False, "pick service returned no response."
        return bool(response.success), response.message or "grasp pipeline finished."

    def _deposit_goal(self):
        """Read the persisted deposit pose; return a map-frame PoseStamped or None."""
        path = expand_user_path(self.get_parameter("deposit_file").value)
        try:
            with open(path, "r") as fh:
                text = fh.read()
        except (OSError, IOError) as exc:
            self.get_logger().warn(f"Deposit file '{path}' unreadable: {exc}")
            return None
        xy = parse_deposit_file(text)
        if xy is None:
            self.get_logger().warn(f"Deposit file '{path}' has no valid x/y.")
            return None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f"NAVIGATE_TO_DEPOSIT: deposit pose ({xy[0]:.3f}, {xy[1]:.3f}) in map."
        )
        return pose

    def _deposit_open_gripper(self):
        """Open the gripper via GripperCommand to release the sock; (ok, message)."""
        wait_s = float(self.get_parameter("server_wait_timeout_s").value)
        if not self._gripper_client.wait_for_server(timeout_sec=wait_s):
            return False, (
                f"gripper action '{self.get_parameter('gripper_action').value}' "
                "not available."
            )
        goal = GripperCommand.Goal()
        goal.command.position = float(
            self.get_parameter("gripper_open_position").value
        )
        goal.command.max_effort = 5.0
        timeout_s = float(self.get_parameter("gripper_timeout_s").value)
        result = self._send_and_wait(
            self._gripper_client, goal, timeout_s, "gripper"
        )
        if result is None:
            return False, "gripper goal rejected or timed out."
        return True, "gripper opened."

    # ------------------------------------------------------------------
    # Goal builders
    # ------------------------------------------------------------------

    def _site_goal(self, site: PoseStamped) -> PoseStamped:
        """Return the site goal with a defaulted frame_id and fresh stamp."""
        pose = PoseStamped()
        pose.header.frame_id = site_frame_or_default(site.header.frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = site.pose
        return pose

    # ------------------------------------------------------------------
    # cmd_vel
    # ------------------------------------------------------------------

    def _publish_twist(self, omega: float) -> None:
        """Publish a TwistStamped with angular.z = omega (zero linear)."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.get_parameter("base_frame").value
        twist.twist.angular.z = float(omega)
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Generic action send/wait (mirrors mobile_grasp_coordinator)
    # ------------------------------------------------------------------

    def _send_and_wait(self, client, goal, timeout_s, label):
        """Send *goal* on *client*, spin until result/timeout. Result or None."""
        send_future = client.send_goal_async(goal)
        if not self._spin_until(send_future, timeout_s):
            self.get_logger().warn(f"{label}: timed out waiting for goal acceptance.")
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"{label}: goal was rejected.")
            return None
        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future, timeout_s):
            self.get_logger().warn(f"{label}: timed out waiting for result.")
            return None
        wrapped = result_future.result()
        return wrapped.result if wrapped is not None else None

    def _spin_until(self, future, timeout_s: float) -> bool:
        """Block until *future* completes or *timeout_s* elapses (executor spins)."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and not future.done():
            if self.get_clock().now().nanoseconds > deadline:
                return False
            time.sleep(0.02)
        return future.done()

    # ------------------------------------------------------------------
    # State / feedback / terminal helpers
    # ------------------------------------------------------------------

    def _enter_state(self, goal_handle, state: str) -> None:
        self.get_logger().info(f"[state] -> {state}")
        feedback = RunMission.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)
        self._publish_status(state)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _cancelled(self, goal_handle) -> bool:
        return goal_handle.is_cancel_requested

    def _canceled(self, goal_handle):
        """Cancel cleanly: stop the base, mark canceled, return a Result."""
        self._publish_twist(0.0)
        self.get_logger().info("RunMission cancelled.")
        self._enter_state(goal_handle, "CANCELLED")
        goal_handle.canceled()
        result = RunMission.Result()
        result.success = False
        result.outcome = "Mission cancelled."
        return result

    def _fail(self, goal_handle, message: str):
        """Terminal failure: stop the base, publish a terminal feedback, abort."""
        self._publish_twist(0.0)
        self.get_logger().warn(message)
        self._enter_state(goal_handle, "FAILED")
        goal_handle.abort()
        result = RunMission.Result()
        result.success = False
        result.outcome = message
        return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
