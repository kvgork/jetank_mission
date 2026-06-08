#!/usr/bin/env python3
"""Mission coordinator for the JeTank web map-click sock-fetch mission (M2 STUB).

This is the *foundation* node for the web map-click sock-fetch mission described
in ``jetank_ros_main/plans/web-mission-plan.md``. It hosts a single, cancellable
action server ``~/run_mission`` (type ``jetank_mission/action/RunMission``) that
the web control node drives: a pick-site click on the web map becomes a goal
PoseStamped in the ``map`` frame, and the coordinator runs a thin state machine
that orchestrates the existing modular pieces purely via their ROS interfaces.

----------------------------------------------------------------------------
M2 STATUS: STUB
----------------------------------------------------------------------------
Right now this node only proves the contract: it accepts a goal, publishes one
feedback message (``state="STUB"``), then succeeds the goal with a Result of
``success=False`` / ``outcome="stub - not implemented"``. No navigation, search,
grasp or deposit happens yet. The full FSM is implemented across the M2-M5
follow-ups (see the TODO block below).

Uses an ActionServer on a ReentrantCallbackGroup under a MultiThreadedExecutor —
the same idiom as ``jetank_manipulation/grasp_server.py`` and
``mobile_grasp_coordinator.py`` — so that, once implemented, the FSM can run
nested ActionClient sub-calls (nav2, grasp) inside the execute callback while the
server stays responsive to cancellation.

Usage:
  ros2 run jetank_mission mission_coordinator --ros-args -p use_sim_time:=true
  ros2 action send_goal /mission_coordinator/run_mission \
      jetank_mission/action/RunMission "{search_timeout: 0.0}"

# TODO(M2-M5): Replace the stub execute callback with the real state machine.
#
#   NAVIGATE_TO_SITE  (M2)
#     ActionClient nav2 ``NavigateToPose`` (nav2_msgs/action/NavigateToPose) to
#     the goal ``site`` PoseStamped (map frame, from the web map click). On
#     failure -> REPORT(success=False, "navigation to site failed").
#
#   SEARCH  (M3)
#     Rotate-scan in place by publishing TwistStamped to
#     ``/diff_drive_controller/cmd_vel`` while monitoring ``/detections/socks``;
#     stop facing the sock when one is detected. Bounded by ``search_timeout``
#     (0 -> the ``default_search_timeout`` param). Timeout -> REPORT("no sock
#     found").
#
#   PICK  (M4)
#     Trigger the Phase-7 mobile-grasp pipeline:
#     ServiceClient std_srvs/srv/Trigger on
#     ``/mobile_grasp_coordinator/execute_sock_grasp``. Failure -> REPORT.
#
#   NAVIGATE_TO_DEPOSIT  (M5)
#     ActionClient nav2 ``NavigateToPose`` to the persisted ``deposit_pose``
#     (x, y, theta in the map frame; set via the M1 "Set deposit area" web click
#     and persisted to a param/file). No deposit pose set -> REPORT.
#
#   DEPOSIT  (M5)
#     Open the gripper to release the sock (control_msgs/action/GripperCommand on
#     ``/gripper_controller/gripper_cmd``; optional lower/place via grasp_server).
#
#   REPORT
#     Fill RunMission.Result(success, outcome) and ``goal_handle.succeed()``; also
#     publish ``/mission/status`` (state + outcome) for the UI. Each state
#     transition publishes a RunMission.Feedback(state=<STATE>). The server must
#     honour cancellation between/within states (check
#     ``goal_handle.is_cancel_requested`` and call ``goal_handle.canceled()``).
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from jetank_mission.action import RunMission


class MissionCoordinator(Node):
    """Hosts ~/run_mission. M2 STUB: accept -> feedback("STUB") -> succeed(fail)."""

    def __init__(self) -> None:
        super().__init__("mission_coordinator")

        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            RunMission,
            "~/run_mission",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "mission_coordinator ready (M2 STUB). Send goals to ~/run_mission."
        )

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_cb(self, goal_request):
        self.get_logger().info("RunMission goal received.")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("RunMission cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """STUB execute: publish one feedback, succeed with success=False.

        The real FSM (NAVIGATE_TO_SITE -> SEARCH -> PICK -> NAVIGATE_TO_DEPOSIT
        -> DEPOSIT -> REPORT) lands in the M2-M5 follow-ups; see the module-level
        TODO block.
        """
        self.get_logger().info("RunMission executing (STUB).")

        feedback = RunMission.Feedback()
        feedback.state = "STUB"
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = RunMission.Result()
        result.success = False
        result.outcome = "stub - not implemented"
        self.get_logger().info(f"RunMission done (STUB): {result.outcome}")
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
