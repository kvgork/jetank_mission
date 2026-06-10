# Plan — Reject concurrent RunMission goals (single mission at a time)

## Problem (observed 2026-06-10)
The arm started a grasp "while searching." Root cause: **two fetch missions ran
concurrently.** A prior RunMission was mid-pick (its grasp arm sequence running:
`grasp_pre → open → grasp_reach → close`) when a NEW fetch goal was clicked on the
web map. The new mission started NAVIGATE→SEARCH while the old mission's grasp was
still executing → arm grabs during the new mission's search.

## Why it happens
`mission_coordinator` (`jetank_mission/jetank_mission/mission_coordinator.py`):
- `_goal_cb` returns `GoalResponse.ACCEPT` unconditionally.
- Action server runs on a `ReentrantCallbackGroup` under a `MultiThreadedExecutor`.
- ⇒ a second RunMission goal's `_execute_cb` runs **concurrently** with the
  in-flight one. Two FSMs drive the one robot (base nav from mission B, arm grasp
  from mission A) at the same time.

## Fix
Make the coordinator single-mission. Track an "active" flag and reject (or
optionally preempt) new goals while one runs.

Sketch:
```python
# __init__
self._mission_active = False

def _goal_cb(self, _req):
    if self._mission_active:
        self.get_logger().warn("RunMission rejected: a mission is already active.")
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT

def _execute_cb(self, goal_handle):
    self._mission_active = True
    try:
        ... existing FSM ...
    finally:
        self._mission_active = False
```
- Set/clear `_mission_active` in `_execute_cb` (use try/finally so it always
  clears on success, fail, or cancel).
- Web UI: on a rejected goal, surface "mission already running — cancel first".
- Optional (nicer UX): instead of REJECT, cancel the running goal then accept the
  new one (preempt). REJECT is simpler + safer for filming; do REJECT first.

## Build + test
- `colcon build --symlink-install --packages-select jetank_mission` (Python here
  is an installed COPY, not symlinked — must rebuild before restart).
- Restart stack; start a fetch, then immediately click another fetch → 2nd must
  be rejected (warn), 1st completes alone. No arm motion during the active
  mission's SEARCH.

## Notes
- Don't click a new "Fetch sock" until the current mission reaches DONE/FAILED or
  you hit Cancel — current behaviour overlaps them.
- Related: grasp_reach S2=105°, nav xy_goal_tolerance 0.25, height-gate seg.
