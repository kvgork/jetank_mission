"""Pure-logic tests for jetank_mission.mission_coordinator.

These load the *source* ``mission_coordinator.py`` directly by file path and
exercise its ROS-free, module-level helpers — ``parse_deposit_file``,
``detection_passes``, ``resolve_search_timeout``, ``site_frame_or_default`` and
``expand_user_path``.

The node module imports rclpy / nav2_msgs / control_msgs / vision_msgs / the
generated RunMission action at import time. To keep the helpers testable in a
bare environment (no colcon build, no ROS), we stub the absent ROS deps before
loading the module by file path — mirroring jetank_manipulation's test pattern.
When the real packages are present (post colcon build) they are used as-is.
"""

import importlib.util
import json
import os
import sys
import types

import pytest


# ---------------------------------------------------------------------------
# Stub infrastructure (mirrors jetank_manipulation/test/test_base_control.py)
# ---------------------------------------------------------------------------


def _make_stub(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _ensure(name):
    """Register *name* as a stub module iff it cannot already be imported."""
    if importlib.util.find_spec(name) is None:
        return _make_stub(name)
    return None


def _install_stubs():
    rclpy_stub = _ensure("rclpy")
    if rclpy_stub is not None:
        rclpy_stub.ok = lambda *a, **k: True
        rclpy_stub.init = lambda *a, **k: None
        rclpy_stub.shutdown = lambda *a, **k: None

        action_stub = _make_stub("rclpy.action")
        for sym in ("ActionClient", "ActionServer", "CancelResponse", "GoalResponse"):
            setattr(action_stub, sym, type(sym, (), {}))
        rclpy_stub.action = action_stub

        cbg_stub = _make_stub("rclpy.callback_groups")
        cbg_stub.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
        rclpy_stub.callback_groups = cbg_stub

        ex_stub = _make_stub("rclpy.executors")
        ex_stub.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
        rclpy_stub.executors = ex_stub

        node_stub = _make_stub("rclpy.node")
        node_stub.Node = type("Node", (), {})
        rclpy_stub.node = node_stub

        qos_stub = _make_stub("rclpy.qos")
        for sym in ("DurabilityPolicy", "HistoryPolicy", "QoSProfile",
                    "ReliabilityPolicy"):
            setattr(qos_stub, sym, type(sym, (), {}))
        rclpy_stub.qos = qos_stub

    def _msg_stub(modname, *symbols):
        if importlib.util.find_spec(modname) is None:
            mod = _make_stub(modname)
            for sym in symbols:
                setattr(mod, sym, type(sym, (), {}))

    _msg_stub("geometry_msgs.msg", "PoseStamped", "TwistStamped")
    _msg_stub("std_msgs.msg", "String")
    _msg_stub("std_srvs.srv", "Trigger")
    _msg_stub("nav2_msgs.action", "NavigateToPose")
    _msg_stub("control_msgs.action", "GripperCommand")
    _msg_stub("vision_msgs.msg", "Detection2DArray")

    if importlib.util.find_spec("jetank_mission.action") is None:
        # Build a jetank_mission package stub with an .action submodule holding a
        # RunMission whose Result/Feedback are plain settable objects.
        if importlib.util.find_spec("jetank_mission") is None:
            _make_stub("jetank_mission")
        action_mod = _make_stub("jetank_mission.action")

        class _RunMission:
            Result = type("Result", (), {})
            Feedback = type("Feedback", (), {})

        action_mod.RunMission = _RunMission


def _load_module():
    _install_stubs()
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(
        os.path.join(here, "..", "jetank_mission", "mission_coordinator.py")
    )
    spec = importlib.util.spec_from_file_location("_mc_under_test", src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


MC = _load_module()


# ---------------------------------------------------------------------------
# parse_deposit_file
# ---------------------------------------------------------------------------


def test_parse_deposit_file_valid():
    assert MC.parse_deposit_file('{"x": 1.5, "y": -2.25}') == (1.5, -2.25)


def test_parse_deposit_file_integers_coerced_to_float():
    xy = MC.parse_deposit_file('{"x": 3, "y": 4}')
    assert xy == (3.0, 4.0)
    assert isinstance(xy[0], float) and isinstance(xy[1], float)


def test_parse_deposit_file_extra_keys_ignored():
    assert MC.parse_deposit_file('{"x": 0.0, "y": 0.0, "theta": 1.57}') == (0.0, 0.0)


def test_parse_deposit_file_none():
    assert MC.parse_deposit_file(None) is None


def test_parse_deposit_file_empty_and_whitespace():
    assert MC.parse_deposit_file("") is None
    assert MC.parse_deposit_file("   \n ") is None


def test_parse_deposit_file_invalid_json():
    assert MC.parse_deposit_file("{not json}") is None


def test_parse_deposit_file_missing_keys():
    assert MC.parse_deposit_file('{"x": 1.0}') is None
    assert MC.parse_deposit_file('{"y": 1.0}') is None


def test_parse_deposit_file_non_numeric_values():
    assert MC.parse_deposit_file('{"x": "a", "y": "b"}') is None


def test_parse_deposit_file_non_object_json():
    assert MC.parse_deposit_file("[1, 2]") is None
    assert MC.parse_deposit_file("42") is None


def test_parse_deposit_file_roundtrip():
    text = json.dumps({"x": 9.81, "y": -0.5})
    assert MC.parse_deposit_file(text) == (9.81, -0.5)


# ---------------------------------------------------------------------------
# detection_passes
# ---------------------------------------------------------------------------


def _det_msg(scores, hypothesis_shape=True):
    """Build a duck-typed Detection2DArray-like object from a list of scores.

    Each score becomes one detection with a single result. ``hypothesis_shape``
    True -> Humble layout (result.hypothesis.score); False -> legacy result.score.
    """
    detections = []
    for s in scores:
        if hypothesis_shape:
            res = types.SimpleNamespace(
                hypothesis=types.SimpleNamespace(score=s)
            )
        else:
            res = types.SimpleNamespace(score=s)
        detections.append(types.SimpleNamespace(results=[res]))
    return types.SimpleNamespace(detections=detections)


def test_detection_passes_above_threshold_humble():
    assert MC.detection_passes(_det_msg([0.9]), 0.3) is True


def test_detection_passes_above_threshold_legacy():
    assert MC.detection_passes(_det_msg([0.9], hypothesis_shape=False), 0.3) is True


def test_detection_passes_below_threshold():
    assert MC.detection_passes(_det_msg([0.1]), 0.3) is False


def test_detection_passes_at_threshold_inclusive():
    assert MC.detection_passes(_det_msg([0.3]), 0.3) is True


def test_detection_passes_one_of_many_qualifies():
    assert MC.detection_passes(_det_msg([0.1, 0.05, 0.55]), 0.3) is True


def test_detection_passes_none_message():
    assert MC.detection_passes(None, 0.3) is False


def test_detection_passes_empty_detections():
    assert MC.detection_passes(types.SimpleNamespace(detections=[]), 0.3) is False


def test_detection_passes_no_detections_attr():
    assert MC.detection_passes(types.SimpleNamespace(), 0.3) is False


def test_detection_passes_empty_results():
    msg = types.SimpleNamespace(detections=[types.SimpleNamespace(results=[])])
    assert MC.detection_passes(msg, 0.3) is False


def test_detection_passes_malformed_score():
    res = types.SimpleNamespace(hypothesis=types.SimpleNamespace(score="oops"))
    msg = types.SimpleNamespace(detections=[types.SimpleNamespace(results=[res])])
    assert MC.detection_passes(msg, 0.3) is False


# ---------------------------------------------------------------------------
# resolve_search_timeout
# ---------------------------------------------------------------------------


def test_resolve_search_timeout_uses_goal_when_positive():
    assert MC.resolve_search_timeout(12.0, 20.0) == 12.0


def test_resolve_search_timeout_falls_back_on_zero():
    assert MC.resolve_search_timeout(0.0, 20.0) == 20.0


def test_resolve_search_timeout_falls_back_on_negative():
    assert MC.resolve_search_timeout(-5.0, 20.0) == 20.0


def test_resolve_search_timeout_handles_non_numeric_goal():
    assert MC.resolve_search_timeout(None, 20.0) == 20.0


# ---------------------------------------------------------------------------
# site_frame_or_default
# ---------------------------------------------------------------------------


def test_site_frame_or_default_uses_given_frame():
    assert MC.site_frame_or_default("odom") == "odom"


def test_site_frame_or_default_defaults_on_empty():
    assert MC.site_frame_or_default("") == "map"


def test_site_frame_or_default_defaults_on_whitespace():
    assert MC.site_frame_or_default("   ") == "map"


def test_site_frame_or_default_defaults_on_none():
    assert MC.site_frame_or_default(None) == "map"


def test_site_frame_or_default_custom_default():
    assert MC.site_frame_or_default("", default="base_link") == "base_link"


# ---------------------------------------------------------------------------
# expand_user_path
# ---------------------------------------------------------------------------


def test_expand_user_path_expands_home():
    assert MC.expand_user_path("~/.jetank/deposit_pose.json") == os.path.join(
        os.path.expanduser("~"), ".jetank", "deposit_pose.json"
    )


def test_expand_user_path_absolute_unchanged():
    assert MC.expand_user_path("/tmp/x.json") == "/tmp/x.json"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
