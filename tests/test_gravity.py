"""Gravity feed-forward on the EDULITE A3 model."""

from __future__ import annotations

import pickle

import mujoco
import numpy as np
import pytest

from solarian_edulite_a3 import ARM_MJCF_PATH
from solarian_edulite_a3.arm import IK_JOINTS
from solarian_edulite_a3.gravity import STOP_CLEAR, STOP_REST, GravityFeedforward

MODEL = mujoco.MjModel.from_xml_path(ARM_MJCF_PATH)
LIMITS = {name: tuple(MODEL.jnt_range[MODEL.joint(name).id]) for name in IK_JOINTS}
FOLDED = dict.fromkeys(IK_JOINTS, 0.0)
#: Upper arm horizontal, forearm folded back over it: the shoulder's worst case.
REACHING = {**FOLDED, "shoulder_lift": 1.5, "elbow_flex": 1.5}


def to_normalized(q: dict[str, float]) -> dict[str, float]:
    return {
        name: (q[name] - lo) / (hi - lo) * 200.0 - 100.0 for name, (lo, hi) in LIMITS.items()
    }


def test_folded_at_rest_loads_the_shoulder_and_elbow_only():
    tau = GravityFeedforward(LIMITS).torques(FOLDED)
    assert abs(tau["shoulder_lift"]) > 0.5
    assert abs(tau["elbow_flex"]) > 0.5
    assert tau["shoulder_pan"] == pytest.approx(0.0, abs=1e-6)
    assert tau["wrist_roll"] == pytest.approx(0.0, abs=1e-3)


def test_zero_share_sends_nothing():
    assert GravityFeedforward(LIMITS, scale=0.0)(to_normalized(REACHING)) == {}


def test_share_scales_every_torque():
    full = GravityFeedforward(LIMITS).torques(REACHING)
    half = GravityFeedforward(LIMITS, scale=0.5).torques(REACHING)
    for name in IK_JOINTS:
        assert half[name] == pytest.approx(0.5 * full[name])


def test_tool_adds_its_weight_times_the_lever_arm():
    mass, com = 0.4, np.array([0.0, 0.0, 0.07])
    bare = GravityFeedforward(LIMITS).torques(REACHING)
    loaded = GravityFeedforward(LIMITS, tool_mass_kg=mass, tool_com=com).torques(REACHING)

    data = mujoco.MjData(MODEL)
    for name, value in REACHING.items():
        data.qpos[MODEL.jnt_qposadr[MODEL.joint(name).id]] = value
    mujoco.mj_kinematics(MODEL, data)
    site = MODEL.site("flange").id
    point = data.site_xpos[site] + data.site_xmat[site].reshape(3, 3) @ com
    joint = MODEL.joint("shoulder_lift").id
    axis, anchor = data.xaxis[joint], data.xanchor[joint]
    weight = mass * MODEL.opt.gravity
    # Holding torque cancels the weight's moment about the joint axis.
    expected = -axis @ np.cross(point - anchor, weight)
    assert abs(expected) > 0.05
    assert loaded["shoulder_lift"] - bare["shoulder_lift"] == pytest.approx(expected, rel=1e-6)


def test_normalized_positions_map_onto_the_model_limits():
    gravity = GravityFeedforward(LIMITS)
    assert gravity(to_normalized(REACHING)) == pytest.approx(gravity.torques(REACHING))
    hi = {name: LIMITS[name][1] for name in IK_JOINTS}
    sent = gravity(dict.fromkeys(IK_JOINTS, 100.0))
    model = gravity.torques(hi)
    # At its upper limit gravity pulls the elbow back toward its lower one.
    assert model["elbow_flex"] > 0.5
    assert sent["elbow_flex"] == pytest.approx(model["elbow_flex"])


def test_folded_on_the_stops_sends_no_torque_to_the_joints_resting_on_them():
    gravity = GravityFeedforward(LIMITS, tool_mass_kg=0.5, tool_com=[0, 0, 0.07])
    model = gravity.torques(FOLDED)
    sent = gravity(to_normalized(FOLDED))
    assert model["elbow_flex"] > 1.0
    assert sent["shoulder_lift"] == 0.0
    assert sent["elbow_flex"] == 0.0
    # Mid-range, nothing to rest on: still held.
    assert sent["wrist_flex"] == pytest.approx(model["wrist_flex"])


@pytest.mark.parametrize(
    "gap, share",
    [
        (0.5 * STOP_REST, 0.0),
        (0.5 * (STOP_REST + STOP_CLEAR), 0.5),
        (STOP_CLEAR, 1.0),
        (0.5, 1.0),
    ],
)
def test_leaving_a_stop_fades_the_torque_back_in(gap, share):
    gravity = GravityFeedforward(LIMITS)
    q = {**FOLDED, "elbow_flex": LIMITS["elbow_flex"][0] + gap}
    sent = gravity(to_normalized(q))["elbow_flex"]
    assert sent == pytest.approx(share * gravity.torques(q)["elbow_flex"])


def test_a_partial_pose_gives_no_torque():
    partial = to_normalized(REACHING)
    del partial["wrist_flex"]
    assert GravityFeedforward(LIMITS)(partial) == {}


def test_survives_pickling_after_use():
    gravity = GravityFeedforward(LIMITS, tool_mass_kg=0.3, tool_com=[0, 0, 0.05])
    before = gravity.torques(REACHING)
    copy = pickle.loads(pickle.dumps(gravity))
    assert copy.torques(REACHING) == pytest.approx(before)
