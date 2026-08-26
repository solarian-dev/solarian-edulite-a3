"""EDULITE A3 6-DOF arm module (Robstride joints + Mink IK)."""

from __future__ import annotations

import logging
import time

import numpy as np

from solarian.core.arm_module import ArmModule
from solarian.core.data import PosData
from solarian.core.ik import CartesianIKActuator, MinkIK
from solarian_edulite_a3 import ARM_MJCF_PATH
from solarian_robstride.actuator import RobstrideServo

logger = logging.getLogger(__name__)

# Same names as SO-106. CAN IDs 1-6 stay in YAML; MinkIK maps 1:1 onto MJCF.
IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]


class EduliteA3ArmModule(ArmModule):
    """EL-A3 arm: six Robstride joints plus Cartesian IK via mink / MuJoCo."""

    shoulder_pan = RobstrideServo()
    shoulder_lift = RobstrideServo()
    elbow_flex = RobstrideServo()
    wrist_flex = RobstrideServo()
    wrist_yaw = RobstrideServo()
    wrist_roll = RobstrideServo()

    x = CartesianIKActuator()
    y = CartesianIKActuator()
    z = CartesianIKActuator()
    rx = CartesianIKActuator()
    ry = CartesianIKActuator()
    rz = CartesianIKActuator()

    ik_backend = MinkIK(
        mjcf=ARM_MJCF_PATH,
        joints=dict(zip(IK_JOINTS, IK_JOINTS)),
        frame="gripper",
        position_cost=1.0,
        orientation_cost=1.0,
    )

    @property
    def home_q(self):
        """Mechanical zero in radians (after Type 6 set-zero)."""
        return np.zeros(len(self._ik_joint_names()), dtype=float)

    def initialize(self):
        super().initialize()
        self.disable_torque(IK_JOINTS)
        self.configure_components(
            {
                "run_mode": 5,  # CSP
                "limit_spd": 4.0,
            },
            component_names=IK_JOINTS,
        )
        self.enable_torque(IK_JOINTS)
        time.sleep(0.05)
        self._hold_present_pose()

    def _hold_present_pose(self) -> None:
        present = self.read_sensors(PosData, IK_JOINTS)
        self.module_bus.write(
            {name: PosData(value=present[name]["value"]) for name in IK_JOINTS},
            normalize=True,
        )

    def enter_safe_hold(self) -> None:
        self._hold_present_pose()
        self.enable_torque(IK_JOINTS)

    def on_arm(self) -> None:
        self._hold_present_pose()
        self.enable_torque(IK_JOINTS)

    def on_disarm(self) -> None:
        pass
