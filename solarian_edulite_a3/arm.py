"""EDULITE A3 6-DOF arm module (Robstride joints + Mink IK)."""

from __future__ import annotations

import logging
from typing import Literal

from solarian.core.actuator_module import SPEED_BACKSTOP_MARGIN
from solarian.core.arm_module import ArmModule
from solarian.core.calibration import MotorCalibration
from solarian.core.configs import ModuleConfig
from solarian.core.ik import CartesianIKActuator, MinkIK
from solarian_edulite_a3 import ARM_MJCF_PATH
from solarian_edulite_a3.gravity import GravityFeedforward
from solarian_robstride.actuator import RobstrideServo
from solarian_robstride.protocol import PROTOCOL_CPR, RunMode, params_for_model

logger = logging.getLogger(__name__)

# Same names as Solarian Arm 1. CAN IDs 1-6 stay in YAML; MinkIK maps 1:1 onto MJCF.
IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]


class EduliteA3ArmConfig(ModuleConfig):
    #: ``mit``: every tick sends each motor its target, velocity, stiffness,
    #: damping and the torque that holds the arm against gravity. ``csp``: the
    #: motors' own position loop, whose factory gains buzz under load.
    control_mode: Literal["mit", "csp"] = "mit"
    #: MIT stiffness (Nm/rad) and damping (Nm·s/rad): one value for every joint,
    #: or one per joint name. The vendor SDK's defaults.
    mit_kp: float | dict[str, float] = 80.0
    mit_kd: float | dict[str, float] = 4.0
    #: Share of the model's gravity torque sent in MIT mode; 0 leaves it all to Kp.
    gravity_feedforward: float = 1.0
    #: What is bolted to the flange, which the arm's model does not contain:
    #: mass (kg) and centre of mass (m) in the flange frame, +z pointing out.
    tool_mass_kg: float = 0.0
    tool_com: list[float] = [0.0, 0.0, 0.0]
    #: Current cap (A) for CSP; ``None`` keeps the motor's own (RS00 15.5, EL05 11).
    #: Lower pushes less hard into an obstacle; too low cannot hold the arm up.
    motor_current_limit: float | None = None
    #: Never unlimited on this arm: the motors' own speed cap is derived from it,
    #: and they allow up to 33 (RS00) and 50 (EL05) rad/s.
    max_joint_speed: float = 0.3


def zero_pose_window(
    lo: float, hi: float, drive_mode: int, ticks_per_rad: float
) -> tuple[int, int]:
    """``(range_min, range_max)`` for a joint whose model zero reads mid-encoder.

    The normalized value maps linearly onto the model limits ``[lo, hi]`` (rad),
    and ``drive_mode`` 1 negates it, so with an asymmetric range the window
    depends on the sign as well as on the zero.
    """
    mid = (PROTOCOL_CPR - 1) // 2
    if drive_mode:
        lo, hi = -hi, -lo
    return int(round(mid + lo * ticks_per_rad)), int(round(mid + hi * ticks_per_rad))


class EduliteA3ArmModule(ArmModule):
    """EL-A3 arm: six Robstride joints plus Cartesian IK via mink / MuJoCo."""

    config_class = EduliteA3ArmConfig

    shoulder_pan = RobstrideServo(model="rs00")
    shoulder_lift = RobstrideServo(model="rs00")
    elbow_flex = RobstrideServo(model="rs00")
    wrist_flex = RobstrideServo(model="el05")
    wrist_yaw = RobstrideServo(model="el05")
    wrist_roll = RobstrideServo(model="el05")

    x = CartesianIKActuator()
    y = CartesianIKActuator()
    z = CartesianIKActuator()
    rx = CartesianIKActuator()
    ry = CartesianIKActuator()
    rz = CartesianIKActuator()

    ik_backend = MinkIK(
        mjcf=ARM_MJCF_PATH,
        joints=dict(zip(IK_JOINTS, IK_JOINTS)),
        frame="flange",
        position_cost=1.0,
        orientation_cost=1.0,
    )

    # A caller's speed or acceleration replaces the config's on ArmModule; on
    # this arm the config values are ceilings a caller can only go under.
    def _pose_speed_kwargs(self, max_pos_speed=None, max_rot_speed=None):
        caps = super()._pose_speed_kwargs()
        asked = super()._pose_speed_kwargs(max_pos_speed, max_rot_speed)
        return {key: min(asked[key], caps[key]) for key in asked}

    def _pose_accel_kwargs(self, max_pos_acc=None, max_rot_acc=None):
        caps = super()._pose_accel_kwargs()
        asked = super()._pose_accel_kwargs(max_pos_acc, max_rot_acc)
        return {
            key: asked[key] if caps[key] is None else min(asked[key], caps[key])
            for key in asked
        }

    def moveJ(self, q, speed=None, acceleration=None, t_now=None) -> None:
        # Core caps the speed at max_joint_speed; the default is that cap too.
        speed = self.cfg.max_joint_speed if speed is None else speed
        cap = self.cfg.max_joint_accel
        if acceleration is not None and cap is not None:
            acceleration = min(acceleration, cap)
        super().moveJ(q, speed=speed, acceleration=acceleration, t_now=t_now)

    def _joint_bus(self):
        """The Robstride bus under the IK wrapper, or a proxy to it.

        In sim it is the world's bus, which has none of the Robstride methods.
        """
        port = next(p for p, names in self.actuators_by_port.items() if IK_JOINTS[0] in names)
        return self.module_bus.component_bus_by_port[port].inner_bus

    def _per_joint(self, value: float | dict[str, float]) -> dict[str, float]:
        if isinstance(value, dict):
            return {name: float(value[name]) for name in IK_JOINTS}
        return {name: float(value) for name in IK_JOINTS}

    def initialize(self):
        super().initialize()
        # Run mode and limits only take while the motor is disabled.
        self.disable_torque(IK_JOINTS)
        bus = self._joint_bus()
        if self.cfg.control_mode == "mit" and hasattr(bus, "use_motion_control"):
            kp, kd = self._per_joint(self.cfg.mit_kp), self._per_joint(self.cfg.mit_kd)
            bus.use_motion_control(
                {name: (kp[name], kd[name]) for name in IK_JOINTS},
                GravityFeedforward(
                    self._joint_limits(),
                    tool_mass_kg=self.cfg.tool_mass_kg,
                    tool_com=self.cfg.tool_com,
                    scale=self.cfg.gravity_feedforward,
                ),
            )
        else:
            limits = {"run_mode": int(RunMode.POSITION_CSP)}
            if self.cfg.motor_current_limit is not None:
                limits["limit_cur"] = self.cfg.motor_current_limit
            self.configure_components(limits, component_names=IK_JOINTS)
        self.enable_torque_holding()
        # After the enable: a disabled motor ignores a limit_spd write.
        self.apply_speed_backstop()

    def apply_speed_backstop(self):
        """Cap every joint at margin x ``max_joint_speed``.

        In CSP the motor enforces it as ``limit_spd``; in MIT the bus bounds how
        far each target may step. The joints are direct drive, so joint and
        motor radians are the same. Cartesian moves are held under
        ``max_joint_speed`` too, so no planned move reaches this cap; a bad
        target or a wrong calibration does.
        """
        limit = SPEED_BACKSTOP_MARGIN * float(self.cfg.max_joint_speed)
        bus = self._joint_bus()
        if hasattr(bus, "set_speed_cap"):
            bus.set_speed_cap(IK_JOINTS, limit)
        else:
            self.configure_components({"limit_spd": limit}, component_names=IK_JOINTS)
        logger.info("EDULITE A3 '%s': motor speed cap %.2f rad/s per joint", self.name, limit)

    def _ticks_per_rad(self, name: str) -> float:
        params = params_for_model(self.actuators[name].cfg.model)
        return (PROTOCOL_CPR - 1) / (params.p_max - params.p_min)

    def _zero_pose_windows(self, calibration: dict[str, MotorCalibration]) -> None:
        limits = self._joint_limits()
        for name in IK_JOINTS:
            entry = calibration.get(name)
            if entry is None:
                continue
            lo, hi = limits[name]
            entry.range_min, entry.range_max = zero_pose_window(
                lo, hi, int(entry.drive_mode), self._ticks_per_rad(name)
            )

    def _save_calibration(self, calibration: dict):
        # The sign check flips drive_mode in place, which moves the window.
        self._zero_pose_windows(calibration)
        super()._save_calibration(calibration)

    def calibrate(self) -> None:
        """Calibrate from one pose: the arm folded at rest, which is the model's zero.

        The joints report multi-turn absolute position, so there is nothing to
        sweep: that pose fixes each joint's offset and the model limits fix the
        window around it. Each joint starts from the ``drive_mode`` in its yaml
        (0 if unset); the sign check that ``solarian calibrate`` runs next
        confirms or flips it against the live sim.
        """
        self.disable_torque(IK_JOINTS)
        print(
            "\n=== Step 1: Zero pose ===\n"
            "Torque is off: support the arm, it is limp.\n"
            "Fold it to its rest pose: upper arm lowered onto its stop, forearm folded\n"
            "back over it onto its stop, wrist straight and in line with the forearm,\n"
            "pan and roll centred. That is every model joint at 0, the pose the next\n"
            "step shows on screen."
        )
        input("Press ENTER when the arm rests folded...")

        offsets = self._joint_bus().set_half_turn_homings(IK_JOINTS)
        calibration = {
            name: MotorCalibration(
                id=int(self.actuators[name].cfg.id),
                drive_mode=int(self.actuators[name].cfg.drive_mode or 0),
                homing_offset=int(offsets[name]),
                range_min=0,
                range_max=PROTOCOL_CPR - 1,
            )
            for name in IK_JOINTS
        }
        self._save_calibration(calibration)
        self.write_calibration()
        for name in IK_JOINTS:
            entry = self.calibration[name]
            print(
                f"  {name}: center_offset={entry.homing_offset}, "
                f"range {entry.range_min} .. {entry.range_max}, drive_mode={entry.drive_mode}"
            )
