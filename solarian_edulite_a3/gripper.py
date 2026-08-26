"""EDULITE A3 gripper module (Robstride L7, aperture in centimetres)."""

from logging import getLogger
import time

from solarian.core.configs import ModuleConfig
from solarian.core.data import PosData
from solarian.core.gripper_module import GripperModule
from solarian_edulite_a3.aperture import (
    APERTURE_CLOSED_CM,
    APERTURE_OPEN_CM,
    aperture_to_norm as cm_to_norm,
    norm_to_aperture as norm_to_cm,
    resolve_aperture_limits,
)
from solarian_robstride.actuator import RobstrideServo

logger = getLogger("module")


class EduliteA3GripperConfig(ModuleConfig):
    """Config for the EL-A3 parallel-jaw gripper (L7)."""

    calibration_file: str | None = None
    aperture_closed_cm: float = APERTURE_CLOSED_CM
    aperture_open_cm: float = APERTURE_OPEN_CM


class EduliteA3GripperModule(GripperModule):
    config_class = EduliteA3GripperConfig

    scalar_actuator = "gripper"
    gripper = RobstrideServo()

    def _aperture_limits(self) -> tuple[float, float]:
        return resolve_aperture_limits(
            getattr(self.cfg, "aperture_closed_cm", None),
            getattr(self.cfg, "aperture_open_cm", None),
        )

    @property
    def home_value(self) -> float:
        """SET_HOME target: fully open finger aperture in cm."""
        return self._aperture_limits()[1]

    def initialize(self):
        super().initialize()
        lo, hi = self._aperture_limits()
        logger.info(
            "EduliteA3 gripper '%s': aperture %.2f–%.2f cm ↔ servo -100–100",
            self.name,
            lo,
            hi,
        )
        self.disable_torque()
        self.configure_components(
            {
                "run_mode": 5,  # CSP
                "limit_spd": 4.0,
            },
            component_names=["gripper"],
        )
        self.enable_torque()
        time.sleep(0.05)
        self._hold_present_pose()

    def _hold_present_pose(self) -> None:
        present = self.read_sensors(PosData, ["gripper"])
        self.module_bus.write(
            {"gripper": PosData(value=present["gripper"]["value"])},
            normalize=True,
        )

    def enter_safe_hold(self) -> None:
        self._hold_present_pose()
        self.enable_torque()

    def on_arm(self) -> None:
        self._hold_present_pose()
        self.enable_torque()

    def on_disarm(self) -> None:
        pass

    def aperture_to_norm(self, aperture_cm: float) -> float:
        lo, hi = self._aperture_limits()
        return cm_to_norm(aperture_cm, lo, hi)

    def norm_to_aperture(self, norm: float) -> float:
        lo, hi = self._aperture_limits()
        return norm_to_cm(norm, lo, hi)

    def read_aperture_cm(self) -> float:
        pos = self.read_sensors(PosData, ["gripper"])
        return self.norm_to_aperture(pos["gripper"]["value"])

    def _scalar_to_actuator(self, value: float) -> float:
        return self.aperture_to_norm(value)

    def _actuator_to_scalar(self, value: float) -> float:
        return self.norm_to_aperture(value)
