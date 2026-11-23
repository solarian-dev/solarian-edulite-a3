"""Gravity feed-forward for the EDULITE A3 joints."""

from __future__ import annotations

from typing import Sequence

import numpy as np

from solarian_edulite_a3 import ARM_MJCF_PATH


class GravityFeedforward:
    """Joint torques (Nm) that hold the arm, and a tool on its flange, against gravity.

    The Robstride bus calls it on every MIT write. When the arm shares its CAN
    port with another module that bus lives in a bus server process, so this
    object is pickled across, and it loads its own MuJoCo model where it runs.

    Positions arrive normalized -100..100, mapped linearly onto each joint's
    model limits (rad) as the IK maps them. Torques are in the model's joint
    sign; the bus applies each motor's ``drive_mode``.
    """

    def __init__(
        self,
        limits: dict[str, tuple[float, float]],
        tool_mass_kg: float = 0.0,
        tool_com: Sequence[float] = (0.0, 0.0, 0.0),
        scale: float = 1.0,
        mjcf: str = ARM_MJCF_PATH,
        flange: str = "flange",
    ):
        self.limits = dict(limits)
        self.tool_mass_kg = float(tool_mass_kg)
        #: metres, in the flange site's frame (+z out of the flange)
        self.tool_com = np.asarray(tool_com, dtype=float)
        self.scale = float(scale)
        self.mjcf = mjcf
        self.flange = flange
        self._model = None
        self._data = None

    def __getstate__(self):
        state = dict(self.__dict__)
        state["_model"] = state["_data"] = None
        return state

    def __call__(self, normalized: dict[str, float]) -> dict[str, float]:
        # Gravity on one joint depends on every joint distal to it, so a
        # partial pose gives no torque rather than a wrong one.
        if self.scale == 0.0 or any(name not in normalized for name in self.limits):
            return {}
        q = {}
        for name, (lo, hi) in self.limits.items():
            v = min(100.0, max(-100.0, float(normalized[name])))
            q[name] = (v + 100.0) / 200.0 * (hi - lo) + lo
        return self.torques(q)

    def torques(self, q: dict[str, float]) -> dict[str, float]:
        """Holding torque per joint at joint angles ``q`` (model rad)."""
        import mujoco

        if self._model is None:
            self._model = mujoco.MjModel.from_xml_path(self.mjcf)
            self._data = mujoco.MjData(self._model)
        m, d = self._model, self._data
        d.qvel[:] = 0.0
        for name, value in q.items():
            d.qpos[m.jnt_qposadr[m.joint(name).id]] = value
        mujoco.mj_forward(m, d)
        tau = d.qfrc_bias.copy()
        if self.tool_mass_kg:
            site = m.site(self.flange).id
            com = d.site_xpos[site] + d.site_xmat[site].reshape(3, 3) @ self.tool_com
            jacp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jacp, None, com, m.site_bodyid[site])
            tau -= jacp.T @ (self.tool_mass_kg * m.opt.gravity)
        return {name: self.scale * float(tau[m.jnt_dofadr[m.joint(name).id]]) for name in q}
