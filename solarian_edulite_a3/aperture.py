"""Physical fingertip aperture at the EL-A3 gripper (L7) ends of travel.

Vendor ros2_control uses roughly 2-10 cm of jaw opening. Confirm with a ruler
after OnShape export / hardware bring-up and override in YAML.
"""

from __future__ import annotations

APERTURE_CLOSED_CM = 2.0
APERTURE_OPEN_CM = 10.0


def resolve_aperture_limits(
    closed_cm: float | None = None,
    open_cm: float | None = None,
) -> tuple[float, float]:
    lo = APERTURE_CLOSED_CM if closed_cm is None else float(closed_cm)
    hi = APERTURE_OPEN_CM if open_cm is None else float(open_cm)
    if hi <= lo:
        return APERTURE_CLOSED_CM, APERTURE_OPEN_CM
    return lo, hi


def aperture_to_norm(
    aperture_cm: float,
    closed_cm: float | None = None,
    open_cm: float | None = None,
) -> float:
    """Map finger aperture (cm) to normalized servo command [-100, 100]."""
    lo, hi = resolve_aperture_limits(closed_cm, open_cm)
    norm = (float(aperture_cm) - lo) / (hi - lo) * 200.0 - 100.0
    return max(-100.0, min(100.0, norm))


def norm_to_aperture(
    norm: float,
    closed_cm: float | None = None,
    open_cm: float | None = None,
) -> float:
    """Inverse of :func:`aperture_to_norm` (normalized [-100, 100] → cm)."""
    lo, hi = resolve_aperture_limits(closed_cm, open_cm)
    return lo + ((float(norm) + 100.0) / 200.0) * (hi - lo)
