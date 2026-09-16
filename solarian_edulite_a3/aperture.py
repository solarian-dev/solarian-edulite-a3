"""Physical fingertip aperture at the EL-A3 gripper (L7) ends of travel.

Vendor ros2_control uses roughly 20-100 mm of jaw opening. Confirm with a ruler
after OnShape export / hardware bring-up and override in YAML.

These are only the straight-line endpoints. The jaw linkage is not linear over
that travel, so run ``solarian calibrate-gripper-ranges`` to replace them with a
measured ``aperture_table``.
"""

from __future__ import annotations

APERTURE_CLOSED_MM = 20.0
APERTURE_OPEN_MM = 100.0
