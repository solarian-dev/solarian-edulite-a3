# solarian-edulite-a3

EDULITE A3 arm and gripper modules for Solarian. Joints use the Solarian Arm 1 names
(`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_yaw`,
`wrist_roll`, `gripper`). Motors are Robstride RS00 / EL05 on one CAN bus;
this plugin imports `RobstrideServo` from `solarian-robstride`.

## Installation

```bash
pip install -e plugins/solarian-robstride
pip install -e plugins/solarian-edulite-a3
```

## Modules

| Entry point | Class | Role |
|---|---|---|
| `edulite-a3-arm` | `EduliteA3ArmModule` | 6 joints + Mink IK (`x..rz`) |
| `edulite-a3-gripper` | `EduliteA3GripperModule` | L7 jaw, aperture in cm |

Default YAML lives at `solarian_edulite_a3/conf_arm.yaml` and
`conf_gripper.yaml`. Copy them into a project's `conf/` (for example
`right_arm.yaml` / `right_gripper.yaml`) and set the same `port` so the
USB-CAN adapter is shared.

Attach on a `Robot` the same way Solarian Arm 1 does:

```python
from solarian.core.robot import Robot
from solarian.sim import SceneConfig, ModuleAttachment
from solarian_edulite_a3 import ARM_MJCF_PATH
from solarian_edulite_a3.arm import EduliteA3ArmModule
from solarian_edulite_a3.gripper import EduliteA3GripperModule

class EduliteRobot(Robot):
    right_arm = EduliteA3ArmModule()
    right_gripper = EduliteA3GripperModule()
    scene = SceneConfig(
        attachments=[
            ModuleAttachment(
                module="right_arm",
                mjcf=ARM_MJCF_PATH,
                prefix="arm/",
            ),
        ],
    )
```

The placeholder MJCF includes the `gripper` joint, so the gripper module's
`sim:` block points at `arm/gripper` and does not attach a second model.

## OnShape export

`assets/edulite_a3.xml` is kinematics-only (capsules, vendor joint limits).
When the assembly is ready:

1. Put the OnShape document URL in `assets/edulite_a3_description/config.json`.
2. Name CAD mates `shoulder_pan` … `wrist_roll` (and `gripper` if the jaw is
   in the same assembly).
3. Keep `solarian.ee_site.name` as `gripper`.
4. Run:

```bash
solarian onshape-export plugins/solarian-edulite-a3/solarian_edulite_a3/assets/edulite_a3_description
```

Then `solarian calibrate`.

## Hardware

- Robstride USB-CAN adapter (CH340): see `solarian-robstride` README.
- CAN IDs 1–6 on the arm, ID 7 on the gripper.
- Set mechanical zero (adapter GUI or bus set-zero) before the first
  `solarian calibrate` sweep.
- Wrist motors: YAML `model: el05` by default; switch to `rs05` if that is
  what is installed.
