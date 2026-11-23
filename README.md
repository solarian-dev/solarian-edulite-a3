# solarian-edulite-a3

EDULITE A3 arm module for Solarian. Joints use the Solarian Arm 1 names
(`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_yaw`,
`wrist_roll`). Motors are Robstride RS00 (joints 1-3) and EL05 (joints 4-6) on
one CAN bus; this plugin imports `RobstrideServo` from `solarian-robstride`.
For a gripper, use `Gripper1RobstrideModule` from `solarian-gripper1[robstride]`
on the same adapter.

## Installation

```bash
pip install -e "plugins/solarian-robstride[hardware]"
pip install -e "plugins/solarian-edulite-a3[hardware]"
```

## Modules

| Entry point | Class | Role |
|---|---|---|
| `edulite-a3-arm` | `EduliteA3ArmModule` | 6 joints + Mink IK (`x..rz`) |

`solarian install edulite-a3` copies the arm's default config,
`templates/modules/right_arm.yaml`, to `conf/modules/right_arm.yaml`. Rename
it to match the attribute on your `Robot`, and give the gripper's
`conf/modules/<attribute>.yaml` the same `port` so the USB-CAN adapter is
shared.

Attach on a `Robot` the same way Solarian Arm 1 does:

```python
from solarian.core.robot import Robot
from solarian.sim import SceneConfig, ModuleAttachment
from solarian_edulite_a3 import ARM_MJCF_PATH
from solarian_edulite_a3.arm import EduliteA3ArmModule
from solarian_gripper1 import MJCF_ROBSTRIDE_PATH
from solarian_gripper1.gripper_robstride import Gripper1RobstrideModule

class EduliteRobot(Robot):
    right_arm = EduliteA3ArmModule(ik_tcp=Gripper1RobstrideModule.tcp_from_flange)
    right_gripper = Gripper1RobstrideModule()
    scene = SceneConfig(
        attachments=[
            ModuleAttachment(module="right_arm", mjcf=ARM_MJCF_PATH, prefix="arm/"),
            ModuleAttachment(
                module="right_gripper",
                mjcf=MJCF_ROBSTRIDE_PATH,
                parent_site="arm/flange",
                child_site="flange",
                prefix="gripper/",
            ),
        ],
    )
```

## Safety limits

`max_speed`, `max_accel`, `max_joint_speed` and `max_joint_accel` are ceilings
on this arm: a skill that asks for more (teleop, a policy) gets the configured
value. `max_joint_speed` (rad/s, default 0.3) also holds Cartesian moves, which
slow down wherever they would need a faster joint, and it is the `moveJ` speed
when the caller names none. `solarian calibrate-arm` measures all three speeds.

Every joint also has a speed cap below the planner, whatever it is commanded:
1.5 x `max_joint_speed`, set on start and again whenever calibration changes
the speed. In CSP the motor enforces it; in MIT mode the bus does, by bounding
how far each tick's target may move. No planned move reaches it; a bad target
or a wrong calibration does.

Enabling torque holds each joint where it is: in CSP the present position is
latched as the motor's target first, and in MIT mode the first command after
the enable is a hold at the present position.

## Control mode

`control_mode: mit` (the default) drives the joints the way RobStride's own
EL-A3 SDK does. Every tick sends each motor a type 1 (MIT) frame: its target,
a velocity, a stiffness Kp, a damping Kd and a torque, and the motor applies
`Kp (target - position) + Kd (velocity - speed) + torque`. The torque is the
model's gravity load at the target pose, so the stiffness only has to correct
small errors, and the velocity is the target's own rate, so the damping does
not brake a planned move.

`control_mode: csp` uses the motors' own position loop instead. Its factory
gains oscillate inside the gear backlash under load, which is the buzz an
EduLite makes holding a pose in CSP. Keep it as a fallback.

The gripper stays in CSP either way: its motor carries no gravity load, and
the bus only sends MIT frames to the arm joints.

MIT fields (ignored in CSP):

- `mit_kp` (Nm/rad, default 80) and `mit_kd` (Nm·s/rad, default 4): one value
  for every joint, or a map by joint name. These are the vendor's defaults.
  Raise Kp for a stiffer hold; raise Kd if it overshoots.
- `gravity_feedforward` (default 1.0): the share of the model's gravity torque
  sent. 0 leaves gravity to Kp, and the arm sags until the error pays for it.
- `tool_mass_kg` (default 0) and `tool_com` (m, flange frame, +z out of the
  flange): whatever is bolted to the flange, which the arm's model does not
  contain. Without it the arm droops by the tool's weight over Kp.

The link masses in the MJCF are the vendor's (`el_a3.urdf` in the EL-A3 SDK),
moved into this model's frames, about 1.8 kg for the moving links. The last
link keeps its own small mass: the vendor's includes their gripper, which is
what `tool_mass_kg` describes here.

CSP field (ignored in MIT):

- `motor_current_limit` (A, default unset): the CSP current limit. Lower pushes
  less hard into an obstacle; too low cannot hold the arm up.

## Calibration

The joints report multi-turn absolute position, so there is no range sweep.
`solarian calibrate` asks for the arm folded at rest (upper arm lowered onto
its stop, forearm folded back over it, wrist straight, pan and roll centred),
which is every model joint at 0. That pose sets each joint's offset, and the MJCF limits set the window
around it. The sign check that follows confirms each joint's direction against
the live sim; the template's `drive_mode` values are first guesses from the
vendor SDK.

Torque is off while calibrating and the arm is limp: support it.

## Hardware

- Robstride USB-CAN adapter (CH340): see `solarian-robstride` README.
- CAN IDs 1-6 on the arm, ID 7 on the gripper.
- Joint models are declared in `arm.py` (`rs00` for 1-3, `el05` for 4-6);
  change them there if your arm has different motors.
- The MJCF's `wrist_flex` range (-0.67 to 2.35 rad) is wider than the vendor
  SDK's ±1.57 rad. Check the real wrist's stops before trusting the upper end.
