# Fashion Star PiperMate Teleoperator for reBot B601

This repository provides a LeRobot teleoperator integration for the Fashion Star PiperMate leader arm, intended to be paired with the Seeed reBot B601 follower arm.

The implementation is intentionally opinionated:

- joint names are aligned to reBot B601
- joint directions are configured in code
- joint limits are taken directly from config
- each startup calibration sets the current servo origin to zero

## Supported Setup

- Leader: Fashion Star PiperMate
- Follower: Seeed reBot B601
- Communication: UART for PiperMate, CAN or Damiao serial bridge for B601

## Installation

Install LeRobot first, then install this package in editable mode:

```bash
cd lerobot-teleoperator-fasionstar-pipermate
pip install -e .
```

This package registers one teleoperator type:

- `fasionstar_pipermate_leader`

## Default Mapping

- `shoulder_pan` -> servo ID `0`
- `shoulder_lift` -> servo ID `1`
- `elbow_flex` -> servo ID `2`
- `wrist_flex` -> servo ID `3`
- `wrist_yaw` -> servo ID `4`
- `wrist_roll` -> servo ID `5`
- `gripper` -> servo ID `6`

Joint directions and joint limits are defined in `lerobot_teleoperator_fasionstar_pipermate/config_fasionstar_pipermate_leader.py`.

## Usage

Standard teleoperation:

```bash
lerobot-teleoperate \
  --teleop.type=fasionstar_pipermate_leader \
  --teleop.id=pipermate_leader \
  --teleop.port=/dev/ttyUSB0
```

## Example Scripts

### `read_raw_angles.py`

Purpose:

- read raw PiperMate servo angles directly from the Fashion Star SDK
- verify servo ID to joint mapping
- check whether a given joint is actually changing at the hardware level

Usage:

```bash
cd lerobot-teleoperator-fasionstar-pipermate
python examples/read_raw_angles.py --port /dev/ttyUSB0
```

What to look for:

- move only one joint at a time
- confirm the expected joint column changes
- if raw values change but teleop behavior is wrong, the problem is usually direction or range config, not the SDK readout

### `read_leader_follower_compare.py`

Purpose:

- read PiperMate leader output and B601 follower observation side by side
- compare `leader`, `follower`, and `delta` per joint
- debug direction mismatches safely without sending follower commands

Prerequisite:

- need a reBot B601 arm setup
- install the python integration for reBot B601, https://github.com/Seeed-Projects/lerobot-robot-seeed-b601

Behavior:

- after connecting the B601 follower, it disables torque so the arm can be moved by hand
- it does not send action commands to the follower

Usage with a Damiao follower on `can0`:

```bash
cd lerobot-teleoperator-fasionstar-pipermate
python examples/read_leader_follower_compare.py --leader-port /dev/ttyUSB0 --follower-port can0 --follower-type dm
```

Usage with a Damiao follower through a Damiao serial bridge:

```bash
cd lerobot-teleoperator-fasionstar-pipermate
python examples/read_leader_follower_compare.py --leader-port /dev/ttyUSB0 --follower-port /dev/ttyACM0 --follower-type dm --follower-can-adapter damiao
```

What to look for:

- move one joint on the leader
- move the same joint by hand on the follower if needed
- compare whether `leader` and `follower` change in the same sign direction
- if one side increases while the other decreases, flip that joint in `joint_directions`

## Notes

- Under the current implementation, startup calibration resets each PiperMate servo origin to the current pose.
- `joint_ranges` are taken from config, not from calibration data.
- If a joint appears stuck near one limit, check `joint_ranges` first.
