# reBot Arm 102 Teleoperator for reBot B601

[中文版说明](./README.zh-CN.md)

This repository provides a LeRobot teleoperator integration for the reBot Arm 102 leader arm, intended to be paired with the Seeed reBot B601 follower arm.

The implementation is intentionally opinionated:

- joint names are aligned to reBot B601
- leader-side joint limits and direction mapping are taken directly from config
- each startup calibration sets the current servo origin to zero

## Supported Setup

- Leader: reBot Arm 102
- Follower: Seeed reBot B601
- Communication: UART for reBot Arm 102, CAN or Damiao serial bridge for B601

## Installation

Install LeRobot first, then install this package in editable mode:

```bash
cd lerobot-teleoperator-rebot-arm-102
pip install -e .
```

This package registers two teleoperator types:

- `rebot_arm_102_leader`
- `bi_rebot_arm_102_leader`

## Default Mapping

- `shoulder_pan` -> servo ID `0`
- `shoulder_lift` -> servo ID `1`
- `elbow_flex` -> servo ID `2`
- `wrist_flex` -> servo ID `3`
- `wrist_yaw` -> servo ID `4`
- `wrist_roll` -> servo ID `5`
- `gripper` -> servo ID `6`

Leader-side joint limits and direction mapping are defined in
`lerobot_teleoperator_rebot_arm_102/config_rebot_arm_102_leader.py`. The default
mapping is aligned to B601 joint-space actions: `shoulder_pan=-1`,
`shoulder_lift=-1`, `elbow_flex=1`, `wrist_flex=1`, `wrist_yaw=1`,
`wrist_roll=-1`, `gripper=-4`.
For the RT serial gripper, `gripper.pos` is emitted as a normalized value:
`0=open`, `1=closed`.

## Usage

### Single-Arm Teleoperation

```bash
lerobot-teleoperate \
  --robot.type=seeed_b601_dm_follower \
  --robot.id=follower1 \
  --robot.port=/dev/ttyACM4 \
  --robot.can_adapter=damiao \
  --teleop.type=rebot_arm_102_leader \
  --teleop.id=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --fps=100 \
  --display_data=true
```

### Dual-Arm Teleoperation

Use `bi_rebot_arm_102_leader` with a bimanual follower such as `bi_seeed_b601_rt_follower`.
The dual leader outputs prefixed joint actions:

- `left_shoulder_pan.pos` ... `left_gripper.pos`
- `right_shoulder_pan.pos` ... `right_gripper.pos`

These keys are consumed directly by `bi_seeed_b601_rt_follower`.
The left and right leader directions can be overridden separately with
`--teleop.left_joint_directions=...` and `--teleop.right_joint_directions=...`.

```bash
lerobot-teleoperate \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=joint \
  --teleop.type=bi_rebot_arm_102_leader \
  --teleop.id=bi_rebot_arm_102_leader \
  --teleop.left_port=/dev/ttyUSB0 \
  --teleop.right_port=/dev/ttyUSB1 \
  --fps=100 \
  --display_data=true
```

### Dual-Arm Data Recording

There are two recording entrypoints with different implementations:

- `lerobot-record`: uses the official LeRobot recorder from the currently installed
  `/home/xense/rebot_lerobot/lerobot` checkout. Streaming encoding support depends on that LeRobot version.
- `lerobot-record-rebot-arm-102`: uses this package's recorder and package-local streaming encoder, independent
  of whether the main `lerobot` recorder supports `--dataset.streaming_encoding`.

`bi_seeed_b601_rt_follower` publishes arm joint position observations by default, so no extra robot
option is needed. The package-local `lerobot-record-rebot-arm-102` entrypoint writes both `action`
and `observation.state` joint fields as
`left_joint_1.pos ... left_joint_6.pos, left_gripper.pos, right_joint_1.pos ...
right_joint_6.pos, right_gripper.pos`. The B601 RT follower internally controls joints in degrees.
This package recorder defaults to `--dataset.joint_unit=rad`, which converts non-gripper joint `.pos`
action and observation values to radians right before writing dataset frames. Gripper values stay
normalized as `0=open, 1=closed`.

#### Option 1: Official LeRobot Recorder, Default Encoding

This command uses the official `lerobot-record` path. It does not pass
`--dataset.streaming_encoding`, so video encoding follows the default behavior of the installed LeRobot recorder.

```bash
lerobot-record \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=joint \
  --teleop.type=bi_rebot_arm_102_leader \
  --teleop.id=bi_rebot_arm_102_leader \
  --teleop.left_port=/dev/ttyUSB0 \
  --teleop.right_port=/dev/ttyUSB1 \
  --dataset.repo_id=xensedyl/b601-bi-arm102-demo \
  --dataset.single_task="Teleoperate dual B601 with dual Arm102 leaders" \
  --dataset.num_episodes=3 \
  --dataset.fps=30 \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

#### Option 2: Official LeRobot Recorder, Official Streaming Encoding

If your installed LeRobot recorder supports `--dataset.streaming_encoding`, this command keeps using the official
`lerobot-record` implementation and lets LeRobot handle streaming encoding. If it reports an unknown argument,
use option 3 instead.

```bash
lerobot-record \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=joint \
  --teleop.type=bi_rebot_arm_102_leader \
  --teleop.id=bi_rebot_arm_102_leader \
  --teleop.left_port=/dev/ttyUSB0 \
  --teleop.right_port=/dev/ttyUSB1 \
  --dataset.repo_id=xensedyl/b601-bi-arm102-demo \
  --dataset.single_task="Teleoperate dual B601 with dual Arm102 leaders" \
  --dataset.num_episodes=3 \
  --dataset.fps=30 \
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

#### Option 3: Package Recorder, Package-Local Streaming Encoding

This command uses `lerobot-teleoperator-rebot-arm-102`'s own recorder. Streaming encoding is implemented by this
package's `StreamingLeRobotDataset`, not by the official LeRobot recorder.

```bash
lerobot-record-rebot-arm-102 \
  --robot.type=bi_seeed_b601_rt_follower \
  --robot.left_port=/dev/ttyACM0 \
  --robot.right_port=/dev/ttyACM1 \
  --robot.id=bi_follower \
  --robot.can_adapter=damiao \
  --robot.action_mode=joint \
  --teleop.type=bi_rebot_arm_102_leader \
  --teleop.id=bi_rebot_arm_102_leader \
  --teleop.left_port=/dev/ttyUSB0 \
  --teleop.right_port=/dev/ttyUSB1 \
  --dataset.repo_id=xensedyl/b601-bi-arm102-demo \
  --dataset.single_task="Teleoperate dual B601 with dual Arm102 leaders" \
  --dataset.num_episodes=3 \
  --dataset.fps=30 \
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --dataset.joint_unit=rad \
  --resume=false \
  --dataset.push_to_hub=true \
  --display_data=false
```

To use this package recorder without streaming encoding, replace the streaming options above with:

```bash
  --dataset.streaming_encoding=false \
  --dataset.vcodec=libsvtav1 \
```

## Example Scripts

### `read_raw_angles.py`

Purpose:

- read raw reBot Arm 102 servo angles directly from the Fashion Star SDK
- verify servo ID to joint mapping
- check whether a given joint is actually changing at the hardware level

Usage:

```bash
cd lerobot-teleoperator-rebot-arm-102
python examples/read_raw_angles.py --port /dev/ttyUSB0
```

What to look for:

- move only one joint at a time
- confirm the expected joint column changes
- if raw values change but teleop behavior is wrong, the problem is usually the leader range config or the follower direction config, not the SDK readout

### `read_leader_follower_compare.py`

Purpose:

- read reBot Arm 102 leader output and B601 follower observation side by side
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
cd lerobot-teleoperator-rebot-arm-102
python examples/read_leader_follower_compare.py --leader-port /dev/ttyUSB0 --follower-port can0 --follower-type dm
```

Usage with a Damiao follower through a Damiao serial bridge:

```bash
cd lerobot-teleoperator-rebot-arm-102
python examples/read_leader_follower_compare.py --leader-port /dev/ttyUSB0 --follower-port /dev/ttyACM0 --follower-type dm --follower-can-adapter damiao
```

What to look for:

- move one joint on the leader
- move the same joint by hand on the follower if needed
- compare whether `mapped` and `follower` change in the same sign direction
- if one side increases while the other decreases, update that joint in the leader `joint_directions` config

## Notes

- Under the current implementation, startup calibration resets each reBot Arm 102 servo origin to the current pose.
- `joint_ranges` are taken from config, not from calibration data.
- If a joint appears stuck near one limit, check `joint_ranges` first.
