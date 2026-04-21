from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("rebot_arm_102_leader")
@dataclass
class RebotArm102LeaderConfig(TeleoperatorConfig):
    """Configuration for the reBot Arm 102 leader arm."""

    port: str
    baudrate: int = 1_000_000
    joint_ids: dict[str, int] = field(
        default_factory=lambda: {
            "shoulder_pan": 0,
            "shoulder_lift": 1,
            "elbow_flex": 2,
            "wrist_flex": 3,
            "wrist_yaw": 4,
            "wrist_roll": 5,
            "gripper": 6,
        }
    )

    # Optional LeRobot calibration directory override for the reBot Arm 102 leader.
    # Note that the factor for gripper is also adding a scale of 6 to better match reBot B601's gripper range.
    joint_directions: dict[str, int] = field(
        default_factory=lambda: {
            "shoulder_pan": -1,
            "shoulder_lift": -1,
            "elbow_flex": 1,
            "wrist_flex": 1,
            "wrist_yaw": 1,
            "wrist_roll": -1,
            "gripper": -6,
        }
    )
    joint_ranges: dict[str, list[int]] = field(
        default_factory=lambda: {
            "shoulder_pan": [-150, 150],
            "shoulder_lift": [-170, 1],
            "elbow_flex": [-200, 1],
            "wrist_flex": [-80, 90],
            "wrist_yaw": [-90, 90],
            "wrist_roll": [-90, 90],
            "gripper": [-270, 0],
        }
    )
    unlock_timeout_ms: int = 900
