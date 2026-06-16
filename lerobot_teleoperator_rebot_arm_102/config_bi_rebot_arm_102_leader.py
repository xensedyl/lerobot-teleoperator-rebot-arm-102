from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

from .config_rebot_arm_102_leader import RebotArm102LeaderConfig


def _default_joint_ids() -> dict[str, int]:
    return {
        "shoulder_pan": 0,
        "shoulder_lift": 1,
        "elbow_flex": 2,
        "wrist_flex": 3,
        "wrist_yaw": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }


def _default_joint_ranges() -> dict[str, tuple[float, float]]:
    return {
        "shoulder_pan": (-145.0, 145.0),
        "shoulder_lift": (-170.0, 1.0),
        "elbow_flex": (-200.0, 1.0),
        "wrist_flex": (-80.0, 90.0),
        "wrist_yaw": (-90.0, 90.0),
        "wrist_roll": (-90.0, 90.0),
        "gripper": (-0.0, 270.0),
    }


def _default_joint_directions() -> dict[str, float]:
    return {
        "shoulder_pan": -1.0,
        "shoulder_lift": -1.0,
        "elbow_flex": 1.0,
        "wrist_flex": 1.0,
        "wrist_yaw": 1.0,
        "wrist_roll": -1.0,
        "gripper": -4.0,
    }


@TeleoperatorConfig.register_subclass("bi_rebot_arm_102_leader")
@dataclass
class BiRebotArm102LeaderConfig(TeleoperatorConfig):
    """Configuration for two reBot Arm 102 leader arms."""

    left_port: str
    right_port: str

    left_baudrate: int = 1_000_000
    right_baudrate: int = 1_000_000

    left_joint_ids: dict[str, int] = field(default_factory=_default_joint_ids)
    right_joint_ids: dict[str, int] = field(default_factory=_default_joint_ids)
    left_joint_ranges: dict[str, tuple[float, float]] = field(default_factory=_default_joint_ranges)
    right_joint_ranges: dict[str, tuple[float, float]] = field(default_factory=_default_joint_ranges)
    left_joint_directions: dict[str, float] = field(default_factory=_default_joint_directions)
    right_joint_directions: dict[str, float] = field(default_factory=_default_joint_directions)

    left_id: str = "left_rebot_arm_102_leader"
    right_id: str = "right_rebot_arm_102_leader"

    def make_left_config(self) -> RebotArm102LeaderConfig:
        return RebotArm102LeaderConfig(
            id=f"{self.id}_left" if self.id else self.left_id,
            calibration_dir=self.calibration_dir,
            port=self.left_port,
            baudrate=self.left_baudrate,
            joint_ids=self.left_joint_ids,
            joint_ranges=self.left_joint_ranges,
            joint_directions=self.left_joint_directions,
        )

    def make_right_config(self) -> RebotArm102LeaderConfig:
        return RebotArm102LeaderConfig(
            id=f"{self.id}_right" if self.id else self.right_id,
            calibration_dir=self.calibration_dir,
            port=self.right_port,
            baudrate=self.right_baudrate,
            joint_ids=self.right_joint_ids,
            joint_ranges=self.right_joint_ranges,
            joint_directions=self.right_joint_directions,
        )
