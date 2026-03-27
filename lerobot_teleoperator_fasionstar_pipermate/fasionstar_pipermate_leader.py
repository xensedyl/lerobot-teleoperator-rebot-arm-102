import logging
import time
from typing import Any

from fashionstar_uart_sdk.uart_pocket_handler import Monitor_data, PortHandler
from lerobot.motors import MotorCalibration
from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_fasionstar_pipermate_leader import FasionStarPiperMateLeaderConfig

logger = logging.getLogger(__name__)

MEDIUM_TIMEOUT_SEC = 0.01


class FasionStarPiperMateLeader(Teleoperator):
    """
    LeRobot teleoperator integration for the Fashion Star PiperMate leader arm.

    This implementation keeps the lifecycle and calibration flow close to other
    LeRobot leader teleoperators. The servo SDK is only used for device access.
    """

    config_class = FasionStarPiperMateLeaderConfig
    name = "fasionstar_pipermate_leader"

    def __init__(self, config: FasionStarPiperMateLeaderConfig):
        super().__init__(config)
        self.config = config
        self.porthandler = PortHandler(self.config.port, self.config.baudrate)
        self.motor_names = list(self.config.joint_ids.keys())
        self._is_connected = False
        self._validate_config()

    def _validate_config(self) -> None:
        required_keys = set(self.config.joint_ids)
        for field_name in ("joint_directions", "joint_ranges"):
            keys = set(getattr(self.config, field_name))
            if keys != required_keys:
                raise ValueError(
                    f"{field_name} keys must match joint_ids keys. "
                    f"Expected {sorted(required_keys)}, got {sorted(keys)}."
                )
        for motor_name, joint_range in self.config.joint_ranges.items():
            if len(joint_range) != 2:
                raise ValueError(f"joint_ranges[{motor_name!r}] must contain exactly [min, max].")
            if joint_range[0] > joint_range[1]:
                raise ValueError(f"joint_ranges[{motor_name!r}] must satisfy min <= max.")

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motor_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting arm on {self.config.port}...")
        self.porthandler.openPort()

        try:
            for motor_name, motor_id in self.config.joint_ids.items():
                if not self.porthandler.ping(motor_id):
                    raise RuntimeError(f"Servo not found for {motor_name} (id={motor_id}).")

            self._is_connected = True

            # if not self.is_calibrated and calibrate:
            #     logger.info("No calibration file found. Running calibration.")
            #     self.calibrate()
            self.calibrate()

            self.configure()
        except Exception:
            self.porthandler.closePort()
            self._is_connected = False
            raise

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration) and set(self.calibration) == set(self.motor_names)

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration for {self}")
        input(
            "\nCalibration: Set Zero Position\n"
            "Please manually move the PiperMate arm to its zero pose and close the gripper.\n"
            "Press ENTER when ready..."
        )

        self.calibration = {}
        for motor_name, motor_id in self.config.joint_ids.items():
            self.porthandler.set_origin_point(motor_id)
            self.calibration[motor_name] = MotorCalibration(
                id=self.config.joint_ids[motor_name],
                drive_mode=0,
                homing_offset=0,
                range_min=-90,
                range_max=90,
            )

        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        for motor_id in self.config.joint_ids.values():
            self.porthandler.write["Stop_On_Control_Mode"](motor_id, "unlocked", self.config.unlock_timeout_ms)
            time.sleep(MEDIUM_TIMEOUT_SEC)

        self.porthandler.ResetLoop(0xFF)

    def _read_raw_positions(self) -> dict[str, float]:
        monitor_data: dict[str, Monitor_data] = self.porthandler.sync_read["Monitor"](self.config.joint_ids)
        raw_positions: dict[str, float] = {}
        for motor_name in self.motor_names:
            state = monitor_data.get(motor_name)
            if state is None:
                raise RuntimeError(f"Failed to read monitor data for {motor_name}.")
            raw_positions[motor_name] = float(state.current_position)
        return raw_positions

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def get_action(self) -> RobotAction:
        start = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        raw_positions = self._read_raw_positions()
        action_dict: dict[str, Any] = {}
        for motor_name in self.motor_names:
            position = raw_positions[motor_name] * self.config.joint_directions[motor_name]
            range_min, range_max = self.config.joint_ranges[motor_name]
            action_dict[f"{motor_name}.pos"] = self._clamp(
                position,
                float(range_min),
                float(range_max),
            )

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for the PiperMate leader.")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.porthandler.closePort()
        self._is_connected = False
        logger.info(f"{self} disconnected.")
