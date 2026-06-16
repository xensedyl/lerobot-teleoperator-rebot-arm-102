import logging
from concurrent.futures import ThreadPoolExecutor
from functools import cached_property

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_bi_rebot_arm_102_leader import BiRebotArm102LeaderConfig
from .rebot_arm_102_leader import RebotArm102Leader


logger = logging.getLogger(__name__)


class BiRebotArm102Leader(Teleoperator):
    """Bimanual reBot Arm 102 leader teleoperator."""

    config_class = BiRebotArm102LeaderConfig
    name = "bi_rebot_arm_102_leader"

    def __init__(self, config: BiRebotArm102LeaderConfig):
        super().__init__(config)
        self.config = config
        self.left_arm = RebotArm102Leader(config.make_left_config())
        self.right_arm = RebotArm102Leader(config.make_right_config())

    @staticmethod
    def _prefix_action(side: str, action: RobotAction) -> RobotAction:
        return {f"{side}_{key}": value for key, value in action.items()}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **self._prefix_action("left", self.left_arm.action_features),
            **self._prefix_action("right", self.right_arm.action_features),
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(
            "Connecting dual reBot Arm 102 leaders: left=%s right=%s...",
            self.config.left_port,
            self.config.right_port,
        )
        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = [
                    executor.submit(self.left_arm.connect, calibrate),
                    executor.submit(self.right_arm.connect, calibrate),
                ]
                for future in futures:
                    future.result()
        except Exception:
            self._disconnect_connected_children()
            raise

        logger.info("%s connected.", self)

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def get_action(self) -> RobotAction:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left_arm.get_action)
            right_future = executor.submit(self.right_arm.get_action)
            left_action = left_future.result()
            right_action = right_future.result()

        return {
            **self._prefix_action("left", left_action),
            **self._prefix_action("right", right_action),
        }

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for bi reBot Arm 102 leaders.")

    def _disconnect_connected_children(self) -> None:
        for arm in (self.left_arm, self.right_arm):
            try:
                if arm.is_connected:
                    arm.disconnect()
            except Exception:
                logger.debug("Failed to disconnect %s during cleanup.", arm, exc_info=True)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(self.left_arm.disconnect),
                executor.submit(self.right_arm.disconnect),
            ]
            for future in futures:
                future.result()

        logger.info("%s disconnected.", self)
