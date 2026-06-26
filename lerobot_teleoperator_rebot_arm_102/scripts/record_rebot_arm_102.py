import logging
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

import rerun as rr

from lerobot.cameras import CameraConfig  # noqa: F401
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.processor import (
    PolicyProcessorPipeline,
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

import lerobot_robot_seeed_b601_rt  # noqa: F401
import lerobot_teleoperator_rebot_arm_102  # noqa: F401
from lerobot_teleoperator_rebot_arm_102.streaming_dataset import (
    StreamingLeRobotDataset,
    StreamingVideoEncodingManager,
)

KINEMATIC_JOINT_NAME_MAP = {
    "shoulder_pan": "joint_1",
    "shoulder_lift": "joint_2",
    "elbow_flex": "joint_3",
    "wrist_flex": "joint_4",
    "wrist_yaw": "joint_5",
    "wrist_roll": "joint_6",
}

CANONICAL_JOINT_STATE_NAMES = [
    "left_joint_1.pos",
    "left_joint_2.pos",
    "left_joint_3.pos",
    "left_joint_4.pos",
    "left_joint_5.pos",
    "left_joint_6.pos",
    "left_gripper.pos",
    "right_joint_1.pos",
    "right_joint_2.pos",
    "right_joint_3.pos",
    "right_joint_4.pos",
    "right_joint_5.pos",
    "right_joint_6.pos",
    "right_gripper.pos",
]


@dataclass
class RebotArm102DatasetRecordConfig:
    repo_id: str
    single_task: str
    root: str | Path | None = None
    fps: int = 30
    episode_time_s: int | float = 60
    reset_time_s: int | float = 60
    num_episodes: int = 50
    video: bool = True
    push_to_hub: bool = True
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1
    vcodec: str = "auto"
    streaming_encoding: bool = True
    encoder_queue_maxsize: int = 30
    encoder_threads: int | None = None
    joint_unit: str = "rad"
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.single_task is None:
            raise ValueError("You need to provide --dataset.single_task.")
        if self.joint_unit not in {"rad", "deg"}:
            raise ValueError("--dataset.joint_unit must be either 'rad' or 'deg'.")


@dataclass
class RebotArm102RecordConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    dataset: RebotArm102DatasetRecordConfig
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    resume: bool = False


def _disconnect_devices(robot: Robot, teleop: Teleoperator | None) -> None:
    try:
        if robot.is_connected:
            robot.disconnect()
    except Exception as exc:
        logging.warning("Robot disconnect failed: %s", exc)

    try:
        if teleop is not None and teleop.is_connected:
            teleop.disconnect()
    except Exception as exc:
        logging.warning("Teleoperator disconnect failed: %s", exc)


def _safe_log_say(text: str, play_sounds: bool = True, blocking: bool = False) -> None:
    logging.info(text)
    if not play_sounds:
        return

    try:
        say(text, blocking)
    except Exception as exc:
        logging.warning("Text-to-speech failed: %s", exc)


def _build_dataset_features(
    robot: Robot,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    use_videos: bool,
) -> dict[str, dict]:
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=use_videos,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=use_videos,
        ),
    )
    return _normalize_dataset_feature_names(features)


def _normalize_joint_key(key: str) -> str:
    for source_name, target_name in KINEMATIC_JOINT_NAME_MAP.items():
        for prefix in ("left_", "right_", ""):
            source_prefix = f"{prefix}{source_name}."
            if key.startswith(source_prefix):
                return f"{prefix}{target_name}.{key[len(source_prefix):]}"
    return key


def _normalize_joint_value_keys(values: dict[str, Any]) -> dict[str, Any]:
    return {_normalize_joint_key(key): value for key, value in values.items()}


def _canonicalize_feature_names(names: list[str] | tuple[str, ...] | None) -> list[str] | None:
    if names is None:
        return None

    normalized_names = [_normalize_joint_key(name) for name in names]
    if len(set(normalized_names)) != len(normalized_names):
        raise ValueError(f"Duplicate dataset feature names after normalization: {normalized_names}")

    ordered_names = [name for name in CANONICAL_JOINT_STATE_NAMES if name in normalized_names]
    ordered_names.extend(name for name in normalized_names if name not in ordered_names)

    return ordered_names


def _normalize_dataset_feature_names(features: dict[str, dict]) -> dict[str, dict]:
    normalized_features: dict[str, dict] = {}
    for key, ft in features.items():
        normalized_ft = dict(ft)
        if normalized_ft.get("dtype") == "float32" and len(normalized_ft.get("shape", ())) == 1:
            normalized_ft["names"] = _canonicalize_feature_names(normalized_ft.get("names"))
            if normalized_ft["names"] is not None:
                normalized_ft["shape"] = (len(normalized_ft["names"]),)
        normalized_features[key] = normalized_ft
    return normalized_features


def _should_convert_joint_value_to_rad(key: str) -> bool:
    if not key.endswith(".pos"):
        return False
    if "gripper" in key:
        return False
    return any(
        token in key
        for token in (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_yaw",
            "wrist_roll",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        )
    )


def _convert_joint_pos_deg_to_rad(values: dict[str, Any]) -> dict[str, Any]:
    converted = dict(values)
    for key, value in values.items():
        if _should_convert_joint_value_to_rad(key):
            converted[key] = math.radians(float(value))
    return converted


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    dataset: StreamingLeRobotDataset | None = None,
    teleop: Teleoperator | None = None,
    control_time_s: int | float | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    display_compressed_images: bool = False,
    joint_unit: str = "rad",
) -> None:
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0.0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)
        dataset_obs = _normalize_joint_value_keys(obs_processed)
        if joint_unit == "rad":
            dataset_obs = _convert_joint_pos_deg_to_rad(dataset_obs)
        observation_frame = (
            build_dataset_frame(dataset.features, dataset_obs, prefix=OBS_STR)
            if dataset is not None
            else {}
        )

        if teleop is None:
            logging.info("No teleoperator provided; skipping action generation.")
            continue

        raw_action = teleop.get_action()
        action_values = teleop_action_processor((raw_action, obs))
        robot_action = robot_action_processor((action_values, obs))
        sent_action = robot.send_action(robot_action)

        if dataset is not None:
            dataset_action = _normalize_joint_value_keys(sent_action)
            if joint_unit == "rad":
                dataset_action = _convert_joint_pos_deg_to_rad(dataset_action)
            action_frame = build_dataset_frame(dataset.features, dataset_action, prefix=ACTION)
            dataset.add_frame({**observation_frame, **action_frame, "task": single_task})

        if display_data:
            log_rerun_data(
                observation=obs_processed,
                action=sent_action,
                compress_images=display_compressed_images,
            )

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(max(1 / fps - dt_s, 0.0))
        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def record_rebot_arm_102(cfg: RebotArm102RecordConfig) -> StreamingLeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if "rebot_arm_102_leader" not in cfg.teleop.type:
        raise ValueError("lerobot-record-rebot-arm-102 requires a reBot Arm 102 teleoperator.")

    if cfg.display_data:
        init_rerun(session_name="rebot_arm_102_recording", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = _build_dataset_features(
        robot=robot,
        teleop_action_processor=teleop_action_processor,
        robot_observation_processor=robot_observation_processor,
        use_videos=cfg.dataset.video,
    )

    dataset = None
    listener = None
    dataset_finalized = False
    try:
        if cfg.resume:
            dataset = StreamingLeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )
            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
        else:
            sanity_check_dataset_name(cfg.dataset.repo_id, None)
            dataset = StreamingLeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )

        robot.connect()
        teleop.connect()
        listener, events = init_keyboard_listener()

        if not cfg.dataset.streaming_encoding:
            logging.info(
                "Streaming encoding is disabled. Enable it with "
                "--dataset.streaming_encoding=true --dataset.vcodec=auto."
            )

        with StreamingVideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                _safe_log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=dataset,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    display_compressed_images=display_compressed_images,
                    joint_unit=cfg.dataset.joint_unit,
                )

                if not events["stop_recording"] and (
                    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
                ):
                    _safe_log_say("Reset the environment", cfg.play_sounds)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        dataset=None,
                        control_time_s=cfg.dataset.reset_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        display_compressed_images=display_compressed_images,
                        joint_unit=cfg.dataset.joint_unit,
                    )

                if events["rerecord_episode"]:
                    _safe_log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
        dataset_finalized = True
    finally:
        _safe_log_say("Stop recording", cfg.play_sounds, blocking=True)
        _disconnect_devices(robot, teleop)
        if dataset and not dataset_finalized:
            dataset.finalize()
        if not is_headless() and listener:
            listener.stop()
        if cfg.display_data:
            rr.rerun_shutdown()
        if dataset and cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
        _safe_log_say("Exiting", cfg.play_sounds)

    return dataset


def main() -> None:
    record_rebot_arm_102()
