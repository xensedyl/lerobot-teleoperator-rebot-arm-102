import concurrent.futures
import contextlib
import logging
import queue
import shutil
import tempfile
import threading
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
import torch
from PIL import Image

from lerobot.datasets.compute_stats import (
    RunningQuantileStats,
    auto_downsample_height_width,
    compute_episode_stats,
)
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import validate_episode_buffer, validate_frame


HW_ENCODERS = [
    "h264_videotoolbox",
    "hevc_videotoolbox",
    "h264_nvenc",
    "hevc_nvenc",
    "h264_vaapi",
    "h264_qsv",
]
BASE_VIDEO_CODECS = {"h264", "hevc", "libsvtav1"}
VALID_VIDEO_CODECS = BASE_VIDEO_CODECS | {"auto"} | set(HW_ENCODERS)


def _mux_packets(container, packets) -> None:
    if packets is None:
        return
    if isinstance(packets, (list, tuple)):
        for packet in packets:
            container.mux(packet)
    else:
        container.mux(packets)


def _get_codec_options(
    vcodec: str,
    g: int | None = 2,
    crf: int | None = 30,
    preset: int | None = None,
) -> dict[str, str]:
    options = {}

    if g is not None and (vcodec in ("h264_videotoolbox", "hevc_videotoolbox") or vcodec not in HW_ENCODERS):
        options["g"] = str(g)

    if crf is not None:
        if vcodec in ("h264", "hevc", "libsvtav1"):
            options["crf"] = str(crf)
        elif vcodec in ("h264_videotoolbox", "hevc_videotoolbox"):
            options["q:v"] = str(max(1, min(100, int(100 - crf * 2))))
        elif vcodec in ("h264_nvenc", "hevc_nvenc"):
            options["rc"] = "constqp"
            options["qp"] = str(crf)
        elif vcodec == "h264_vaapi":
            options["qp"] = str(crf)
        elif vcodec == "h264_qsv":
            options["global_quality"] = str(crf)

    if vcodec == "libsvtav1":
        options["preset"] = str(preset) if preset is not None else "12"

    return options


def detect_available_hw_encoders() -> list[str]:
    available = []
    for codec_name in HW_ENCODERS:
        try:
            av.codec.Codec(codec_name, "w")
        except Exception:
            continue
        available.append(codec_name)
    return available


def resolve_vcodec(vcodec: str) -> str:
    if vcodec not in VALID_VIDEO_CODECS:
        raise ValueError(f"Invalid vcodec '{vcodec}'. Must be one of: {sorted(VALID_VIDEO_CODECS)}")

    if vcodec != "auto":
        logging.info("Using video codec: %s", vcodec)
        return vcodec

    available = detect_available_hw_encoders()
    for encoder in HW_ENCODERS:
        if encoder in available:
            logging.info("Auto-selected video codec: %s", encoder)
            return encoder

    logging.info("No hardware encoder available; falling back to libsvtav1.")
    return "libsvtav1"


class _CameraEncoderThread(threading.Thread):
    def __init__(
        self,
        video_path: Path,
        fps: int,
        vcodec: str,
        pix_fmt: str,
        g: int | None,
        crf: int | None,
        preset: int | None,
        frame_queue: queue.Queue,
        result_queue: queue.Queue,
        stop_event: threading.Event,
        encoder_threads: int | None = None,
    ):
        super().__init__(daemon=True)
        self.video_path = video_path
        self.fps = fps
        self.vcodec = vcodec
        self.pix_fmt = pix_fmt
        self.g = g
        self.crf = crf
        self.preset = preset
        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.stop_event = stop_event
        self.encoder_threads = encoder_threads

    def run(self) -> None:
        container = None
        output_stream = None
        stats_tracker = RunningQuantileStats()
        frame_count = 0

        try:
            logging.getLogger("libav").setLevel(av.logging.WARNING)

            while True:
                try:
                    frame_data = self.frame_queue.get(timeout=1)
                except queue.Empty:
                    if self.stop_event.is_set():
                        break
                    continue

                if frame_data is None:
                    break

                if isinstance(frame_data, np.ndarray):
                    if frame_data.ndim == 3 and frame_data.shape[0] == 3:
                        frame_data = frame_data.transpose(1, 2, 0)
                    if frame_data.dtype != np.uint8:
                        frame_data = (frame_data * 255).astype(np.uint8)

                if container is None:
                    height, width = frame_data.shape[:2]
                    video_options = _get_codec_options(self.vcodec, self.g, self.crf, self.preset)
                    if self.encoder_threads is not None:
                        if self.vcodec == "libsvtav1":
                            lp_param = f"lp={self.encoder_threads}"
                            if "svtav1-params" in video_options:
                                video_options["svtav1-params"] += f":{lp_param}"
                            else:
                                video_options["svtav1-params"] = lp_param
                        else:
                            video_options["threads"] = str(self.encoder_threads)

                    self.video_path.parent.mkdir(parents=True, exist_ok=True)
                    container = av.open(str(self.video_path), "w")
                    output_stream = container.add_stream(self.vcodec, self.fps, options=video_options)
                    output_stream.pix_fmt = self.pix_fmt
                    output_stream.width = width
                    output_stream.height = height
                    output_stream.time_base = Fraction(1, self.fps)

                video_frame = av.VideoFrame.from_image(Image.fromarray(frame_data))
                video_frame.pts = frame_count
                video_frame.time_base = Fraction(1, self.fps)
                _mux_packets(container, output_stream.encode(video_frame))

                img_chw = frame_data.transpose(2, 0, 1)
                img_downsampled = auto_downsample_height_width(img_chw)
                channels = img_downsampled.shape[0]
                img_for_stats = img_downsampled.transpose(1, 2, 0).reshape(-1, channels)
                stats_tracker.update(img_for_stats)
                frame_count += 1

            if output_stream is not None:
                _mux_packets(container, output_stream.encode())

            if container is not None:
                container.close()

            av.logging.restore_default_callback()
            self.result_queue.put(("ok", stats_tracker.get_statistics() if frame_count >= 2 else None))

        except Exception as exc:
            logging.error("Encoder thread error: %s", exc)
            if container is not None:
                with contextlib.suppress(Exception):
                    container.close()
            self.result_queue.put(("error", str(exc)))


class StreamingVideoEncoder:
    def __init__(
        self,
        fps: int,
        vcodec: str = "libsvtav1",
        pix_fmt: str = "yuv420p",
        g: int | None = 2,
        crf: int | None = 30,
        preset: int | None = None,
        queue_maxsize: int = 30,
        encoder_threads: int | None = None,
    ):
        self.fps = fps
        self.vcodec = resolve_vcodec(vcodec)
        self.pix_fmt = pix_fmt
        self.g = g
        self.crf = crf
        self.preset = preset
        self.queue_maxsize = queue_maxsize
        self.encoder_threads = encoder_threads

        self._frame_queues: dict[str, queue.Queue] = {}
        self._result_queues: dict[str, queue.Queue] = {}
        self._threads: dict[str, _CameraEncoderThread] = {}
        self._stop_events: dict[str, threading.Event] = {}
        self._video_paths: dict[str, Path] = {}
        self._dropped_frames: dict[str, int] = {}
        self._episode_active = False

    def start_episode(self, video_keys: list[str], temp_dir: Path) -> None:
        if self._episode_active:
            self.cancel_episode()

        self._dropped_frames.clear()

        for video_key in video_keys:
            frame_queue: queue.Queue = queue.Queue(maxsize=self.queue_maxsize)
            result_queue: queue.Queue = queue.Queue(maxsize=1)
            stop_event = threading.Event()
            temp_video_dir = Path(tempfile.mkdtemp(dir=temp_dir))
            video_path = temp_video_dir / f"{video_key.replace('/', '_')}_streaming.mp4"

            encoder_thread = _CameraEncoderThread(
                video_path=video_path,
                fps=self.fps,
                vcodec=self.vcodec,
                pix_fmt=self.pix_fmt,
                g=self.g,
                crf=self.crf,
                preset=self.preset,
                frame_queue=frame_queue,
                result_queue=result_queue,
                stop_event=stop_event,
                encoder_threads=self.encoder_threads,
            )
            encoder_thread.start()

            self._frame_queues[video_key] = frame_queue
            self._result_queues[video_key] = result_queue
            self._threads[video_key] = encoder_thread
            self._stop_events[video_key] = stop_event
            self._video_paths[video_key] = video_path

        self._episode_active = True

    def feed_frame(self, video_key: str, image: np.ndarray) -> None:
        if not self._episode_active:
            raise RuntimeError("No active episode. Call start_episode() first.")

        thread = self._threads[video_key]
        if not thread.is_alive():
            try:
                status, msg = self._result_queues[video_key].get_nowait()
                if status == "error":
                    raise RuntimeError(f"Encoder thread for {video_key} crashed: {msg}")
            except queue.Empty:
                pass
            raise RuntimeError(f"Encoder thread for {video_key} is not alive")

        try:
            self._frame_queues[video_key].put(image.copy(), timeout=0.1)
        except queue.Full:
            self._dropped_frames[video_key] = self._dropped_frames.get(video_key, 0) + 1
            count = self._dropped_frames[video_key]
            if count == 1 or count % 10 == 0:
                logging.warning(
                    "Encoder queue full for %s, dropped %s frame(s). "
                    "Try --dataset.vcodec=auto or increase --dataset.encoder_queue_maxsize.",
                    video_key,
                    count,
                )

    def finish_episode(self) -> dict[str, tuple[Path, dict | None]]:
        if not self._episode_active:
            raise RuntimeError("No active episode to finish.")

        for video_key, count in self._dropped_frames.items():
            if count > 0:
                logging.warning("Episode finished with %s dropped frame(s) for %s.", count, video_key)

        for video_key in self._frame_queues:
            self._frame_queues[video_key].put(None)

        results = {}
        for video_key in self._threads:
            self._threads[video_key].join(timeout=120)
            if self._threads[video_key].is_alive():
                logging.error("Encoder thread for %s did not finish in time.", video_key)
                self._stop_events[video_key].set()
                self._threads[video_key].join(timeout=5)
                results[video_key] = (self._video_paths[video_key], None)
                continue

            try:
                status, data = self._result_queues[video_key].get(timeout=5)
                if status == "error":
                    raise RuntimeError(f"Encoder thread for {video_key} failed: {data}")
                results[video_key] = (self._video_paths[video_key], data)
            except queue.Empty:
                logging.error("No result from encoder thread for %s.", video_key)
                results[video_key] = (self._video_paths[video_key], None)

        self._cleanup()
        self._episode_active = False
        return results

    def cancel_episode(self) -> None:
        if not self._episode_active:
            return

        for video_key in self._stop_events:
            self._stop_events[video_key].set()

        for video_key in self._threads:
            self._threads[video_key].join(timeout=5)
            video_path = self._video_paths.get(video_key)
            if video_path is not None and video_path.exists():
                shutil.rmtree(str(video_path.parent), ignore_errors=True)

        self._cleanup()
        self._episode_active = False

    def close(self) -> None:
        if self._episode_active:
            self.cancel_episode()

    def _cleanup(self) -> None:
        self._frame_queues.clear()
        self._result_queues.clear()
        self._threads.clear()
        self._stop_events.clear()
        self._video_paths.clear()
        self._dropped_frames.clear()


class StreamingLeRobotDataset(LeRobotDataset):
    def __init__(
        self,
        repo_id: str,
        root: str | Path | None = None,
        episodes: list[int] | None = None,
        image_transforms=None,
        delta_timestamps: dict[str, list[float]] | None = None,
        tolerance_s: float = 1e-4,
        revision: str | None = None,
        force_cache_sync: bool = False,
        download_videos: bool = True,
        video_backend: str | None = None,
        batch_encoding_size: int = 1,
        vcodec: str = "libsvtav1",
        streaming_encoding: bool = False,
        encoder_queue_maxsize: int = 30,
        encoder_threads: int | None = None,
    ):
        resolved_vcodec = resolve_vcodec(vcodec)
        base_vcodec = resolved_vcodec if resolved_vcodec in BASE_VIDEO_CODECS else "libsvtav1"
        super().__init__(
            repo_id=repo_id,
            root=root,
            episodes=episodes,
            image_transforms=image_transforms,
            delta_timestamps=delta_timestamps,
            tolerance_s=tolerance_s,
            revision=revision,
            force_cache_sync=force_cache_sync,
            download_videos=download_videos,
            video_backend=video_backend,
            batch_encoding_size=batch_encoding_size,
            vcodec=base_vcodec,
        )
        self._encoder_threads = encoder_threads
        self._streaming_encoder = None
        self._streaming_queue_maxsize = encoder_queue_maxsize

        if streaming_encoding and len(self.meta.video_keys) > 0:
            self.vcodec = resolved_vcodec
            self._streaming_encoder = StreamingVideoEncoder(
                fps=self.meta.fps,
                vcodec=resolved_vcodec,
                pix_fmt="yuv420p",
                g=2,
                crf=30,
                preset=None,
                queue_maxsize=encoder_queue_maxsize,
                encoder_threads=encoder_threads,
            )
        elif resolved_vcodec not in BASE_VIDEO_CODECS:
            logging.warning(
                "Codec %s is only supported by streaming encoding in this command; "
                "falling back to %s for offline encoding.",
                resolved_vcodec,
                base_vcodec,
            )

    @classmethod
    def create(
        cls,
        repo_id: str,
        fps: int,
        features: dict,
        root: str | Path | None = None,
        robot_type: str | None = None,
        use_videos: bool = True,
        tolerance_s: float = 1e-4,
        image_writer_processes: int = 0,
        image_writer_threads: int = 0,
        video_backend: str | None = None,
        batch_encoding_size: int = 1,
        vcodec: str = "libsvtav1",
        streaming_encoding: bool = False,
        encoder_queue_maxsize: int = 30,
        encoder_threads: int | None = None,
    ) -> "StreamingLeRobotDataset":
        resolved_vcodec = resolve_vcodec(vcodec)
        base_vcodec = resolved_vcodec if resolved_vcodec in BASE_VIDEO_CODECS else "libsvtav1"
        obj = super().create(
            repo_id=repo_id,
            fps=fps,
            features=features,
            root=root,
            robot_type=robot_type,
            use_videos=use_videos,
            tolerance_s=tolerance_s,
            image_writer_processes=image_writer_processes,
            image_writer_threads=image_writer_threads,
            video_backend=video_backend,
            batch_encoding_size=batch_encoding_size,
            vcodec=base_vcodec,
        )
        obj._encoder_threads = encoder_threads
        obj._streaming_encoder = None
        obj._streaming_queue_maxsize = encoder_queue_maxsize

        if streaming_encoding and len(obj.meta.video_keys) > 0:
            obj.vcodec = resolved_vcodec
            obj._streaming_encoder = StreamingVideoEncoder(
                fps=obj.meta.fps,
                vcodec=resolved_vcodec,
                pix_fmt="yuv420p",
                g=2,
                crf=30,
                preset=None,
                queue_maxsize=encoder_queue_maxsize,
                encoder_threads=encoder_threads,
            )
        elif resolved_vcodec not in BASE_VIDEO_CODECS:
            logging.warning(
                "Codec %s is only supported by streaming encoding in this command; "
                "falling back to %s for offline encoding.",
                resolved_vcodec,
                base_vcodec,
            )

        return obj

    def add_frame(self, frame: dict) -> None:
        if self._streaming_encoder is None:
            super().add_frame(frame)
            return

        for name in frame:
            if isinstance(frame[name], torch.Tensor):
                frame[name] = frame[name].numpy()

        validate_frame(frame, self.features)

        if self.episode_buffer is None:
            self.episode_buffer = self.create_episode_buffer()

        frame_index = self.episode_buffer["size"]
        timestamp = frame.pop("timestamp") if "timestamp" in frame else frame_index / self.fps
        self.episode_buffer["frame_index"].append(frame_index)
        self.episode_buffer["timestamp"].append(timestamp)
        self.episode_buffer["task"].append(frame.pop("task"))

        if frame_index == 0:
            self._streaming_encoder.start_episode(video_keys=list(self.meta.video_keys), temp_dir=self.root)

        for key in frame:
            if key not in self.features:
                raise ValueError(
                    f"An element of the frame is not in the features. '{key}' not in '{self.features.keys()}'."
                )

            if self.features[key]["dtype"] == "video":
                self._streaming_encoder.feed_frame(key, frame[key])
                self.episode_buffer[key].append(None)
            elif self.features[key]["dtype"] == "image":
                img_path = self._get_image_file_path(
                    episode_index=self.episode_buffer["episode_index"], image_key=key, frame_index=frame_index
                )
                if frame_index == 0:
                    img_path.parent.mkdir(parents=True, exist_ok=True)
                self._save_image(frame[key], img_path, compress_level=6)
                self.episode_buffer[key].append(str(img_path))
            else:
                self.episode_buffer[key].append(frame[key])

        self.episode_buffer["size"] += 1

    def save_episode(self, episode_data: dict | None = None, parallel_encoding: bool = True) -> None:
        if self._streaming_encoder is None:
            super().save_episode(episode_data=episode_data, parallel_encoding=parallel_encoding)
            return

        episode_buffer = episode_data if episode_data is not None else self.episode_buffer
        validate_episode_buffer(episode_buffer, self.meta.total_episodes, self.features)

        episode_length = episode_buffer.pop("size")
        tasks = episode_buffer.pop("task")
        episode_tasks = list(set(tasks))
        episode_index = episode_buffer["episode_index"]

        episode_buffer["index"] = np.arange(self.meta.total_frames, self.meta.total_frames + episode_length)
        episode_buffer["episode_index"] = np.full((episode_length,), episode_index)
        self.meta.save_episode_tasks(episode_tasks)
        episode_buffer["task_index"] = np.array([self.meta.get_task_index(task) for task in tasks])

        for key, ft in self.features.items():
            if key in ["index", "episode_index", "task_index"] or ft["dtype"] in ["image", "video"]:
                continue
            episode_buffer[key] = np.stack(episode_buffer[key])

        self._wait_image_writer()
        non_video_buffer = {
            key: value
            for key, value in episode_buffer.items()
            if self.features.get(key, {}).get("dtype") != "video"
        }
        non_video_features = {key: value for key, value in self.features.items() if value["dtype"] != "video"}
        ep_stats = compute_episode_stats(non_video_buffer, non_video_features)
        ep_metadata = self._save_episode_data(episode_buffer)

        streaming_results = self._streaming_encoder.finish_episode()
        for video_key in self.meta.video_keys:
            temp_path, video_stats = streaming_results[video_key]
            if video_stats is not None:
                ep_stats[video_key] = {
                    key: value if key == "count" else np.squeeze(value.reshape(1, -1, 1, 1) / 255.0, axis=0)
                    for key, value in video_stats.items()
                }
            ep_metadata.update(self._save_episode_video(video_key, episode_index, temp_path=temp_path))

        self.meta.save_episode(episode_index, episode_length, episode_tasks, ep_stats, ep_metadata)

        if episode_data is None:
            self.clear_episode_buffer(delete_images=len(self.meta.image_keys) > 0)

    def clear_episode_buffer(self, delete_images: bool = True) -> None:
        if self._streaming_encoder is not None:
            self._streaming_encoder.cancel_episode()
        super().clear_episode_buffer(delete_images=delete_images)

    def finalize(self) -> None:
        if self._streaming_encoder is not None:
            self._streaming_encoder.close()
        super().finalize()

    def _encode_temporary_episode_video(self, video_key: str, episode_index: int) -> Path:
        if self._encoder_threads is None:
            return super()._encode_temporary_episode_video(video_key, episode_index)

        temp_path = Path(tempfile.mkdtemp(dir=self.root)) / f"{video_key}_{episode_index:03d}.mp4"
        fpath = self._get_image_file_path(episode_index=episode_index, image_key=video_key, frame_index=0)
        img_dir = fpath.parent
        from lerobot.datasets.video_utils import encode_video_frames

        encode_video_frames(img_dir, temp_path, self.fps, vcodec=self.vcodec, overwrite=True)
        shutil.rmtree(img_dir)
        return temp_path


class StreamingVideoEncodingManager:
    def __init__(self, dataset: StreamingLeRobotDataset):
        self.dataset = dataset

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        streaming_encoder = getattr(self.dataset, "_streaming_encoder", None)

        if streaming_encoder is not None:
            if exc_type is not None:
                streaming_encoder.cancel_episode()
            streaming_encoder.close()
        elif self.dataset.episodes_since_last_encoding > 0:
            start_ep = self.dataset.num_episodes - self.dataset.episodes_since_last_encoding
            end_ep = self.dataset.num_episodes
            logging.info(
                "Encoding remaining %s episode(s), from episode %s to %s.",
                self.dataset.episodes_since_last_encoding,
                start_ep,
                end_ep - 1,
            )
            self.dataset._batch_save_episode_video(start_ep, end_ep)

        self.dataset.finalize()

        if exc_type is not None and streaming_encoder is None:
            interrupted_episode_index = self.dataset.num_episodes
            for key in self.dataset.meta.video_keys:
                img_dir = self.dataset._get_image_file_path(
                    episode_index=interrupted_episode_index,
                    image_key=key,
                    frame_index=0,
                ).parent
                if img_dir.exists():
                    shutil.rmtree(img_dir, ignore_errors=True)

        return False
