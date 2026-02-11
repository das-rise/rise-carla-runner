import logging
import os
from datetime import datetime
from typing import Tuple, Optional, List
import pygame
import numpy as np
import carla
import cv2
import queue
import threading
import atexit
import time
from kinetics.vehicle_tools import Actor


class WindowedBirdseyeCamera:
    """
    Displays a top-down RGB camera view in a Pygame window.
    """

    def __init__(self, world, loc: Tuple[float, float, float] = (0, 0, 50)) -> None:
        """
        Initialize the camera and Pygame window.

        Args:
            world (carla.World): The CARLA world instance.
            loc (Tuple[float, float, float]): Camera location in world coordinates.
        """
        pygame.init()
        # Get the spectator (camera) object
        self._world = world
        spectator = self._world.get_spectator()

        self._width = 1920
        self._height = 1080

        # Create a camera sensor
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self._width))
        camera_bp.set_attribute("image_size_y", str(self._height))
        camera_bp.set_attribute("fov", "110")

        # Attach the camera to the spectator
        camera_transform = carla.Transform(
            carla.Location(x=loc[0], y=loc[1], z=loc[2]),
            carla.Rotation(pitch=270, yaw=0.0, roll=0),
        )
        self._camera = world.spawn_actor(
            camera_bp, camera_transform, attach_to=spectator
        )

        # Initialize the display window
        self._screen = pygame.display.set_mode((self._width, self._height))
        pygame.display.set_caption("SpectatorCamera")
        font = pygame.font.Font(None, 40)
        text = font.render("Camera standing by to receive", True, (255, 0, 0))
        self._screen.blit(
            text,
            (
                self._width - text.get_width() - 10,
                self._height - text.get_height() - 10,
            ),
        )
        pygame.display.flip()

    def _process_image(self, image) -> None:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        self._screen.blit(surface, (0, 0))

        # Print timestamp to the bottom right of the screen
        font = pygame.font.Font(None, 30)
        text = font.render(
            f"time: {image.timestamp}\nframe: {self._world.get_snapshot().frame}",
            True,
            (255, 0, 0),
        )
        self._screen.blit(
            text,
            (
                self._width - text.get_width() - 10,
                self._height - text.get_height() - 10,
            ),
        )
        pygame.display.flip()

    def listen(self) -> None:
        """Start listening to the camera stream."""
        self._camera.listen(lambda image: self._process_image(image))


class StreamingCamera:
    """
    Direct MP4 recorder for CARLA top-down (bird's-eye) camera.

    Key features:
      - Direct streaming to MP4 (no per-frame images).
      - Bounded queue + writer thread for back-pressure.
      - Configurable overload policy: 'block' | 'drop_new' | 'drop_oldest'.
      - Optional timestamp overlay.
    """

    def __init__(
        self,
        world,
        loc: Tuple[float, float, float] = (0, 0, 50),
        output_dir: str = "./camera_output",
        video_name: Optional[str] = None,
        fps: float = 30,
        width: int = 1920,
        height: int = 1080,
        queue_max: int = 2048,
        overload_policy: str = "block",  # 'block' | 'drop_new' | 'drop_oldest'
        overlay_timestamp: bool = True,
        preferred_fourccs: Optional[List[str]] = None,  # e.g. ["avc1", "H264", "mp4v"],
        timestamp_offset=0,
        ego_vehicle: Optional[Actor] = None,
    ) -> None:
        """
        Args:
            world: carla.World
            loc: camera world location (x, y, z). Pitch is set to -90 for top-down.
            output_dir: directory for the MP4 file.
            video_name: base name for the MP4 file (timestamped if None).
            fps: writer frame rate (for async sims prefer CARLA synchronous mode).
            width, height: image size.
            queue_max: max frames buffered between sensor and writer.
            overload_policy: behavior when queue is full:
                - 'block'      -> sensor callback blocks (keep every frame)
                - 'drop_new'   -> drop the incoming frame
                - 'drop_oldest'-> drop the oldest queued frame, keep the new one
            overlay_timestamp: overlay "Time: <t>" on each frame (in BGR).
            preferred_fourccs: tried in order until one opens; defaults inside.
            ego_vehicle: optional vehicle to attach camera to. If provided, loc is relative to the vehicle.
        """
        self._world = world
        self._width = width
        self._height = height
        self._fps = fps
        self._overlay_timestamp = overlay_timestamp

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._video_name = video_name or f"birdseye_recording_{ts}"
        self._output_dir = output_dir
        os.makedirs(self._output_dir, exist_ok=True)
        self._video_path = os.path.join(self._output_dir, f"{self._video_name}.mp4")

        # Camera

        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self._width))
        bp.set_attribute("image_size_y", str(self._height))
        bp.set_attribute("fov", "110")

        if ego_vehicle:
            # tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.5))
            tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=50.0), carla.Rotation(pitch=-90, yaw=0.0, roll=0))
            target = ego_vehicle.get_actor()
        else:
            tf = carla.Transform(
                carla.Location(x=loc[0], y=loc[1], z=loc[2]),
                carla.Rotation(pitch=-90, yaw=0.0, roll=0),
            )
            target = self._world.get_spectator()

        self._camera = world.spawn_actor(bp, tf, attach_to=target)

        # Atexit
        self._writer_thread = None
        self._is_recording = False
        atexit.register(self._atexit_cleanup)

        # Queue + threading
        self._queue = queue.Queue(maxsize=queue_max)
        self._overload_policy = overload_policy
        assert self._overload_policy in ("block", "drop_new", "drop_oldest")
        self._SENTINEL = object()
        self._writer_thread: Optional[threading.Thread] = None
        self._is_recording = False

        # Stats
        self._captured = 0
        self._written = 0
        self._dropped = 0

        # Writer (will be opened in writer thread)
        self._fourcc_candidates = preferred_fourccs or [
            "avc1",
            "H264",
            "h264",
            "mp4v",
            "XVID",
        ]
        self._fourcc = None
        self._writer = None

        # Overlay pre-config
        self._font = cv2.FONT_HERSHEY_SIMPLEX
        self._font_scale = 0.8
        self._font_thickness = 2
        self._text_color = (0, 0, 255)  # red in BGR

        # Timestamp offset
        self.timestamp_offset = timestamp_offset

        logging.info(f"Streaming recorder ready -> {self._video_path}")

    # ---------- Internals ----------

    def _open_writer(self) -> None:
        """Try several FourCCs until one opens for (mp4) container."""
        for tag in self._fourcc_candidates:
            fourcc = cv2.VideoWriter_fourcc(*tag)
            writer = cv2.VideoWriter(
                self._video_path, fourcc, self._fps, (self._width, self._height)
            )
            if writer.isOpened():
                self._fourcc = tag
                self._writer = writer
                logging.info(
                    f"Opened VideoWriter with FOURCC='{tag}' at {self._fps} FPS, size {self._width}x{self._height}"
                )
                return
            writer.release()
        raise RuntimeError(
            f"Could not open VideoWriter for {self._video_path} with any FOURCC in {self._fourcc_candidates}. "
            "Ensure your OpenCV build has FFmpeg/GStreamer support."
        )

    def _overlay_time(self, frame_bgr: np.ndarray, timestamp: float) -> None:
        """Overlays a timestamp on the given frame."""
        if not self._overlay_timestamp:
            return
        text = f"Time: {timestamp + self.timestamp_offset:.2f}"
        (tw, th), _ = cv2.getTextSize(
            text, self._font, self._font_scale, self._font_thickness
        )
        x = max(10, self._width - tw - 10)
        y = self._height - 10
        overlay = frame_bgr.copy()
        cv2.rectangle(
            overlay, (x - 6, y - th - 6), (self._width, self._height), (0, 0, 0), -1
        )
        cv2.addWeighted(frame_bgr, 0.72, overlay, 0.28, 0, frame_bgr)
        cv2.putText(
            frame_bgr,
            text,
            (x, y),
            self._font,
            self._font_scale,
            self._text_color,
            self._font_thickness,
            cv2.LINE_AA,
        )

    def _writer_loop(self) -> None:
        """Consumer loop: open writer, pull frames, overlay (optional), write to MP4."""
        try:
            self._open_writer()
        except Exception:
            logging.exception("Failed to open VideoWriter")
            return

        try:
            while True:
                item = self._queue.get()
                try:
                    if item is self._SENTINEL:
                        # Mark done for the sentinel and exit the loop.
                        self._queue.task_done()
                        break

                    frame_bgr, ts = item
                    # Optional overlay
                    self._overlay_time(frame_bgr, ts)
                    # Write frame
                    self._writer.write(frame_bgr)
                    self._written += 1

                except Exception:
                    logging.exception("Writer thread error while processing a frame")
                finally:
                    # Mark the (frame) task done (or sentinel if break didn't hit above)
                    # If we broke above, we already called task_done() for the sentinel.
                    if item is not self._SENTINEL:
                        self._queue.task_done()
        finally:
            # Always release the writer when the loop ends
            try:
                if self._writer is not None:
                    self._writer.release()
            except Exception:
                logging.exception("Failed to release VideoWriter")
            self._writer = None

    def _atexit_cleanup(self):
        """Best-effort safety net to avoid interpreter-finalization crashes."""
        try:
            if self._is_recording:
                self.stop_recording()
            # Do NOT destroy here automatically — the host app might manage actors.
            # But it's generally safe if you own the actor:
            # if self._camera is not None:
            #     self._camera.destroy()
        except Exception:
            # Swallow exceptions at exit
            pass

    # ---------- Sensor callback ----------

    def _process_image(self, image) -> None:
        """Called by CARLA sensor thread; convert BGRA->BGR, apply back-pressure policy, enqueue."""
        if not self._is_recording:
            return

        # CARLA provides BGRA; convert to BGR and copy so CARLA's buffer can be reused.
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4)
        )
        frame_bgr = arr[:, :, :3].copy()
        timestamp = float(image.timestamp)

        item = (frame_bgr, timestamp)

        # Enqueue with chosen overload policy
        try:
            if self._overload_policy == "block":
                self._queue.put(item)  # blocks if full
            elif self._overload_policy == "drop_new":
                self._queue.put_nowait(item)  # drop if full
            else:  # 'drop_oldest'
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    # Remove oldest to make room, and account for it
                    try:
                        self._queue.task_done()
                        self._dropped += 1
                    except queue.Empty:
                        pass
                    # Try put again (should succeed now)
                    self._queue.put_nowait(item)
        except queue.Full:
            # 'drop_new' path falls here -> we drop the new frame
            self._dropped += 1

        self._captured += 1

    # ---------- Public API ----------

    def start_recording(self) -> None:
        if self._is_recording:
            logging.warning("Already recording.")
            return
        self._captured = 0
        self._written = 0
        self._dropped = 0

        # Start writer thread (non-daemon, we will join it)
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="carla-stream-writer", daemon=False
        )
        self._writer_thread.start()

        # Start sensor stream last
        self._is_recording = True
        self._camera.listen(lambda image: self._process_image(image))
        logging.info(
            f"Started streaming to MP4 with policy={self._overload_policy}, "
            f"queue_max={self._queue.maxsize}"
        )

    def stop_recording(self) -> None:
        """Stop recording and flush remaining frames to the video file."""
        if not self._is_recording:
            return

        logging.info("Stopping streaming recorder...")
        self._is_recording = False

        # 1) Stop CARLA sensor first — prevents any further Python callbacks
        try:
            self._camera.stop()
            # Give CARLA's internal callback thread a moment to unwind
            time.sleep(0.05)
        except Exception:
            logging.exception("Error while stopping CARLA camera")

        # 2) Tell writer to finish once the queue is drained
        try:
            self._queue.put(self._SENTINEL)
        except Exception:
            pass

        # 3) Wait for all queued tasks to complete
        try:
            self._queue.join()
        except Exception:
            logging.exception("Error while joining queue")

        # 4) Join writer thread
        if self._writer_thread is not None:
            try:
                self._writer_thread.join(timeout=5.0)
            except Exception:
                logging.exception("Error while joining writer thread")
            finally:
                self._writer_thread = None

        logging.info(
            f"Stopped. Captured: {self._captured}, written: {self._written}, dropped: {self._dropped}. "
            f"Video: {self._video_path}"
        )

    def destroy(self) -> None:
        """Destroy the CARLA camera actor."""
        try:
            if self._is_recording:
                self.stop_recording()
        finally:
            if self._camera is not None:
                try:
                    # Stopping again is harmless; ensure no callbacks linger.
                    self._camera.stop()
                except Exception:
                    pass
                try:
                    self._camera.destroy()
                except Exception:
                    logging.exception("Failed to destroy camera actor")
                self._camera = None

    @property
    def video_path(self) -> str:
        """
        Get the path to the output video file.
        """
        return self._video_path
