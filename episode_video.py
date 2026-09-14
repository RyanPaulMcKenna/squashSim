"""Synchronized capture of the active SOFA viewport for one episode."""

import csv
import os
from pathlib import Path
import shutil
import subprocess


DEFAULT_VIDEO_FPS = 15.0
VIDEO_FILENAME = "episode.mp4"
VIDEO_TIMESTAMPS_FILENAME = "video_frame_timestamps.csv"
TEMPORARY_FRAME_DIRECTORY = "viewport_frames"


def _environment_flag(name, default=True):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _configured_fps(value=None):
    configured = (
        value
        if value is not None
        else os.environ.get("SQUASHSIM_VIDEO_FPS", DEFAULT_VIDEO_FPS)
    )
    fps = float(configured)
    if fps <= 0.0:
        raise ValueError("SQUASHSIM_VIDEO_FPS must be positive")
    return fps


def _sofa_screenshot(filename):
    """Resolve Sofa.Gui lazily because the GUI does not exist at scene load."""
    import Sofa.Gui

    Sofa.Gui.GUIManager.SaveScreenshot(str(filename))


class ViewportVideoRecorder:
    """Capture SOFA viewport images and encode them as an episode MP4.

    Capture cadence is based on monotonic wall time. This makes the resulting
    video play at the rate seen by the operator even when SOFA is running with
    a real-time factor below one. The timestamp table maps every video frame
    back to both the wall and simulation clocks in ``episode.npz``.
    """

    def __init__(
        self,
        fps=None,
        enabled=None,
        keep_frames=None,
        screenshotter=None,
        ffmpeg_executable=None,
        command_runner=None,
    ):
        self.requested_fps = _configured_fps(fps)
        self.enabled = (
            _environment_flag("SQUASHSIM_VIDEO", True)
            if enabled is None
            else bool(enabled)
        )
        self.keep_frames = (
            _environment_flag("SQUASHSIM_KEEP_VIDEO_FRAMES", False)
            if keep_frames is None
            else bool(keep_frames)
        )
        self._screenshotter = screenshotter or _sofa_screenshot
        self._ffmpeg_executable = ffmpeg_executable
        self._command_runner = command_runner or subprocess.run
        self.is_recording = False
        self._episode_directory = None
        self._frame_directory = None
        self._frames = []
        self._next_capture_wall_time = 0.0
        self._capture_error = None

    @property
    def capture_error(self):
        return self._capture_error

    def start(self, episode_directory):
        self._episode_directory = Path(episode_directory).resolve()
        self._frames = []
        self._next_capture_wall_time = 0.0
        self._capture_error = None
        self.is_recording = False
        if not self.enabled:
            return False
        self._frame_directory = (
            self._episode_directory / TEMPORARY_FRAME_DIRECTORY
        )
        try:
            self._frame_directory.mkdir(parents=True, exist_ok=False)
        except Exception as error:
            self._capture_error = str(error)
            return False
        self.is_recording = True
        return True

    def capture(self, timing):
        """Capture the displayed viewport when the wall-time cadence is due."""
        if not self.is_recording or self._capture_error is not None:
            return False

        wall_time_s = float(timing.wall_time_s)
        if wall_time_s + 1.0e-9 < self._next_capture_wall_time:
            return False

        frame_index = len(self._frames)
        frame_path = self._frame_directory / f"frame_{frame_index:06d}.jpg"
        try:
            self._screenshotter(frame_path)
            if not frame_path.is_file() or frame_path.stat().st_size <= 0:
                raise RuntimeError("SOFA did not create the viewport image")
        except Exception as error:
            self._capture_error = str(error)
            return False

        self._frames.append(
            {
                "frame_index": frame_index,
                "sample_index": int(timing.sample_index),
                "sim_time_s": float(timing.sim_time_s),
                "episode_time_s": float(timing.episode_time_s),
                "wall_time_s": wall_time_s,
            }
        )
        period = 1.0 / self.requested_fps
        while self._next_capture_wall_time <= wall_time_s + 1.0e-9:
            self._next_capture_wall_time += period
        return True

    def _write_timestamps(self, encoded_fps):
        path = self._episode_directory / VIDEO_TIMESTAMPS_FILENAME
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "video_frame_index",
                    "video_time_s",
                    "sample_index",
                    "sim_time_s",
                    "episode_time_s",
                    "wall_time_s",
                )
            )
            for frame in self._frames:
                writer.writerow(
                    (
                        frame["frame_index"],
                        frame["frame_index"] / encoded_fps,
                        frame["sample_index"],
                        frame["sim_time_s"],
                        frame["episode_time_s"],
                        frame["wall_time_s"],
                    )
                )
        return path

    def _ffmpeg_path(self):
        configured = self._ffmpeg_executable or os.environ.get(
            "SQUASHSIM_FFMPEG"
        )
        if configured:
            return str(configured)
        return shutil.which("ffmpeg")

    def _encode(self, encoded_fps):
        ffmpeg = self._ffmpeg_path()
        if not ffmpeg:
            raise RuntimeError(
                "ffmpeg was not found; install it or set SQUASHSIM_FFMPEG"
            )
        output_path = self._episode_directory / VIDEO_FILENAME
        command = [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            f"{encoded_fps:.9g}",
            "-i",
            str(self._frame_directory / "frame_%06d.jpg"),
            "-vf",
            "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "22",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_path),
        ]
        completed = self._command_runner(
            command,
            cwd=str(self._episode_directory),
            check=False,
            capture_output=True,
            text=True,
            timeout=120.0,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "").strip()
            raise RuntimeError(
                f"ffmpeg exited with code {completed.returncode}: {detail}"
            )
        if not output_path.is_file() or output_path.stat().st_size <= 0:
            raise RuntimeError("ffmpeg did not create a non-empty episode.mp4")
        return output_path

    def stop_and_encode(self):
        """Finish capture and return JSON-ready metadata for the episode."""
        self.is_recording = False
        result = {
            "enabled": bool(self.enabled),
            "status": "disabled" if not self.enabled else "frames-only",
            "capture_source": "active SOFA GUI viewport",
            "timing_basis": "wall clock with simulation-time mapping",
            "requested_frame_rate_hz": self.requested_fps,
            "encoded_frame_rate_hz": None,
            "frames_captured": len(self._frames),
            "wall_span_s": 0.0,
            "simulation_span_s": 0.0,
            "files": [],
            "error": self._capture_error,
        }
        if not self.enabled:
            return result

        if len(self._frames) > 1:
            wall_span = max(
                0.0,
                self._frames[-1]["wall_time_s"]
                - self._frames[0]["wall_time_s"],
            )
            simulation_span = max(
                0.0,
                self._frames[-1]["sim_time_s"]
                - self._frames[0]["sim_time_s"],
            )
            encoded_fps = (
                (len(self._frames) - 1) / wall_span
                if wall_span > 0.0
                else self.requested_fps
            )
        else:
            wall_span = 0.0
            simulation_span = 0.0
            encoded_fps = self.requested_fps

        result["wall_span_s"] = wall_span
        result["simulation_span_s"] = simulation_span
        result["encoded_frame_rate_hz"] = encoded_fps

        timestamp_path = self._write_timestamps(encoded_fps)
        result["files"].append(timestamp_path.name)

        if not self._frames:
            result["status"] = "failed"
            if result["error"] is None:
                result["error"] = "no viewport frames were captured"

        if self._frames and result["error"] is None:
            try:
                output_path = self._encode(encoded_fps)
                result["status"] = "encoded"
                result["files"].insert(0, output_path.name)
            except Exception as error:
                result["error"] = str(error)

        if result["error"] is not None:
            error_path = self._episode_directory / "video_capture_error.txt"
            error_path.write_text(result["error"] + "\n", encoding="utf-8")
            result["files"].append(error_path.name)

        if (
            result["status"] == "encoded"
            and not self.keep_frames
            and self._frame_directory.exists()
        ):
            shutil.rmtree(self._frame_directory)
        elif self._frame_directory is not None and self._frame_directory.exists():
            result["files"].append(TEMPORARY_FRAME_DIRECTORY + "/")

        return result
