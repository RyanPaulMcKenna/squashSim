"""Viewport video tests without SOFA, a GUI or a real ffmpeg process."""

import csv
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from episode_video import ViewportVideoRecorder


class _CompletedProcess:
    returncode = 0
    stdout = ""
    stderr = ""


class EpisodeVideoTests(unittest.TestCase):
    def test_frames_are_timestamped_encoded_and_temporary_images_removed(self):
        commands = []

        def screenshotter(path):
            Path(path).write_bytes(b"jpeg frame")

        def command_runner(command, **_kwargs):
            commands.append(command)
            Path(command[-1]).write_bytes(b"mp4 video")
            return _CompletedProcess()

        with tempfile.TemporaryDirectory() as temporary_directory:
            episode = Path(temporary_directory) / "episode"
            episode.mkdir()
            recorder = ViewportVideoRecorder(
                fps=10.0,
                screenshotter=screenshotter,
                ffmpeg_executable="ffmpeg-test",
                command_runner=command_runner,
            )
            recorder.start(episode)
            for index, wall_time in enumerate((0.01, 0.05, 0.11, 0.19, 0.21)):
                recorder.capture(
                    SimpleNamespace(
                        sample_index=index,
                        sim_time_s=3.0 + 0.01 * index,
                        episode_time_s=0.01 * index,
                        wall_time_s=wall_time,
                    )
                )
            result = recorder.stop_and_encode()

            self.assertEqual(result["status"], "encoded")
            self.assertEqual(result["frames_captured"], 3)
            self.assertAlmostEqual(result["encoded_frame_rate_hz"], 10.0)
            self.assertTrue((episode / "episode.mp4").is_file())
            self.assertFalse((episode / "viewport_frames").exists())
            self.assertEqual(len(commands), 1)
            self.assertIn("libx264", commands[0])

            with (episode / "video_frame_timestamps.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[-1]["sample_index"], "4")
            self.assertAlmostEqual(float(rows[-1]["video_time_s"]), 0.2)
            self.assertAlmostEqual(float(rows[-1]["wall_time_s"]), 0.21)

    def test_capture_failure_preserves_diagnostic_and_frame_directory(self):
        def failing_screenshotter(_path):
            raise RuntimeError("no active SOFA GUI")

        with tempfile.TemporaryDirectory() as temporary_directory:
            episode = Path(temporary_directory) / "episode"
            episode.mkdir()
            recorder = ViewportVideoRecorder(
                fps=15.0,
                screenshotter=failing_screenshotter,
            )
            recorder.start(episode)
            recorder.capture(
                SimpleNamespace(
                    sample_index=0,
                    sim_time_s=0.01,
                    episode_time_s=0.01,
                    wall_time_s=0.01,
                )
            )
            result = recorder.stop_and_encode()

            self.assertEqual(result["status"], "failed")
            self.assertIn("no active SOFA GUI", result["error"])
            self.assertTrue((episode / "video_capture_error.txt").is_file())
            self.assertTrue((episode / "viewport_frames").is_dir())
            self.assertTrue(
                (episode / "video_frame_timestamps.csv").is_file()
            )


if __name__ == "__main__":
    unittest.main()
