"""Tests for recording/export without SOFA, Tk, Pygame or Matplotlib."""

import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from episode_recorder import EpisodeRecorder, FrameSample


class _Clock:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


def _frame(index, contact_count):
    positions = np.asarray(
        [
            [0.0, 0.10 + 0.01 * index, 0.0],
            [0.5, 0.12 + 0.02 * index, 0.0],
            [1.0, 0.10 + 0.01 * index, 0.0],
        ]
    )
    return FrameSample(
        sim_time_s=5.01 + 0.01 * index,
        simulation_dt_s=0.01,
        joint_position_rad=np.arange(8, dtype=float) + 0.1 * index,
        joint_velocity_rad_s=np.full(8, 0.2 * index),
        commanded_joint_position_rad=np.arange(8, dtype=float) + 0.2,
        object_node_position_m=positions,
        ee_position_m=np.asarray([0.3, 0.4 + 0.03 * index, 0.2]),
        selected_control_index=2,
        speed_index=1,
        command_speed_rad_s=0.45,
        command_delta_rad=0.0045,
        left_x=0.0,
        right_x=0.2,
        right_y=-0.1,
        left_trigger=0.0,
        right_trigger=0.5,
        left_bumper=False,
        right_bumper=False,
        a_button=index == 0,
        gripper_object_contact_count=contact_count,
        floor_object_contact_count=3,
    )


class EpisodeRecorderTests(unittest.TestCase):
    def test_complete_episode_exports_raw_data_tables_and_plots(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            project_root = Path(temporary_directory)
            recorder = EpisodeRecorder(
                project_root=project_root,
                output_root=project_root / "output",
                joint_names=("J1", "J2", "J3", "J4", "J5", "J6", "L", "R"),
                control_labels=("J1", "J2", "J3", "J4", "J5", "J6", "RG2"),
                object_label="flexible cable",
                representative_object_index=1,
                configuration={
                    "robot": "UR5 + OnRobot RG2",
                    "timestep": {"value": 0.01, "unit": "s"},
                },
                clock=_Clock([100.0, 100.01, 100.02, 100.03]),
            )

            recorder.start(5.0)
            recorder.record(_frame(0, 0))
            recorder.record(_frame(1, 2))
            recorder.record(_frame(2, 0))
            output = recorder.stop_and_export()

            expected_files = {
                "episode.npz",
                "samples.csv",
                "metadata.json",
                "configuration.csv",
                "recorded_channels.csv",
                "evidence_summary.md",
                "height_vs_time.svg",
                "contact_activity.svg",
                "joint_positions.svg",
            }
            self.assertTrue(expected_files.issubset({p.name for p in output.iterdir()}))

            with np.load(output / "episode.npz") as arrays:
                self.assertEqual(arrays["joint_position_rad"].shape, (3, 8))
                self.assertEqual(arrays["object_node_position_m"].shape, (3, 3, 3))
                np.testing.assert_array_equal(
                    arrays["contact_event"], [0, 1, -1]
                )
                np.testing.assert_array_equal(
                    arrays["contact_state"], [False, True, False]
                )

            with (output / "samples.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows), 4)
            self.assertIn("joint_velocity_rad_s_j1", rows[0])
            self.assertIn("object_node_02_z_m", rows[0])

            metadata = json.loads(
                (output / "metadata.json").read_text(encoding="utf-8")
            )
            self.assertEqual(metadata["schema_version"], "squashsim-episode-v1")
            self.assertEqual(metadata["representative_object_index"], 1)
            self.assertIn("implementation_id", metadata["source"])
            self.assertAlmostEqual(
                metadata["configuration"]["real-time factor"]["value"],
                1.0,
            )
            self.assertIn(
                "<svg",
                (output / "height_vs_time.svg").read_text(encoding="utf-8"),
            )


if __name__ == "__main__":
    unittest.main()
