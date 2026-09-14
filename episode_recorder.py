"""Synchronous episode recording and evidence export for squashSim."""

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import numpy as np

from episode_plots import write_episode_plots


SCHEMA_VERSION = "squashsim-episode-v2"
REPOSITORY_URL = "https://github.com/RyanPaulMcKenna/squashSim"
DEFAULT_OUTPUT_DIRECTORY = "recordings"


@dataclass(frozen=True)
class FrameSample:
    """All channels captured at one completed SOFA animation step."""

    sim_time_s: float
    simulation_dt_s: float
    joint_position_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    commanded_joint_position_rad: np.ndarray
    object_node_position_m: np.ndarray
    ee_position_m: np.ndarray
    selected_control_index: int
    speed_index: int
    command_speed_rad_s: float
    command_delta_rad: float
    left_x: float
    right_x: float
    right_y: float
    left_trigger: float
    right_trigger: float
    left_bumper: bool
    right_bumper: bool
    a_button: bool
    gripper_object_contact_count: int
    floor_object_contact_count: int


@dataclass(frozen=True)
class SampleTiming:
    """Clock mapping for one sample, also used by viewport video frames."""

    sample_index: int
    sim_time_s: float
    episode_time_s: float
    wall_time_s: float


def _slug(value):
    return re.sub(r"[^a-z0-9]+", "_", str(value).lower()).strip("_")


def _run_git(project_root, *arguments):
    try:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=str(project_root),
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip()


def source_identity(project_root):
    """Return a traceable identity even when the patch is not yet committed."""
    project_root = Path(project_root).resolve()
    source_hasher = hashlib.sha256()
    source_files = sorted(project_root.glob("*.py"))
    for path in source_files:
        source_hasher.update(path.name.encode("utf-8"))
        source_hasher.update(b"\0")
        source_hasher.update(path.read_bytes())
        source_hasher.update(b"\0")
    source_sha256 = source_hasher.hexdigest()

    git_commit = _run_git(project_root, "rev-parse", "HEAD")
    git_status = _run_git(project_root, "status", "--porcelain")
    dirty = None if git_status is None else bool(git_status)
    commit_label = git_commit[:12] if git_commit else "no-git"
    dirty_label = "-dirty" if dirty else ""
    implementation_id = (
        f"{commit_label}{dirty_label}-{source_sha256[:12]}"
    )
    return {
        "repository": REPOSITORY_URL,
        "git_commit": git_commit,
        "working_tree_dirty": dirty,
        "source_sha256": source_sha256,
        "source_files": [path.name for path in source_files],
        "implementation_id": implementation_id,
    }


def _configuration_entry(value, unit=""):
    return {"value": value, "unit": unit}


def _safe_markdown(value):
    return str(value).replace("|", "\\|").replace("\n", " ")


class EpisodeRecorder:
    """Buffer one A-button-delimited episode and export it on completion."""

    def __init__(
        self,
        project_root,
        joint_names,
        control_labels,
        object_label="deformable object",
        representative_object_index=None,
        configuration=None,
        output_root=None,
        clock=None,
    ):
        self.project_root = Path(project_root).resolve()
        configured_output = output_root or os.environ.get(
            "SQUASHSIM_RECORDING_DIR", DEFAULT_OUTPUT_DIRECTORY
        )
        configured_output = Path(configured_output).expanduser()
        if not configured_output.is_absolute():
            configured_output = self.project_root / configured_output
        self.output_root = configured_output.resolve()
        self.joint_names = tuple(str(name) for name in joint_names)
        self.control_labels = tuple(str(name) for name in control_labels)
        self.object_label = str(object_label)
        self.representative_object_index = representative_object_index
        self.configuration = dict(configuration or {})
        self.clock = clock or time.monotonic
        self.is_recording = False
        self.episode_directory = None
        self.last_export_directory = None
        self._frames = []
        self._start_wall_time = None
        self._start_sim_time = None
        self._started_utc = None
        self._object_node_count = None

    @property
    def recorded_duration_s(self):
        if not self.is_recording or self._start_wall_time is None:
            return 0.0
        return max(0.0, float(self.clock()) - self._start_wall_time)

    def _new_episode_directory(self):
        self.output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S_%fZ"
        )
        base = self.output_root / f"episode_{timestamp}"
        candidate = base
        suffix = 1
        while candidate.exists():
            candidate = self.output_root / f"{base.name}_{suffix:02d}"
            suffix += 1
        candidate.mkdir()
        return candidate

    def start(self, sim_time_s):
        if self.is_recording:
            return self.episode_directory
        self._frames = []
        self._start_wall_time = float(self.clock())
        self._start_sim_time = float(sim_time_s)
        self._started_utc = datetime.now(timezone.utc)
        self._object_node_count = None
        self.episode_directory = self._new_episode_directory()
        self.is_recording = True
        return self.episode_directory

    def record(self, sample):
        if not self.is_recording:
            return None
        if not isinstance(sample, FrameSample):
            raise TypeError("sample must be a FrameSample")

        joint_position = np.asarray(
            sample.joint_position_rad, dtype=float
        ).reshape(-1)
        joint_velocity = np.asarray(
            sample.joint_velocity_rad_s, dtype=float
        ).reshape(-1)
        commanded_position = np.asarray(
            sample.commanded_joint_position_rad, dtype=float
        ).reshape(-1)
        expected_joints = len(self.joint_names)
        for name, values in (
            ("joint position", joint_position),
            ("joint velocity", joint_velocity),
            ("commanded joint position", commanded_position),
        ):
            if values.size != expected_joints or not np.isfinite(values).all():
                raise ValueError(
                    f"{name} must contain {expected_joints} finite values"
                )

        object_position = np.asarray(
            sample.object_node_position_m, dtype=float
        )
        if object_position.size == 0:
            object_position = np.empty((0, 3), dtype=float)
        if (
            object_position.ndim != 2
            or object_position.shape[1] != 3
            or not np.isfinite(object_position).all()
        ):
            raise ValueError("object node positions must have shape (N, 3)")
        if self._object_node_count is None:
            self._object_node_count = len(object_position)
        elif len(object_position) != self._object_node_count:
            raise ValueError("object node count changed during the episode")

        ee_position = np.asarray(sample.ee_position_m, dtype=float).reshape(-1)
        if ee_position.size != 3 or not np.isfinite(ee_position).all():
            raise ValueError("end-effector position must contain 3 finite values")

        wall_time_s = max(
            0.0, float(self.clock()) - self._start_wall_time
        )
        sample_index = len(self._frames)
        self._frames.append(
            {
                "wall_time_s": wall_time_s,
                "sim_time_s": float(sample.sim_time_s),
                "simulation_dt_s": float(sample.simulation_dt_s),
                "joint_position_rad": joint_position.copy(),
                "joint_velocity_rad_s": joint_velocity.copy(),
                "commanded_joint_position_rad": commanded_position.copy(),
                "object_node_position_m": object_position.copy(),
                "ee_position_m": ee_position.copy(),
                "selected_control_index": int(sample.selected_control_index),
                "speed_index": int(sample.speed_index),
                "command_speed_rad_s": float(sample.command_speed_rad_s),
                "command_delta_rad": float(sample.command_delta_rad),
                "left_x": float(sample.left_x),
                "right_x": float(sample.right_x),
                "right_y": float(sample.right_y),
                "left_trigger": float(sample.left_trigger),
                "right_trigger": float(sample.right_trigger),
                "left_bumper": bool(sample.left_bumper),
                "right_bumper": bool(sample.right_bumper),
                "a_button": bool(sample.a_button),
                "gripper_object_contact_count": max(
                    0, int(sample.gripper_object_contact_count)
                ),
                "floor_object_contact_count": max(
                    0, int(sample.floor_object_contact_count)
                ),
            }
        )
        return SampleTiming(
            sample_index=sample_index,
            sim_time_s=float(sample.sim_time_s),
            episode_time_s=float(sample.sim_time_s) - self._start_sim_time,
            wall_time_s=wall_time_s,
        )

    def _representative_index(self, node_count):
        if node_count <= 0:
            return None
        if self.representative_object_index is None:
            return node_count // 2
        return max(0, min(node_count - 1, int(self.representative_object_index)))

    def _arrays(self):
        if not self._frames:
            raise ValueError("the recording contains no completed simulation steps")

        scalar_float_names = (
            "wall_time_s",
            "sim_time_s",
            "simulation_dt_s",
            "command_speed_rad_s",
            "command_delta_rad",
            "left_x",
            "right_x",
            "right_y",
            "left_trigger",
            "right_trigger",
        )
        scalar_integer_names = (
            "selected_control_index",
            "speed_index",
            "gripper_object_contact_count",
            "floor_object_contact_count",
        )
        scalar_boolean_names = (
            "left_bumper",
            "right_bumper",
            "a_button",
        )
        arrays = {
            name: np.asarray([frame[name] for frame in self._frames], dtype=float)
            for name in scalar_float_names
        }
        arrays.update(
            {
                name: np.asarray(
                    [frame[name] for frame in self._frames], dtype=np.int32
                )
                for name in scalar_integer_names
            }
        )
        arrays.update(
            {
                name: np.asarray(
                    [frame[name] for frame in self._frames], dtype=bool
                )
                for name in scalar_boolean_names
            }
        )
        for name in (
            "joint_position_rad",
            "joint_velocity_rad_s",
            "commanded_joint_position_rad",
            "object_node_position_m",
            "ee_position_m",
        ):
            arrays[name] = np.stack(
                [frame[name] for frame in self._frames], axis=0
            )

        arrays["episode_time_s"] = (
            arrays["sim_time_s"] - float(self._start_sim_time)
        )
        object_positions = arrays["object_node_position_m"]
        if object_positions.shape[1] == 0:
            arrays["object_center_position_m"] = np.empty(
                (len(self._frames), 0), dtype=float
            )
            arrays["object_representative_position_m"] = np.empty(
                (len(self._frames), 0), dtype=float
            )
        else:
            arrays["object_center_position_m"] = object_positions.mean(axis=1)
            representative_index = self._representative_index(
                object_positions.shape[1]
            )
            arrays["object_representative_position_m"] = object_positions[
                :, representative_index, :
            ]

        arrays["contact_state"] = (
            arrays["gripper_object_contact_count"] > 0
        )
        previous_state = np.concatenate(
            (np.asarray([False]), arrays["contact_state"][:-1])
        )
        arrays["contact_event"] = (
            arrays["contact_state"].astype(np.int8)
            - previous_state.astype(np.int8)
        )
        arrays["sample_index"] = np.arange(len(self._frames), dtype=np.int32)
        return arrays

    def _channel_rows(self, arrays):
        samples = len(self._frames)
        joints = len(self.joint_names)
        nodes = arrays["object_node_position_m"].shape[1]
        summary_coordinates = 3 if nodes else 0
        return [
            ("sim_time_s", f"[{samples}]", "s", "SOFA simulation timestamp"),
            ("wall_time_s", f"[{samples}]", "s", "Monotonic wall timestamp from recording start"),
            ("simulation_dt_s", f"[{samples}]", "s", "SOFA timestep used by each sample"),
            ("joint_position_rad", f"[{samples}, {joints}]", "rad", "Solved UR5 and RG2 joint positions"),
            ("joint_velocity_rad_s", f"[{samples}, {joints}]", "rad/s", "Solved UR5 and RG2 joint velocities"),
            ("commanded_joint_position_rad", f"[{samples}, {joints}]", "rad", "Slider/controller joint targets"),
            ("object_node_position_m", f"[{samples}, {nodes}, 3]", "m", "All deformable-object node positions"),
            ("ee_position_m", f"[{samples}, 3]", "m", "Mean position of the two RG2 finger rigid origins"),
            ("object_center_position_m", f"[{samples}, {summary_coordinates}]", "m", "Mean deformable-object node position"),
            ("object_representative_position_m", f"[{samples}, {summary_coordinates}]", "m", "Representative central object node"),
            ("gripper_object_contact_count", f"[{samples}]", "count", "Contacts reported between RG2 meshes and the object"),
            ("floor_object_contact_count", f"[{samples}]", "count", "Contacts reported between the floor and the object"),
            ("contact_state", f"[{samples}]", "bool", "True while gripper-object contact count is non-zero"),
            ("contact_event", f"[{samples}]", "-", "+1 contact began, -1 contact ended, 0 unchanged"),
            ("controller_action", f"[{samples}]", "mixed", "Selected joint, speed, trigger delta, sticks and buttons"),
        ]

    def _episode_configuration(self, arrays, identity, video_metadata=None):
        episode_duration = max(0.0, float(arrays["episode_time_s"][-1]))
        wall_duration = max(0.0, float(arrays["wall_time_s"][-1]))
        real_time_factor = (
            episode_duration / wall_duration if wall_duration > 0.0 else None
        )
        sample_frequency = (
            len(self._frames) / wall_duration
            if wall_duration > 0.0
            else None
        )
        configuration = {
            key: (
                value
                if isinstance(value, dict) and "value" in value
                else _configuration_entry(value)
            )
            for key, value in self.configuration.items()
        }
        configuration.update(
            {
                "episode duration": _configuration_entry(episode_duration, "s"),
                "wall duration": _configuration_entry(wall_duration, "s"),
                "recorded samples": _configuration_entry(len(self._frames), "count"),
                "measured sample frequency": _configuration_entry(sample_frequency, "Hz"),
                "real-time factor": _configuration_entry(real_time_factor, "sim s / wall s"),
                "implementation identifier": _configuration_entry(
                    identity["implementation_id"]
                ),
            }
        )
        if video_metadata:
            configuration.update(
                {
                    "viewport video status": _configuration_entry(
                        video_metadata.get("status", "unknown")
                    ),
                    "viewport video frames": _configuration_entry(
                        video_metadata.get("frames_captured", 0), "count"
                    ),
                    "viewport video frame rate": _configuration_entry(
                        video_metadata.get("encoded_frame_rate_hz"), "Hz"
                    ),
                }
            )
        return configuration

    def _write_npz(self, directory, arrays):
        np.savez_compressed(directory / "episode.npz", **arrays)

    def _write_samples_csv(self, directory, arrays):
        header = [
            "sample_index",
            "sim_time_s",
            "episode_time_s",
            "wall_time_s",
            "simulation_dt_s",
            "selected_control_index",
            "selected_control_label",
            "speed_index",
            "command_speed_rad_s",
            "command_delta_rad",
            "left_x",
            "right_x",
            "right_y",
            "left_trigger",
            "right_trigger",
            "left_bumper",
            "right_bumper",
            "a_button",
            "gripper_object_contact_count",
            "floor_object_contact_count",
            "contact_state",
            "contact_event",
            "ee_x_m",
            "ee_y_m",
            "ee_z_m",
            "object_center_x_m",
            "object_center_y_m",
            "object_center_z_m",
            "object_point_x_m",
            "object_point_y_m",
            "object_point_z_m",
        ]
        for prefix in (
            "joint_position_rad",
            "joint_velocity_rad_s",
            "commanded_joint_position_rad",
        ):
            header.extend(f"{prefix}_{_slug(name)}" for name in self.joint_names)
        node_count = arrays["object_node_position_m"].shape[1]
        header.extend(
            f"object_node_{node:02d}_{axis}_m"
            for node in range(node_count)
            for axis in "xyz"
        )

        with (directory / "samples.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            for index in range(len(self._frames)):
                selected_index = int(arrays["selected_control_index"][index])
                selected_label = (
                    self.control_labels[selected_index]
                    if 0 <= selected_index < len(self.control_labels)
                    else "unknown"
                )
                if node_count:
                    object_center = arrays["object_center_position_m"][index]
                    object_point = arrays[
                        "object_representative_position_m"
                    ][index]
                else:
                    object_center = [float("nan")] * 3
                    object_point = [float("nan")] * 3
                row = [
                    int(arrays["sample_index"][index]),
                    float(arrays["sim_time_s"][index]),
                    float(arrays["episode_time_s"][index]),
                    float(arrays["wall_time_s"][index]),
                    float(arrays["simulation_dt_s"][index]),
                    selected_index,
                    selected_label,
                    int(arrays["speed_index"][index]),
                    float(arrays["command_speed_rad_s"][index]),
                    float(arrays["command_delta_rad"][index]),
                    float(arrays["left_x"][index]),
                    float(arrays["right_x"][index]),
                    float(arrays["right_y"][index]),
                    float(arrays["left_trigger"][index]),
                    float(arrays["right_trigger"][index]),
                    int(arrays["left_bumper"][index]),
                    int(arrays["right_bumper"][index]),
                    int(arrays["a_button"][index]),
                    int(arrays["gripper_object_contact_count"][index]),
                    int(arrays["floor_object_contact_count"][index]),
                    int(arrays["contact_state"][index]),
                    int(arrays["contact_event"][index]),
                    *arrays["ee_position_m"][index].tolist(),
                    *np.asarray(object_center).tolist(),
                    *np.asarray(object_point).tolist(),
                    *arrays["joint_position_rad"][index].tolist(),
                    *arrays["joint_velocity_rad_s"][index].tolist(),
                    *arrays["commanded_joint_position_rad"][index].tolist(),
                    *arrays["object_node_position_m"][index].reshape(-1).tolist(),
                ]
                writer.writerow(row)

    @staticmethod
    def _write_table_csv(path, header, rows):
        with Path(path).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)

    def _write_evidence_summary(
        self,
        directory,
        configuration,
        channel_rows,
        identity,
        plot_paths,
        video_metadata=None,
    ):
        lines = [
            "# squashSim episode evidence",
            "",
            (
                f"Implementation `{identity['implementation_id']}`; "
                f"raw data schema `{SCHEMA_VERSION}`."
            ),
            "",
            "## Configuration",
            "",
            "| Field | Value | Unit |",
            "| --- | ---: | --- |",
        ]
        for name, entry in configuration.items():
            lines.append(
                f"| {_safe_markdown(name)} | "
                f"{_safe_markdown(entry['value'])} | "
                f"{_safe_markdown(entry.get('unit', ''))} |"
            )
        lines.extend(
            [
                "",
                "## Recorded channels",
                "",
                "| Channel | Shape | Unit | Description |",
                "| --- | --- | --- | --- |",
            ]
        )
        for name, shape, unit, description in channel_rows:
            lines.append(
                f"| `{_safe_markdown(name)}` | {_safe_markdown(shape)} | "
                f"{_safe_markdown(unit)} | {_safe_markdown(description)} |"
            )
        if video_metadata:
            lines.extend(
                [
                    "",
                    "## Synchronized viewport video",
                    "",
                    (
                        f"Status: `{_safe_markdown(video_metadata.get('status', 'unknown'))}`; "
                        f"frames: {video_metadata.get('frames_captured', 0)}."
                    ),
                    "",
                ]
            )
            if video_metadata.get("status") == "encoded":
                lines.append("[Play the recorded SOFA viewport](episode.mp4)")
                lines.append("")
            if "video_frame_timestamps.csv" in video_metadata.get("files", []):
                lines.append(
                    "`video_frame_timestamps.csv` maps every video frame to "
                    "the recorded sample, simulation time and monotonic wall time."
                )
                lines.append("")
        lines.extend(["", "## Plots", ""])
        for path in plot_paths:
            label = path.stem.replace("_", " ").title()
            lines.append(f"![{label}]({path.name})")
            lines.append("")
        lines.extend(
            [
                "## Raw outputs",
                "",
                "- `episode.npz`: compact, shape-preserving NumPy arrays.",
                "- `samples.csv`: flat sample table including every object node.",
                "- `metadata.json`: configuration, channel schema and traceability.",
                "- `configuration.csv` and `recorded_channels.csv`: appendix-ready tables.",
            ]
        )
        if video_metadata:
            if video_metadata.get("status") == "encoded":
                lines.append(
                    "- `episode.mp4`: synchronized recording of the active SOFA viewport."
                )
            if "video_frame_timestamps.csv" in video_metadata.get("files", []):
                lines.append(
                    "- `video_frame_timestamps.csv`: video-frame to episode-clock mapping."
                )
        lines.extend(
            [
                "",
                "Contact force is not exported because SOFA 25.06's Python "
                "`ContactListener` exposes contact counts, points and elements, "
                "but not solved contact forces.",
                "",
            ]
        )
        (directory / "evidence_summary.md").write_text(
            "\n".join(lines), encoding="utf-8"
        )

    def stop_and_export(self, video_metadata=None, ended_utc=None):
        if not self.is_recording:
            return self.last_export_directory
        self.is_recording = False
        directory = self.episode_directory
        video_metadata = dict(video_metadata or {})
        if ended_utc is None:
            ended_utc = datetime.now(timezone.utc)
        ended_utc_text = (
            ended_utc.isoformat()
            if hasattr(ended_utc, "isoformat")
            else str(ended_utc)
        )
        arrays = self._arrays()
        identity = source_identity(self.project_root)
        configuration = self._episode_configuration(
            arrays, identity, video_metadata
        )
        channel_rows = self._channel_rows(arrays)

        self._write_npz(directory, arrays)
        self._write_samples_csv(directory, arrays)
        self._write_table_csv(
            directory / "configuration.csv",
            ("field", "value", "unit"),
            [
                (name, entry["value"], entry.get("unit", ""))
                for name, entry in configuration.items()
            ],
        )
        self._write_table_csv(
            directory / "recorded_channels.csv",
            ("channel", "shape", "unit", "description"),
            channel_rows,
        )
        plot_paths = write_episode_plots(
            directory, arrays, self.joint_names, self.object_label
        )

        video_files = [
            filename
            for filename in video_metadata.get("files", [])
            if (directory / filename.rstrip("/")).exists()
        ]
        if video_metadata:
            video_metadata["files"] = video_files

        metadata = {
            "schema_version": SCHEMA_VERSION,
            "recording_started_utc": self._started_utc.isoformat(),
            "recording_ended_utc": ended_utc_text,
            "object_label": self.object_label,
            "representative_object_index": self._representative_index(
                arrays["object_node_position_m"].shape[1]
            ),
            "joint_names": list(self.joint_names),
            "configuration": configuration,
            "channels": [
                {
                    "name": name,
                    "shape": shape,
                    "unit": unit,
                    "description": description,
                }
                for name, shape, unit, description in channel_rows
            ],
            "contact_force_available": False,
            "source": identity,
            "video": video_metadata or None,
            "files": [
                "episode.npz",
                "samples.csv",
                "metadata.json",
                "configuration.csv",
                "recorded_channels.csv",
                "evidence_summary.md",
                *[path.name for path in plot_paths],
                *video_files,
            ],
        }
        (directory / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self._write_evidence_summary(
            directory,
            configuration,
            channel_rows,
            identity,
            plot_paths,
            video_metadata,
        )
        self.last_export_directory = directory
        return directory
