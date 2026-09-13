"""Dependency-free SVG plots for a recorded squashSim episode."""

from html import escape
from pathlib import Path

import numpy as np


PLOT_COLORS = (
    "#1769aa",
    "#d95f02",
    "#2e7d32",
    "#7b1fa2",
    "#00838f",
    "#c62828",
    "#6d4c41",
    "#455a64",
)


def _finite_bounds(values, fallback=(0.0, 1.0)):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return fallback
    lower = float(finite.min())
    upper = float(finite.max())
    if np.isclose(lower, upper):
        padding = max(0.05 * abs(lower), 0.05)
    else:
        padding = 0.06 * (upper - lower)
    return lower - padding, upper + padding


def _polyline_points(x_values, y_values, map_x, map_y):
    points = []
    for x_value, y_value in zip(x_values, y_values):
        if np.isfinite(x_value) and np.isfinite(y_value):
            points.append(f"{map_x(x_value):.2f},{map_y(y_value):.2f}")
    return " ".join(points)


def write_line_plot(
    path,
    title,
    x_values,
    series,
    x_label,
    y_label,
    y_bounds=None,
):
    """Write a compact, report-ready SVG line plot.

    ``series`` is an iterable of ``(label, values, colour)`` tuples.
    """
    path = Path(path)
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    prepared = [
        (str(label), np.asarray(values, dtype=float).reshape(-1), str(colour))
        for label, values, colour in series
    ]
    if x_values.size == 0:
        raise ValueError("a plot requires at least one sample")
    for label, values, _colour in prepared:
        if values.size != x_values.size:
            raise ValueError(f"series {label!r} has the wrong sample count")

    x_lower, x_upper = _finite_bounds(x_values)
    if y_bounds is None:
        combined = np.concatenate([values for _label, values, _colour in prepared])
        y_lower, y_upper = _finite_bounds(combined)
    else:
        y_lower, y_upper = (float(y_bounds[0]), float(y_bounds[1]))
        if y_upper <= y_lower:
            y_upper = y_lower + 1.0

    width, height = 1000, 560
    legend_rows = max(1, (len(prepared) + 2) // 3)
    left, right = 88, 30
    top = 68 + 22 * (legend_rows - 1)
    bottom = 72
    plot_width = width - left - right
    plot_height = height - top - bottom

    def map_x(value):
        return left + (float(value) - x_lower) * plot_width / (x_upper - x_lower)

    def map_y(value):
        return top + (y_upper - float(value)) * plot_height / (y_upper - y_lower)

    fragments = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        '<g font-family="DejaVu Sans,Arial,sans-serif" fill="#202124">',
        f'<text x="{width / 2:.1f}" y="32" text-anchor="middle" '
        f'font-size="22" font-weight="600">{escape(title)}</text>',
    ]

    tick_count = 6
    for index in range(tick_count):
        fraction = index / (tick_count - 1)
        x_value = x_lower + fraction * (x_upper - x_lower)
        pixel_x = left + fraction * plot_width
        fragments.extend(
            [
                f'<line x1="{pixel_x:.2f}" y1="{top}" x2="{pixel_x:.2f}" '
                f'y2="{top + plot_height}" stroke="#e4e7eb" stroke-width="1"/>',
                f'<text x="{pixel_x:.2f}" y="{top + plot_height + 24}" '
                f'text-anchor="middle" font-size="13">{x_value:.2f}</text>',
            ]
        )

        y_value = y_lower + fraction * (y_upper - y_lower)
        pixel_y = top + (1.0 - fraction) * plot_height
        fragments.extend(
            [
                f'<line x1="{left}" y1="{pixel_y:.2f}" '
                f'x2="{left + plot_width}" y2="{pixel_y:.2f}" '
                f'stroke="#e4e7eb" stroke-width="1"/>',
                f'<text x="{left - 12}" y="{pixel_y + 4:.2f}" '
                f'text-anchor="end" font-size="13">{y_value:.3g}</text>',
            ]
        )

    fragments.extend(
        [
            f'<line x1="{left}" y1="{top + plot_height}" '
            f'x2="{left + plot_width}" y2="{top + plot_height}" '
            f'stroke="#343a40" stroke-width="1.5"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" '
            f'y2="{top + plot_height}" stroke="#343a40" stroke-width="1.5"/>',
        ]
    )

    for label, values, colour in prepared:
        points = _polyline_points(x_values, values, map_x, map_y)
        fragments.append(
            f'<polyline points="{points}" fill="none" stroke="{colour}" '
            'stroke-width="2.4" stroke-linejoin="round" '
            'stroke-linecap="round"/>'
        )

    legend_x = left + 12
    legend_y = 58
    for index, (label, _values, colour) in enumerate(prepared):
        row = index // 3
        column = index % 3
        item_x = legend_x + column * 285
        item_y = legend_y + row * 22
        fragments.extend(
            [
                f'<line x1="{item_x}" y1="{item_y}" '
                f'x2="{item_x + 26}" y2="{item_y}" '
                f'stroke="{colour}" stroke-width="3"/>',
                f'<text x="{item_x + 34}" y="{item_y + 4}" '
                f'font-size="13">{escape(label)}</text>',
            ]
        )

    fragments.extend(
        [
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 20}" '
            f'text-anchor="middle" font-size="15">{escape(x_label)}</text>',
            f'<text x="22" y="{top + plot_height / 2:.2f}" '
            f'text-anchor="middle" font-size="15" '
            f'transform="rotate(-90 22 {top + plot_height / 2:.2f})">'
            f'{escape(y_label)}</text>',
            "</g>",
            "</svg>",
        ]
    )
    path.write_text("\n".join(fragments) + "\n", encoding="utf-8")
    return path


def write_episode_plots(output_directory, arrays, joint_names, object_label):
    """Generate the three evidence plots and return their paths."""
    output_directory = Path(output_directory)
    time_values = np.asarray(arrays["episode_time_s"], dtype=float)
    if time_values.size == 0:
        return []

    height_series = [
        ("End effector", arrays["ee_position_m"][:, 1], PLOT_COLORS[0]),
    ]
    if arrays["object_center_position_m"].shape[1] == 3:
        height_series.extend(
            [
                (
                    f"{object_label} centre",
                    arrays["object_center_position_m"][:, 1],
                    PLOT_COLORS[1],
                ),
                (
                    f"{object_label} point",
                    arrays["object_representative_position_m"][:, 1],
                    PLOT_COLORS[2],
                ),
            ]
        )
    paths = [
        write_line_plot(
            output_directory / "height_vs_time.svg",
            "End-effector and deformable-object height",
            time_values,
            height_series,
            "Simulated episode time (s)",
            "Y height (m)",
        )
    ]

    gripper_contacts = np.asarray(
        arrays["gripper_object_contact_count"], dtype=float
    )
    floor_contacts = np.asarray(
        arrays["floor_object_contact_count"], dtype=float
    )
    contact_state = np.asarray(arrays["contact_state"], dtype=float)
    contact_upper = max(
        1.0,
        float(gripper_contacts.max(initial=0.0)),
        float(floor_contacts.max(initial=0.0)),
    )
    paths.append(
        write_line_plot(
            output_directory / "contact_activity.svg",
            "Contact activity",
            time_values,
            [
                ("Gripper-object contacts", gripper_contacts, PLOT_COLORS[1]),
                ("Floor-object contacts", floor_contacts, PLOT_COLORS[4]),
                ("Gripper contact state", contact_state, PLOT_COLORS[2]),
            ],
            "Simulated episode time (s)",
            "Contact count / state",
            y_bounds=(0.0, contact_upper * 1.08),
        )
    )

    joint_position = np.asarray(arrays["joint_position_rad"], dtype=float)
    joint_series = [
        (
            str(name),
            joint_position[:, index],
            PLOT_COLORS[index % len(PLOT_COLORS)],
        )
        for index, name in enumerate(joint_names)
    ]
    paths.append(
        write_line_plot(
            output_directory / "joint_positions.svg",
            "Recorded robot joint positions",
            time_values,
            joint_series,
            "Simulated episode time (s)",
            "Joint position (rad)",
        )
    )
    return paths
