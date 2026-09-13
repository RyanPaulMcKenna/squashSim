"""Tests for right-stick camera math without SOFA or a controller."""

import math
import unittest

import numpy as np

from camera_controller import apply_stick_deadzone, orbit_camera, orbit_position


class _Data:
    def __init__(self, value):
        self.value = value


class _Camera:
    def __init__(self):
        self.data = {
            "position": _Data([1.0, 0.5, 1.0]),
            "lookAt": _Data([0.0, 0.2, 0.0]),
            "orientation": _Data([0.0, 0.0, 0.0, 1.0]),
            "distance": _Data(0.0),
        }

    def getData(self, name):
        return self.data[name]

    def getOrientationFromLookAt(self, position, look_at):
        self.orientation_arguments = (position, look_at)
        return [0.1, 0.2, 0.3, 0.9]


class CameraControllerTests(unittest.TestCase):
    def test_deadzone_is_quiet_and_rescales_remaining_range(self):
        self.assertEqual(apply_stick_deadzone(0.10), 0.0)
        self.assertEqual(apply_stick_deadzone(-0.16), 0.0)
        self.assertAlmostEqual(apply_stick_deadzone(1.0), 1.0)
        self.assertAlmostEqual(apply_stick_deadzone(-1.0), -1.0)

    def test_orbit_preserves_radius_and_look_at(self):
        look_at = np.asarray([0.2, 0.3, 0.1])
        position = np.asarray([1.4, 1.0, 1.3])
        result = orbit_position(position, look_at, 0.8, -0.6, 0.2)

        self.assertFalse(np.allclose(result, position))
        self.assertAlmostEqual(
            np.linalg.norm(result - look_at),
            np.linalg.norm(position - look_at),
        )
        self.assertGreater(result[1], position[1])

    def test_zero_input_does_not_move_camera(self):
        position = [1.0, 0.8, 1.0]
        result = orbit_position(position, [0.0, 0.2, 0.0], 0.0, 0.0, 1.0)
        np.testing.assert_array_equal(result, position)

    def test_pitch_is_clamped_away_from_poles(self):
        result = np.asarray([1.0, 0.0, 0.0])
        for _ in range(100):
            result = orbit_position(result, [0.0, 0.0, 0.0], 0.0, -1.0, 1.0)
        elevation = math.asin(result[1] / np.linalg.norm(result))
        self.assertLess(elevation, math.radians(81.0))

    def test_sofa_camera_adapter_sets_position_orientation_and_distance(self):
        camera = _Camera()
        old_radius = np.linalg.norm(
            np.asarray(camera.data["position"].value)
            - np.asarray(camera.data["lookAt"].value)
        )

        self.assertTrue(orbit_camera(camera, 0.8, 0.0, 0.1))
        self.assertEqual(camera.data["orientation"].value, [0.1, 0.2, 0.3, 0.9])
        self.assertAlmostEqual(camera.data["distance"].value, old_radius)
        self.assertEqual(
            camera.orientation_arguments[0], camera.data["position"].value
        )


if __name__ == "__main__":
    unittest.main()
