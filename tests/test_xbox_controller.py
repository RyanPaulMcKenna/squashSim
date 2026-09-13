"""Tests for Xbox input without requiring SOFA, Tk, or real hardware."""

import unittest

from xbox_controller import (
    DEFAULT_SPEED_INDEX,
    GamepadSample,
    PygameXboxBackend,
    SPEED_PRESETS,
    XboxControlState,
)


def connected_sample(**kwargs):
    values = {
        "connected": True,
        "name": "Test Xbox Pad",
        "status": "Connected: Test Xbox Pad",
    }
    values.update(kwargs)
    return GamepadSample(**values)


class XboxControlStateTests(unittest.TestCase):
    def test_stick_flick_cycles_once_until_returned_to_neutral(self):
        state = XboxControlState(control_count=7)

        update = state.step(connected_sample(left_x=0.8), 0.005)
        self.assertEqual(update.selected_index, 1)
        update = state.step(connected_sample(left_x=1.0), 0.005)
        self.assertEqual(update.selected_index, 1)

        state.step(connected_sample(left_x=0.0), 0.005)
        update = state.step(connected_sample(left_x=0.8), 0.005)
        self.assertEqual(update.selected_index, 2)

        state = XboxControlState(control_count=7)
        update = state.step(connected_sample(left_x=-0.8), 0.005)
        self.assertEqual(update.selected_index, 6)

    def test_bumpers_change_speed_once_per_press_and_stop_at_limits(self):
        state = XboxControlState()
        self.assertEqual(state.speed_index, DEFAULT_SPEED_INDEX)

        state.step(connected_sample(left_bumper=True), 0.005)
        self.assertEqual(state.speed_index, 0)
        state.step(connected_sample(left_bumper=True), 0.005)
        self.assertEqual(state.speed_index, 0)

        state.step(connected_sample(), 0.005)
        state.step(connected_sample(right_bumper=True), 0.005)
        self.assertEqual(state.speed_index, 1)
        state.step(connected_sample(), 0.005)
        state.step(connected_sample(right_bumper=True), 0.005)
        self.assertEqual(state.speed_index, 2)
        state.step(connected_sample(), 0.005)
        state.step(connected_sample(right_bumper=True), 0.005)
        self.assertEqual(state.speed_index, 2)

    def test_trigger_depth_direction_and_timestep_scale_the_increment(self):
        state = XboxControlState()
        normal_speed = SPEED_PRESETS[DEFAULT_SPEED_INDEX][1]

        full_positive = state.step(
            connected_sample(right_trigger=1.0), 0.1
        ).command_delta
        full_negative = state.step(
            connected_sample(left_trigger=1.0), 0.1
        ).command_delta
        half_positive = state.step(
            connected_sample(right_trigger=0.525), 0.1
        ).command_delta

        self.assertAlmostEqual(full_positive, normal_speed * 0.1)
        self.assertAlmostEqual(full_negative, -normal_speed * 0.1)
        self.assertAlmostEqual(half_positive, normal_speed * 0.1 * 0.5)
        self.assertEqual(
            state.step(
                connected_sample(right_trigger=0.03), 0.1
            ).command_delta,
            0.0,
        )
        self.assertEqual(
            state.step(
                connected_sample(left_trigger=0.8, right_trigger=0.8),
                0.1,
            ).command_delta,
            0.0,
        )

    def test_disconnected_controller_cannot_move_a_joint(self):
        state = XboxControlState()
        update = state.step(
            GamepadSample(left_trigger=1.0, right_trigger=1.0), 1.0
        )
        self.assertEqual(update.command_delta, 0.0)
        self.assertFalse(state.connected)


class _FakeDisplay:
    def __init__(self):
        self.initialized = False

    def get_init(self):
        return self.initialized

    def init(self):
        self.initialized = True


class _FakeEventQueue:
    def __init__(self):
        self.pumps = 0
        self.clears = 0

    def pump(self):
        self.pumps += 1

    def clear(self):
        self.clears += 1


class _FakePygame:
    def __init__(self):
        self.display = _FakeDisplay()
        self.event = _FakeEventQueue()


class _FakeController:
    def __init__(self):
        self.is_attached = True
        self.closed = False
        self.axes = {0: -16384, 4: 8192, 5: 24576}
        self.buttons = {9: True, 10: False}

    def attached(self):
        return self.is_attached

    def quit(self):
        self.closed = True

    def get_axis(self, axis):
        return self.axes[axis]

    def get_button(self, button):
        return self.buttons[button]


class _FakeControllerModule:
    def __init__(self, device):
        self.initialized = False
        self.devices = [device]

    def get_init(self):
        return self.initialized

    def init(self):
        self.initialized = True

    def get_count(self):
        return len(self.devices)

    def is_controller(self, index):
        return 0 <= index < len(self.devices)

    def Controller(self, index):
        return self.devices[index]

    def name_forindex(self, _index):
        return "Test Xbox Pad"


class PygameBackendTests(unittest.TestCase):
    def test_named_controls_are_normalized_and_disconnect_is_safe(self):
        pygame = _FakePygame()
        device = _FakeController()
        controller_module = _FakeControllerModule(device)
        backend = PygameXboxBackend(
            pygame_module=pygame,
            controller_module=controller_module,
            clock=lambda: 0.0,
        )

        sample = backend.poll()
        self.assertTrue(sample.connected)
        self.assertEqual(sample.name, "Test Xbox Pad")
        self.assertAlmostEqual(sample.left_x, -0.5)
        self.assertAlmostEqual(sample.left_trigger, 0.25)
        self.assertAlmostEqual(sample.right_trigger, 0.75)
        self.assertTrue(sample.left_bumper)
        self.assertFalse(sample.right_bumper)
        self.assertEqual(pygame.event.pumps, 1)
        self.assertEqual(pygame.event.clears, 1)

        device.is_attached = False
        controller_module.devices = []
        sample = backend.poll()
        self.assertFalse(sample.connected)
        self.assertTrue(device.closed)


if __name__ == "__main__":
    unittest.main()
