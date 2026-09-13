"""Optional Xbox-style controller input for the squashSim joint GUI.

The state machine in this module is independent of Pygame so it can be tested
without a controller or a graphical desktop.  Pygame is imported lazily by the
backend; if it is absent, the existing mouse sliders remain fully functional.
"""

from dataclasses import dataclass
import os
import time


# SDL normally stops updating joysticks when another window has focus.  SOFA,
# rather than Pygame, owns the visible window in this process.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")


CONTROL_LABELS = ("J1", "J2", "J3", "J4", "J5", "J6", "RG2")
SPEED_PRESETS = (
    ("Fine", 0.15),
    ("Normal", 0.45),
    ("Fast", 1.20),
)
DEFAULT_SPEED_INDEX = 1
STICK_SELECT_THRESHOLD = 0.65
STICK_RELEASE_THRESHOLD = 0.25
TRIGGER_DEADZONE = 0.05


@dataclass(frozen=True)
class GamepadSample:
    """One normalized snapshot of the controls used by squashSim."""

    connected: bool = False
    name: str = ""
    status: str = "No Xbox-compatible controller detected"
    left_x: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    left_bumper: bool = False
    right_bumper: bool = False


@dataclass(frozen=True)
class ControlUpdate:
    """Result of advancing the controller state by one simulation step."""

    selected_index: int
    speed_index: int
    command_delta: float
    state_changed: bool


class XboxControlState:
    """Turn controller samples into selection, speed, and joint increments."""

    def __init__(self, control_count=len(CONTROL_LABELS)):
        if control_count <= 0:
            raise ValueError("control_count must be positive")
        self.control_count = int(control_count)
        self.selected_index = 0
        self.speed_index = DEFAULT_SPEED_INDEX
        self.connected = False
        self._selector_armed = True
        self._left_bumper_was_down = False
        self._right_bumper_was_down = False

    @property
    def speed_name(self):
        return SPEED_PRESETS[self.speed_index][0]

    @property
    def speed_radians_per_second(self):
        return SPEED_PRESETS[self.speed_index][1]

    def _release_inputs(self):
        self._selector_armed = True
        self._left_bumper_was_down = False
        self._right_bumper_was_down = False

    def step(self, sample, dt):
        """Advance by ``dt`` seconds and return a joint command increment."""
        state_changed = sample.connected != self.connected
        self.connected = bool(sample.connected)
        if not self.connected:
            self._release_inputs()
            return ControlUpdate(
                self.selected_index,
                self.speed_index,
                0.0,
                state_changed,
            )

        left_x = max(-1.0, min(1.0, float(sample.left_x)))
        if self._selector_armed:
            if left_x >= STICK_SELECT_THRESHOLD:
                self.selected_index = (
                    self.selected_index + 1
                ) % self.control_count
                self._selector_armed = False
                state_changed = True
            elif left_x <= -STICK_SELECT_THRESHOLD:
                self.selected_index = (
                    self.selected_index - 1
                ) % self.control_count
                self._selector_armed = False
                state_changed = True
        elif abs(left_x) <= STICK_RELEASE_THRESHOLD:
            self._selector_armed = True

        left_bumper = bool(sample.left_bumper)
        right_bumper = bool(sample.right_bumper)
        # If both bumpers are pressed together, leave the speed unchanged.
        if not (left_bumper and right_bumper):
            if left_bumper and not self._left_bumper_was_down:
                new_index = max(0, self.speed_index - 1)
                state_changed = state_changed or new_index != self.speed_index
                self.speed_index = new_index
            elif right_bumper and not self._right_bumper_was_down:
                new_index = min(len(SPEED_PRESETS) - 1, self.speed_index + 1)
                state_changed = state_changed or new_index != self.speed_index
                self.speed_index = new_index
        self._left_bumper_was_down = left_bumper
        self._right_bumper_was_down = right_bumper

        left_trigger = max(0.0, min(1.0, float(sample.left_trigger)))
        right_trigger = max(0.0, min(1.0, float(sample.right_trigger)))
        signed_trigger = right_trigger - left_trigger
        magnitude = abs(signed_trigger)
        if magnitude <= TRIGGER_DEADZONE or float(dt) <= 0.0:
            command_delta = 0.0
        else:
            # Remove the dead zone while retaining the full 0..1 output range.
            magnitude = (magnitude - TRIGGER_DEADZONE) / (
                1.0 - TRIGGER_DEADZONE
            )
            command_delta = (
                (1.0 if signed_trigger > 0.0 else -1.0)
                * magnitude
                * self.speed_radians_per_second
                * float(dt)
            )

        return ControlUpdate(
            self.selected_index,
            self.speed_index,
            command_delta,
            state_changed,
        )


class PygameXboxBackend:
    """Poll the first SDL game controller, with hot-plug reconnection."""

    # SDL_GameControllerAxis/Button enum values.  Pygame normally exports the
    # named constants, but these documented SDL values keep older Pygame 2
    # builds usable as well.
    _CONSTANT_DEFAULTS = {
        "CONTROLLER_AXIS_LEFTX": 0,
        "CONTROLLER_AXIS_TRIGGERLEFT": 4,
        "CONTROLLER_AXIS_TRIGGERRIGHT": 5,
        "CONTROLLER_BUTTON_LEFTSHOULDER": 9,
        "CONTROLLER_BUTTON_RIGHTSHOULDER": 10,
    }

    def __init__(
        self,
        reconnect_interval=1.0,
        pygame_module=None,
        controller_module=None,
        clock=None,
    ):
        self._clock = clock or time.monotonic
        self._reconnect_interval = max(0.0, float(reconnect_interval))
        self._next_reconnect = 0.0
        self._controller = None
        self._controller_name = ""
        self._available = False
        self._status = "Pygame controller input is unavailable"

        if (pygame_module is None) != (controller_module is None):
            raise ValueError(
                "pygame_module and controller_module must be supplied together"
            )

        if pygame_module is None:
            try:
                import pygame as pygame_module
                from pygame._sdl2 import controller as controller_module
            except (ImportError, ModuleNotFoundError):
                self._status = (
                    "Pygame is not installed; mouse sliders remain active"
                )
                return

        self._pygame = pygame_module
        self._controller_api = controller_module
        try:
            if not self._pygame.display.get_init():
                self._pygame.display.init()
            if not self._controller_api.get_init():
                self._controller_api.init()
            self._constants = {
                name: self._find_constant(name, fallback)
                for name, fallback in self._CONSTANT_DEFAULTS.items()
            }
            self._available = True
            self._status = "No Xbox-compatible controller detected"
            self._try_connect(force=True)
        except Exception as error:
            self._status = f"Pygame controller initialization failed: {error}"

    def _find_constant(self, name, fallback):
        for module in (self._controller_api, self._pygame):
            if hasattr(module, name):
                return getattr(module, name)
        return fallback

    def _close_controller(self):
        if self._controller is not None:
            try:
                self._controller.quit()
            except Exception:
                pass
        self._controller = None
        self._controller_name = ""

    def _pump_events(self):
        # Axis values may remain stale unless SDL's event queue is serviced.
        self._pygame.event.pump()
        if hasattr(self._pygame.event, "clear"):
            self._pygame.event.clear()

    def _try_connect(self, force=False):
        now = self._clock()
        if not force and now < self._next_reconnect:
            return
        self._next_reconnect = now + self._reconnect_interval
        try:
            for index in range(int(self._controller_api.get_count())):
                if not self._controller_api.is_controller(index):
                    continue
                candidate = self._controller_api.Controller(index)
                if candidate.attached():
                    self._controller = candidate
                    self._controller_name = (
                        self._controller_api.name_forindex(index)
                        or "Xbox-compatible controller"
                    )
                    self._status = f"Connected: {self._controller_name}"
                    return
                candidate.quit()
            self._status = "No Xbox-compatible controller detected"
        except Exception as error:
            self._close_controller()
            self._status = f"Controller discovery failed: {error}"

    @staticmethod
    def _normalise_stick(raw_value):
        value = float(raw_value)
        denominator = 32768.0 if value < 0.0 else 32767.0
        return max(-1.0, min(1.0, value / denominator))

    @staticmethod
    def _normalise_trigger(raw_value):
        return max(0.0, min(1.0, float(raw_value) / 32768.0))

    def poll(self):
        if not self._available:
            return GamepadSample(status=self._status)

        try:
            self._pump_events()
        except Exception as error:
            self._status = f"Pygame event polling failed: {error}"
            return GamepadSample(status=self._status)

        if self._controller is not None and not self._controller.attached():
            self._close_controller()
            self._next_reconnect = 0.0
            self._status = "Controller disconnected; mouse sliders remain active"

        if self._controller is None:
            self._try_connect()
        if self._controller is None:
            return GamepadSample(status=self._status)

        try:
            return GamepadSample(
                connected=True,
                name=self._controller_name,
                status=f"Connected: {self._controller_name}",
                left_x=self._normalise_stick(
                    self._controller.get_axis(
                        self._constants["CONTROLLER_AXIS_LEFTX"]
                    )
                ),
                left_trigger=self._normalise_trigger(
                    self._controller.get_axis(
                        self._constants["CONTROLLER_AXIS_TRIGGERLEFT"]
                    )
                ),
                right_trigger=self._normalise_trigger(
                    self._controller.get_axis(
                        self._constants["CONTROLLER_AXIS_TRIGGERRIGHT"]
                    )
                ),
                left_bumper=bool(
                    self._controller.get_button(
                        self._constants["CONTROLLER_BUTTON_LEFTSHOULDER"]
                    )
                ),
                right_bumper=bool(
                    self._controller.get_button(
                        self._constants["CONTROLLER_BUTTON_RIGHTSHOULDER"]
                    )
                ),
            )
        except Exception as error:
            self._close_controller()
            self._next_reconnect = 0.0
            self._status = f"Controller read failed: {error}"
            return GamepadSample(status=self._status)
