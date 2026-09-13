"""Launch-time selection of the object placed beside the robot."""

import os

from deformable_ball import add_deformable_ball
from flexible_cable import add_flexible_cable


OBJECT_ENVIRONMENT_VARIABLE = "SQUASHSIM_OBJECT"
DEFAULT_OBJECT = "cable"
OBJECT_FACTORIES = {
    "ball": add_deformable_ball,
    "cable": add_flexible_cable,
}


def selected_object_name(environment=None):
    """Return the validated ball/cable/none launch selection."""
    if environment is None:
        environment = os.environ
    selection = environment.get(
        OBJECT_ENVIRONMENT_VARIABLE, DEFAULT_OBJECT
    ).strip().lower()
    if selection not in {*OBJECT_FACTORIES, "none"}:
        choices = ", ".join((*OBJECT_FACTORIES, "none"))
        raise ValueError(
            f"Invalid {OBJECT_ENVIRONMENT_VARIABLE}={selection!r}; "
            f"choose one of: {choices}"
        )
    return selection


def add_demo_object(root_node, selection=None):
    """Add exactly one selected object, or leave a robot-only scene."""
    if selection is None:
        selection = selected_object_name()
    else:
        selection = selected_object_name(
            {OBJECT_ENVIRONMENT_VARIABLE: str(selection)}
        )

    print(f"[squashSim] selected demo object: {selection}")
    if selection == "none":
        return None
    return OBJECT_FACTORIES[selection](root_node)
