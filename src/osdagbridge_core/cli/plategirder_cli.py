"""Plate Girder CLI operations."""

from copy import deepcopy
from typing import Any

from osdagbridge_core.core.bridge_types.plate_girder.plategirderbridge import (
    PlateGirderBridge,
)
from osdagbridge_core.core.bridge_types.plate_girder.defaults import (
    solve_extend_basic_input_dict,
)


class PlateGirderCLI:
    """Expose CLI operations backed by ``PlateGirderBridge``."""

    def __init__(self, input_dict: dict[str, Any]) -> None:
        solve_extend_basic_input_dict(input_dict)
        self.backend = PlateGirderBridge()
        self.backend.set_input(input_dict)

    def get_input(self) -> dict[str, Any]:
        """Return normalized inputs without running design."""
        return deepcopy(self.backend.input_dict)

    def design(self) -> dict[str, Any]:
        """Run design and return the backend's complete result dictionary."""
        self.backend.design()
        return self.backend.get_results()
