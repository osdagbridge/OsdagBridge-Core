"""Command implementations for the OsdagBridge CLI."""

from copy import deepcopy
from pprint import pformat
from pathlib import Path
from typing import Any

import yaml

from osdagbridge_core.core.bridge_types.plate_girder.defaults import (
    solve_extend_basic_input_dict,
)
from osdagbridge_core.core.bridge_types.plate_girder.plategirderbridge import (
    PlateGirderBridge,
)
from osdagbridge_core.core.utils.osi_validator import validate_osi_inputs


def load_and_prepare_osi_input(path: str | Path) -> dict[str, Any]:
    """Load, validate, and prepare an OsdagBridge ``.osi`` input dictionary.

    The YAML document must contain the flat dictionary used by the Core
    backend. The dictionary is validated with the existing OSI validator and
    then mutated in place by ``solve_extend_basic_input_dict`` to add defaults
    and derived bridge-layout values. The returned dictionary is prepared for
    backend setup but has not yet been normalized by ``PlateGirderBridge``.
    """
    input_path = Path(path)
    with input_path.open("r", encoding="utf-8") as input_file:
        data = yaml.safe_load(input_file)

    if not isinstance(data, dict):
        raise ValueError("File does not contain a valid input dictionary.")

    validation = validate_osi_inputs(data)
    if not validation.is_valid:
        raise ValueError(
            "The OSI file contains invalid values:\n"
            f"{validation.error_text()}"
        )

    solve_extend_basic_input_dict(data)
    return data


def create_backend(input_dict: dict[str, Any]) -> PlateGirderBridge:
    """Create a backend from an already prepared input dictionary.

    ``input_dict`` is expected to have already been loaded, validated, and
    extended by :func:`load_and_prepare_osi_input`. This function only invokes the
    backend's public ``set_input`` method, so normalization remains owned by
    the Core implementation. It does not execute design calculations.
    """
    backend = PlateGirderBridge()
    backend.set_input(input_dict)
    return backend


def get_normalized_input(input_dict: dict[str, Any]) -> dict[str, Any]:
    """Return normalized inputs immediately after backend input setup.

    The supplied dictionary must already be prepared for backend use. It is
    passed through ``PlateGirderBridge.set_input()``, and a copy of the
    resulting ``backend.input_dict`` is captured before ``design()`` runs.
    This function never executes design calculations, and the returned copy is
    independent of the backend-owned dictionary.
    """
    backend = create_backend(input_dict)
    return deepcopy(backend.input_dict)


def run_design(input_dict: dict[str, Any]) -> dict[str, Any]:
    """Run bridge design from an already prepared input dictionary.

    The dictionary is expected to come from :func:`load_and_prepare_osi_input`; this
    function does not load YAML, validate OSI data, or repeat input
    preparation. It configures the backend, executes its existing
    ``design()`` pipeline, and returns the dictionary produced by
    ``get_results()``.
    """
    backend = create_backend(input_dict)
    backend.design()
    return backend.get_results()


def design_cmd(path: str | Path) -> dict:
    """Load an ``.osi`` file and run the existing bridge design pipeline."""
    return run_design(load_and_prepare_osi_input(path))


def get_input_cmd(path: str | Path) -> dict[str, Any]:
    """Load an ``.osi`` file and return its normalized backend inputs.

    Design is intentionally not executed. The normalized snapshot is captured
    immediately after ``set_input()`` so later design-time input mutations
    cannot change what this command displays.
    """
    return get_normalized_input(load_and_prepare_osi_input(path))


def print_input(input_dict: dict[str, Any]) -> None:
    """Print a normalized input dictionary without changing it."""
    print("=" * 100)
    print("--Normalized Input--")
    print(pformat(input_dict, sort_dicts=False))
    print("=" * 100)


def print_results(results: dict) -> None:
    """Print the complete results dictionary in the original CLI format."""
    print("=" * 100)
    print("--Design Results--\n")
    print(pformat(results, sort_dicts=False))
    print("=" * 100)
