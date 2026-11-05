"""Command implementations for the OsdagBridge CLI."""

from pathlib import Path
from pprint import pformat
from typing import Any

import yaml

from osdagbridge_core.cli.plategirder_cli import PlateGirderCLI
from osdagbridge_core.core.utils.common import (
    KEY_MODULE,
    KEY_MODULE_PLATE_GIRDER,
)
from osdagbridge_core.core.utils.osi_validator import validate_osi_inputs


def load_and_validate_osi_input(path: str | Path) -> dict[str, Any]:
    """Load and validate an OsdagBridge ``.osi`` input dictionary."""
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

    return data


def select_cli_class(input_dict: dict[str, Any]) -> type[PlateGirderCLI]:
    """Select the CLI class based on the input module."""
    module = input_dict.get(KEY_MODULE)
    if module is None:
        raise ValueError(f"Input is missing the required '{KEY_MODULE}' field.")
    if module == KEY_MODULE_PLATE_GIRDER:
        return PlateGirderCLI
    raise ValueError(f"Unsupported bridge module '{module}'.")


def design_cmd(path: str | Path) -> dict:
    """Load an ``.osi`` file and run the existing bridge design pipeline."""
    input_dict = load_and_validate_osi_input(path)
    cli_class = select_cli_class(input_dict)
    return cli_class(input_dict).design()


def get_input_cmd(path: str | Path) -> dict[str, Any]:
    """Load an ``.osi`` file and return its normalized backend inputs."""
    input_dict = load_and_validate_osi_input(path)
    cli_class = select_cli_class(input_dict)
    return cli_class(input_dict).get_input()


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
