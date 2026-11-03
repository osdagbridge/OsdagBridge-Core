"""Click entrypoint for the OsdagBridge CLI."""

from pathlib import Path

import click
import yaml

from .commands import design_cmd, get_input_cmd, print_input, print_results


@click.group()
def cli() -> None:
    """Run OsdagBridge command-line operations."""


@cli.command()
@click.option(
    "-i",
    "--input",
    "input_path",
    required=True,
    type=click.Path(
        exists=True,
        dir_okay=False,
        readable=True,
        path_type=Path,
    ),
    help="Path to the .osi input file.",
)
def design(input_path: Path) -> None:
    """Run a bridge design from an .osi input file."""
    try:
        results = design_cmd(input_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise click.ClickException(str(exc)) from exc

    print_results(results)


@cli.command("get-input")
@click.option(
    "-i",
    "--input",
    "input_path",
    required=True,
    type=click.Path(
        exists=True,
        dir_okay=False,
        readable=True,
        path_type=Path,
    ),
    help="Path to the .osi input file.",
)
def get_input(input_path: Path) -> None:
    """Display normalized inputs without running bridge design."""
    try:
        normalized_input = get_input_cmd(input_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise click.ClickException(str(exc)) from exc

    print_input(normalized_input)


main = cli


if __name__ == "__main__":
    cli()
