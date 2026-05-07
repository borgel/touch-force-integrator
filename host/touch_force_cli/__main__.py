"""Allow ``python -m touch_force_cli ...`` to dispatch to the CLI."""

from touch_force_cli.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
