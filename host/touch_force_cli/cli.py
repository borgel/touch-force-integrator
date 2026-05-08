"""Command-line interface for the touchforce.v1 protocol.

Usage:

    uv run python -m touch_force_cli get-uptime --port /dev/cu.usbmodem...

The single subcommand opens the serial port, sends a GetUptime
request, and prints the result in milliseconds.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import serial  # pyserial

from touch_force_cli.link import Link, ProtocolError, RemoteError


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="touch_force_cli")
    p.add_argument(
        "--port",
        required=True,
        help="serial device path, e.g. /dev/cu.usbmodemXXXX",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="baud rate (USB CDC ignores this but pyserial requires a value)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="per-read timeout in seconds (default: 2.0)",
    )
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("get-uptime", help="ask the MCU for its uptime in ms")
    sub.add_parser("get-telemetry", help="ask the MCU for streaming counters")
    sub.add_parser("stream-touches", help="print streamed touch events to stdout (Ctrl-C exits)")

    # Haptic-area mode.
    p_set_mode = sub.add_parser("set-haptic-mode", help="enable or disable haptic-area mode")
    p_set_mode.add_argument("state", choices=["on", "off"], help="on or off")

    sub.add_parser("get-haptic-mode", help="read the current haptic-area mode")

    # Add a new haptic area.
    p_add = sub.add_parser("add-area", help="create a new haptic area")
    p_add.add_argument("--x0", type=int, required=True, help="left edge")
    p_add.add_argument("--y0", type=int, required=True, help="top edge")
    p_add.add_argument("--x1", type=int, required=True, help="right edge")
    p_add.add_argument("--y1", type=int, required=True, help="bottom edge")
    p_add.add_argument("--tag", default="", help="optional label string")
    p_add.add_argument("--kind", choices=["fire", "block"], default="fire",
                       help="area kind (default: fire)")
    p_add.add_argument("--disabled", action="store_true",
                       help="create the area in a disabled state")
    p_add.add_argument("--haptic", choices=["default"], default="default",
                       help="haptic effect (default: default)")

    # Update an existing haptic area (read-modify-write).
    p_upd = sub.add_parser("update-area", help="update fields of an existing haptic area")
    p_upd.add_argument("--id", type=int, required=True, dest="area_id", help="area id")
    p_upd.add_argument("--x0", type=int)
    p_upd.add_argument("--y0", type=int)
    p_upd.add_argument("--x1", type=int)
    p_upd.add_argument("--y1", type=int)
    p_upd.add_argument("--tag")
    p_upd.add_argument("--kind", choices=["fire", "block"])
    p_upd.add_argument("--haptic", choices=["default"])
    upd_en = p_upd.add_mutually_exclusive_group()
    upd_en.add_argument("--enabled", action="store_true", default=None,
                        help="enable the area")
    upd_en.add_argument("--disabled", action="store_true", default=None,
                        help="disable the area")

    # Get one area.
    p_get = sub.add_parser("get-area", help="fetch one haptic area by id")
    p_get.add_argument("--id", type=int, required=True, dest="area_id", help="area id")

    # List all areas.
    sub.add_parser("list-areas", help="list all haptic area ids")

    # Delete one area.
    p_del = sub.add_parser("delete-area", help="delete a haptic area by id")
    p_del.add_argument("--id", type=int, required=True, dest="area_id", help="area id")

    # Delete all areas.
    sub.add_parser("delete-all-areas", help="delete every haptic area")

    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        lnk = Link(ser)
        if args.command == "get-uptime":
            try:
                ms = lnk.get_uptime()
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print(f"uptime: {ms} ms")
            return 0
        if args.command == "get-telemetry":
            try:
                sent, fails = lnk.get_telemetry()
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            total = sent + fails
            rate = (fails / total * 100.0) if total else 0.0
            print(f"events sent: {sent}")
            print(f"tx fails:    {fails}  ({rate:.3f}%)")
            return 0
        if args.command == "stream-touches":
            # Streaming consumers want to wait indefinitely for the next
            # event; the 2s default timeout would make read_frame() return
            # None whenever the user pauses touching, terminating the
            # generator and looking like the program crashed.
            ser.timeout = None
            # Send SetTouchStreaming(true) regardless of state — guarantees
            # streaming is on even after a prior session left it disabled
            # via Ctrl-C cleanup.
            lnk.set_touch_streaming(enabled=True)
            try:
                for event in lnk.read_events():
                    if event.WhichOneof("payload") != "touch_frame":
                        continue
                    tfe = event.touch_frame
                    fingers_repr = " ".join(
                        f"id={f.id} (x={f.x},y={f.y})" for f in tfe.fingers
                    )
                    # Use a high-resolution monotonic clock for the host-side
                    # timestamp; the MCU doesn't stamp events in v2.
                    ts = time.monotonic()
                    print(
                        f"{ts:.3f}  {tfe.live_touches} live, "
                        f"{tfe.tips_touched_down} down: {fingers_repr}",
                        flush=True,
                    )
            except KeyboardInterrupt:
                # Best-effort polite shutdown so the MCU stops streaming
                # to a host that's no longer there. Failures are fine —
                # if the bytes don't go through, the host is gone anyway.
                try:
                    lnk.set_touch_streaming(enabled=False)
                except Exception:
                    pass
            return 0
        if args.command == "set-haptic-mode":
            try:
                enabled = args.state == "on"
                now = lnk.set_haptic_area_mode(enabled=enabled)
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print("OK")
            return 0
        if args.command == "get-haptic-mode":
            try:
                enabled = lnk.get_haptic_area_mode()
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print("on" if enabled else "off")
            return 0
        if args.command == "add-area":
            try:
                result = lnk.add_haptic_area(
                    x0=args.x0, y0=args.y0, x1=args.x1, y1=args.y1,
                    tag=args.tag, kind=args.kind,
                    enabled=not args.disabled,
                    haptic=args.haptic,
                )
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print(json.dumps(result, indent=2))
            return 0
        if args.command == "update-area":
            fields = {}
            for key in ("x0", "y0", "x1", "y1", "tag", "kind", "haptic"):
                val = getattr(args, key, None)
                if val is not None:
                    fields[key] = val
            if args.enabled:
                fields["enabled"] = True
            elif args.disabled:
                fields["enabled"] = False
            try:
                result = lnk.update_haptic_area(args.area_id, **fields)
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print(json.dumps(result, indent=2))
            return 0
        if args.command == "get-area":
            try:
                result = lnk.get_haptic_area(args.area_id)
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print(json.dumps(result, indent=2))
            return 0
        if args.command == "list-areas":
            try:
                ids = lnk.list_haptic_areas()
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print(json.dumps(ids, indent=2))
            return 0
        if args.command == "delete-area":
            try:
                lnk.delete_haptic_area(args.area_id)
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print("OK")
            return 0
        if args.command == "delete-all-areas":
            try:
                lnk.delete_all_haptic_areas()
            except RemoteError as e:
                print(f"MCU returned error: {e}", file=sys.stderr)
                return 2
            except ProtocolError as e:
                print(f"protocol error: {e}", file=sys.stderr)
                return 3
            print("OK")
            return 0
        # argparse with required=True ensures we never reach here.
        raise AssertionError(f"unhandled command: {args.command!r}")


if __name__ == "__main__":
    sys.exit(main())
