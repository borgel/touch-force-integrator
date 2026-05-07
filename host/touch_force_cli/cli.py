"""Command-line interface for the touchforce.v1 protocol.

Usage:

    uv run python -m touch_force_cli get-uptime --port /dev/cu.usbmodem...

The single subcommand opens the serial port, sends a GetUptime
request, and prints the result in milliseconds.
"""

from __future__ import annotations

import argparse
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
            # SetTouchStreaming(true) is technically redundant — the firmware
            # boots with streaming on — but sending it makes the command's
            # intent self-evident in protocol-trace logs.
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
        # argparse with required=True ensures we never reach here.
        raise AssertionError(f"unhandled command: {args.command!r}")


if __name__ == "__main__":
    sys.exit(main())
