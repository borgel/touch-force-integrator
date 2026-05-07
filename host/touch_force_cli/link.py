"""Wire-level helpers for the touchforce.v1 serial protocol.

This module handles three concerns:
  * COBS encode / decode (cobs_encode, cobs_decode)
  * Reading and writing whole COBS frames over a byte transport
    (read_frame, write_frame)
  * A Link wrapper that binds the framing layer to nanopb-compatible
    Request/Response messages and adds request_id correlation
"""

from __future__ import annotations

import itertools
from typing import Callable, Optional, Protocol

from touch_force_cli import touch_force_pb2 as _pb


class FramingError(ValueError):
    """Raised when a frame on the wire violates COBS encoding rules."""


# ---------------------------------------------------------------------------
# COBS primitives
#
# Reference: Cheshire & Baker 1999 ("Consistent Overhead Byte Stuffing").
# The encoder replaces every 0x00 with the distance to the next 0x00 (or
# end of frame), prefixed with the distance to the first 0x00 (or end).
# ---------------------------------------------------------------------------

def cobs_encode(data: bytes) -> bytes:
    """COBS-encode ``data``. The output never contains 0x00."""
    out = bytearray()
    # Reserve a slot for the leading overhead byte; we'll patch it
    # at the end of each segment.
    out.append(0)
    code_index = 0
    code = 1  # number of bytes in the current segment + 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)  # placeholder for next segment's code
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                # Segment is full; close it and open a new one.
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS-decode ``data``. Raises ``FramingError`` on malformed input."""
    if not data:
        raise FramingError("empty COBS frame")
    if 0 in data:
        raise FramingError("COBS payload must not contain 0x00")

    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise FramingError("zero code byte inside COBS frame")
        end = i + code
        if end > n:
            raise FramingError("COBS code byte points past end of frame")
        out.extend(data[i + 1 : end])
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# Frame I/O
# ---------------------------------------------------------------------------

class _ByteTransport(Protocol):
    """Minimal byte-stream interface (matches pyserial.Serial)."""

    def read(self, n: int) -> bytes: ...
    def write(self, data: bytes) -> int: ...


def write_frame(transport: _ByteTransport, payload: bytes) -> None:
    """COBS-encode ``payload`` and write it followed by 0x00."""
    encoded = cobs_encode(payload)
    transport.write(encoded + b"\x00")


def read_frame(transport: _ByteTransport) -> Optional[bytes]:
    """Read bytes from ``transport`` until 0x00, then COBS-decode.

    Returns ``None`` at EOF (used by tests with a finite buffer; on
    a real pyserial transport with no timeout this never happens).
    Leading 0x00 bytes (stray delimiters from a previous corrupted
    frame, or bytes the MCU emitted before the host opened the port)
    are silently dropped.
    """
    buf = bytearray()
    while True:
        chunk = transport.read(1)
        if not chunk:
            if buf:
                # Hit EOF mid-frame; treat as no-more-frames.
                return None
            return None
        b = chunk[0]
        if b == 0:
            if not buf:
                # Leading delimiter, ignore and keep reading.
                continue
            return cobs_decode(bytes(buf))
        buf.append(b)


# ---------------------------------------------------------------------------
# Link: request/response correlation
# ---------------------------------------------------------------------------


class ProtocolError(RuntimeError):
    """Raised when a wire frame violates the protocol contract."""


class RemoteError(RuntimeError):
    """Raised when the MCU returns ErrorResponse instead of the expected reply."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(f"remote error code={code}: {message}")
        self.code = code
        self.message = message


_default_id_counter = itertools.count(1)


class Link:
    """High-level request/response wrapper over a byte transport."""

    def __init__(
        self,
        transport: _ByteTransport,
        *,
        _next_id: Optional[Callable[[], int]] = None,
    ) -> None:
        self._t = transport
        self._next_id = _next_id or (lambda: next(_default_id_counter))

    def get_uptime(self) -> int:
        """Send GetUptimeRequest, return uptime_ms from the response."""
        rid = self._next_id()
        outgoing = _pb.Frame()
        outgoing.request.request_id = rid
        outgoing.request.get_uptime.SetInParent()
        write_frame(self._t, outgoing.SerializeToString())

        resp = self._read_response(rid)
        which = resp.WhichOneof("payload")
        if which == "error":
            raise RemoteError(code=resp.error.code, message=resp.error.message)
        if which == "get_uptime":
            return resp.get_uptime.uptime_ms
        raise ProtocolError(f"unexpected response payload: {which!r}")

    def _read_response(self, expected_rid: int) -> "_pb.Response":
        """Read frames until a Response with matching request_id arrives.

        Events encountered along the way are silently dropped (this method
        is for synchronous request/response calls; see read_events() for
        the async-event use case).
        """
        while True:
            raw = read_frame(self._t)
            if raw is None:
                raise ProtocolError("transport closed before response arrived")
            frame = _pb.Frame()
            frame.ParseFromString(raw)
            kind = frame.WhichOneof("kind")
            if kind != "response":
                # Drop Events (and any unexpected Requests) while waiting
                # for our reply.
                continue
            resp = frame.response
            if resp.request_id != expected_rid:
                raise ProtocolError(
                    f"request_id mismatch: sent {expected_rid}, "
                    f"got {resp.request_id}"
                )
            return resp

    def set_touch_streaming(self, *, enabled: bool) -> None:
        """Enable or disable touch streaming on the MCU. Fire-and-forget.

        No Response is sent by the firmware; this method writes the Frame
        and returns. Reliability comes from USB CDC; the host confirms
        the change took effect by observing the event stream (events stop
        when enabled=False).
        """
        rid = self._next_id()
        outgoing = _pb.Frame()
        outgoing.request.request_id = rid
        outgoing.request.set_touch_streaming.enabled = enabled
        write_frame(self._t, outgoing.SerializeToString())

    def get_telemetry(self) -> tuple[int, int]:
        """Send GetTelemetryRequest, return (events_sent, tx_fails)."""
        rid = self._next_id()
        outgoing = _pb.Frame()
        outgoing.request.request_id = rid
        outgoing.request.get_telemetry.SetInParent()
        write_frame(self._t, outgoing.SerializeToString())

        resp = self._read_response(rid)
        which = resp.WhichOneof("payload")
        if which == "error":
            raise RemoteError(code=resp.error.code, message=resp.error.message)
        if which == "get_telemetry":
            return (
                resp.get_telemetry.streaming_events_sent,
                resp.get_telemetry.streaming_tx_fails,
            )
        raise ProtocolError(f"unexpected response payload: {which!r}")

    def read_events(self):
        """Generator yielding Event messages forever (or until EOF).

        Responses encountered with no pending request are discarded —
        the CLI use case for `read_events` is one-directional event
        consumption (`stream-touches` subcommand). For interleaved
        request/response + event handling, the webapp uses a
        request_id-keyed pending-promise map instead.
        """
        while True:
            raw = read_frame(self._t)
            if raw is None:
                return
            frame = _pb.Frame()
            frame.ParseFromString(raw)
            kind = frame.WhichOneof("kind")
            if kind == "event":
                yield frame.event
            # else: discard (Response with no pending request, or unexpected)
