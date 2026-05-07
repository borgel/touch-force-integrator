"""Tests for touch_force_cli.link framing primitives."""

from __future__ import annotations

import io

import pytest

from touch_force_cli import link


# ---------------------------------------------------------------------------
# COBS round-trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "raw",
    [
        b"",
        b"\x01",
        b"hello",
        bytes(range(1, 32)),
        # COBS's edge case: a 254-byte run of non-zeros forces an
        # extra overhead byte.
        bytes([0xAA] * 254),
        # A frame containing many zeros, exercising the segmenting code.
        b"\x00\x00\x00",
        b"a\x00b\x00c",
    ],
)
def test_cobs_round_trip(raw: bytes) -> None:
    encoded = link.cobs_encode(raw)
    assert b"\x00" not in encoded, "encoded payload must not contain 0x00"
    assert link.cobs_decode(encoded) == raw


def test_cobs_decode_rejects_zero_in_payload() -> None:
    # 0x00 inside the COBS payload is a framing error.
    with pytest.raises(link.FramingError):
        link.cobs_decode(b"\x02a\x00b")


# ---------------------------------------------------------------------------
# Frame I/O
# ---------------------------------------------------------------------------

class _LoopTransport:
    """Byte-stream transport with a paired in/out buffer for tests."""

    def __init__(self, incoming: bytes = b"") -> None:
        self._incoming = io.BytesIO(incoming)
        self.outgoing = bytearray()

    def read(self, n: int) -> bytes:
        return self._incoming.read(n)

    def write(self, data: bytes) -> int:
        self.outgoing.extend(data)
        return len(data)


def test_write_frame_appends_zero_delimiter() -> None:
    t = _LoopTransport()
    link.write_frame(t, b"hi")
    # COBS-encoded 'hi' is b'\x03hi'; followed by 0x00.
    assert bytes(t.outgoing) == b"\x03hi\x00"


def test_read_frame_drops_leading_zeros_and_returns_payload() -> None:
    # Two consecutive 0x00 bytes on the wire (a stray delimiter
    # followed by a real frame) should be tolerated.
    wire = b"\x00\x03hi\x00"
    t = _LoopTransport(wire)
    assert link.read_frame(t) == b"hi"


def test_read_frame_returns_none_at_eof() -> None:
    t = _LoopTransport(b"")
    assert link.read_frame(t) is None


# ---------------------------------------------------------------------------
# Link: request/response correlation
# ---------------------------------------------------------------------------

from touch_force_cli import touch_force_pb2 as pb


def _encoded_response(*, request_id: int, uptime_ms: int) -> bytes:
    """Helper: return wire bytes for a GetUptime response framed."""
    resp = pb.Response()
    resp.request_id = request_id
    resp.get_uptime.uptime_ms = uptime_ms
    return link.cobs_encode(resp.SerializeToString()) + b"\x00"


def test_link_get_uptime_round_trip() -> None:
    # MCU reply is queued up in the loop transport before we call.
    t = _LoopTransport(_encoded_response(request_id=42, uptime_ms=12_345))

    lnk = link.Link(t, _next_id=lambda: 42)
    uptime_ms = lnk.get_uptime()

    assert uptime_ms == 12_345

    # Verify the host sent a properly-framed Request with our
    # request_id and the get_uptime payload set.
    sent = bytes(t.outgoing)
    assert sent.endswith(b"\x00")
    decoded = link.cobs_decode(sent[:-1])
    req = pb.Request()
    req.ParseFromString(decoded)
    assert req.request_id == 42
    assert req.WhichOneof("payload") == "get_uptime"


def test_link_raises_on_request_id_mismatch() -> None:
    t = _LoopTransport(_encoded_response(request_id=999, uptime_ms=1))

    lnk = link.Link(t, _next_id=lambda: 42)

    with pytest.raises(link.ProtocolError, match="request_id"):
        lnk.get_uptime()


def test_link_propagates_error_response() -> None:
    err = pb.Response()
    err.request_id = 7
    err.error.code = 1
    err.error.message = "unknown command"
    wire = link.cobs_encode(err.SerializeToString()) + b"\x00"
    t = _LoopTransport(wire)

    lnk = link.Link(t, _next_id=lambda: 7)

    with pytest.raises(link.RemoteError) as exc_info:
        lnk.get_uptime()
    assert exc_info.value.code == 1
    assert "unknown command" in str(exc_info.value)
