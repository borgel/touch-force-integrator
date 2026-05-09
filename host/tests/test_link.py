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


def test_cobs_decode_rejects_empty_input() -> None:
    with pytest.raises(link.FramingError, match="empty"):
        link.cobs_decode(b"")


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
    """Helper: return wire bytes for a GetUptime response, Frame-wrapped."""
    frame = pb.Frame()
    frame.response.request_id = request_id
    frame.response.get_uptime.uptime_ms = uptime_ms
    return link.cobs_encode(frame.SerializeToString()) + b"\x00"


def test_link_get_uptime_round_trip() -> None:
    # MCU reply is queued up in the loop transport before we call.
    t = _LoopTransport(_encoded_response(request_id=42, uptime_ms=12_345))

    lnk = link.Link(t, _next_id=lambda: 42)
    uptime_ms = lnk.get_uptime()

    assert uptime_ms == 12_345

    # Verify the host sent a Frame{request} with our request_id
    # and the get_uptime payload set.
    sent = bytes(t.outgoing)
    assert sent.endswith(b"\x00")
    decoded = link.cobs_decode(sent[:-1])
    frame = pb.Frame()
    frame.ParseFromString(decoded)
    assert frame.WhichOneof("kind") == "request"
    assert frame.request.request_id == 42
    assert frame.request.WhichOneof("payload") == "get_uptime"


def test_link_raises_on_request_id_mismatch() -> None:
    t = _LoopTransport(_encoded_response(request_id=999, uptime_ms=1))

    lnk = link.Link(t, _next_id=lambda: 42)

    with pytest.raises(link.ProtocolError, match="request_id"):
        lnk.get_uptime()


def test_link_propagates_error_response() -> None:
    frame = pb.Frame()
    frame.response.request_id = 7
    frame.response.error.code = 1
    frame.response.error.message = "unknown command"
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"
    t = _LoopTransport(wire)

    lnk = link.Link(t, _next_id=lambda: 7)

    with pytest.raises(link.RemoteError) as exc_info:
        lnk.get_uptime()
    assert exc_info.value.code == 1
    assert "unknown command" in str(exc_info.value)


# ---------------------------------------------------------------------------
# Link.set_touch_streaming (fire-and-forget)
# ---------------------------------------------------------------------------

def test_link_set_touch_streaming_enabled_writes_frame() -> None:
    t = _LoopTransport()  # no incoming bytes; method must NOT read.
    lnk = link.Link(t, _next_id=lambda: 99)

    lnk.set_touch_streaming(enabled=True)

    sent = bytes(t.outgoing)
    assert sent.endswith(b"\x00")
    decoded = link.cobs_decode(sent[:-1])
    frame = pb.Frame()
    frame.ParseFromString(decoded)
    assert frame.WhichOneof("kind") == "request"
    assert frame.request.request_id == 99
    assert frame.request.WhichOneof("payload") == "set_touch_streaming"
    assert frame.request.set_touch_streaming.enabled is True


def test_link_set_touch_streaming_disabled_writes_frame() -> None:
    t = _LoopTransport()
    lnk = link.Link(t, _next_id=lambda: 100)

    lnk.set_touch_streaming(enabled=False)

    sent = bytes(t.outgoing)
    decoded = link.cobs_decode(sent[:-1])
    frame = pb.Frame()
    frame.ParseFromString(decoded)
    # proto3 elides default-false on the wire — that's fine; the field
    # just isn't present, but WhichOneof still resolves to the oneof slot
    # because the inner message was set via SetInParent().
    assert frame.request.WhichOneof("payload") == "set_touch_streaming"
    assert frame.request.set_touch_streaming.enabled is False


# ---------------------------------------------------------------------------
# Link.get_telemetry
# ---------------------------------------------------------------------------

def test_link_get_telemetry_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 5
    frame.response.get_telemetry.streaming_events_sent = 12345
    frame.response.get_telemetry.streaming_tx_fails = 7
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 5)

    sent, fails = lnk.get_telemetry()

    assert sent == 12345
    assert fails == 7

    # Verify the host sent a properly-framed GetTelemetryRequest.
    sent_bytes = bytes(t.outgoing)
    decoded = link.cobs_decode(sent_bytes[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(decoded)
    assert out_frame.WhichOneof("kind") == "request"
    assert out_frame.request.WhichOneof("payload") == "get_telemetry"


# ---------------------------------------------------------------------------
# Link.read_events generator
# ---------------------------------------------------------------------------

def _encoded_event_touch_frame(*, live: int, down: int,
                                fingers: list[tuple[int, bool, int, int]]) -> bytes:
    """Build wire bytes for a Frame{event: TouchFrameEvent{...}}."""
    frame = pb.Frame()
    frame.event.touch_frame.live_touches = live
    frame.event.touch_frame.tips_touched_down = down
    for fid, touching, x, y in fingers:
        f = frame.event.touch_frame.fingers.add()
        f.id = fid
        f.touching = touching
        f.x = x
        f.y = y
    return link.cobs_encode(frame.SerializeToString()) + b"\x00"


def test_link_read_events_yields_events_and_skips_responses() -> None:
    # Mix: one event, one stale response, one event.
    e1 = _encoded_event_touch_frame(
        live=1, down=1, fingers=[(0, True, 100, 200)]
    )
    e2 = _encoded_event_touch_frame(
        live=2, down=2, fingers=[(0, True, 100, 200), (1, True, 300, 400)]
    )
    stale_resp = _encoded_response(request_id=999, uptime_ms=42)

    t = _LoopTransport(e1 + stale_resp + e2)
    lnk = link.Link(t)

    events = []
    for event in lnk.read_events():
        events.append(event)
        if len(events) == 2:
            break

    assert len(events) == 2
    assert events[0].WhichOneof("payload") == "touch_frame"
    assert events[0].touch_frame.live_touches == 1
    assert events[1].touch_frame.live_touches == 2


def test_link_read_events_returns_at_eof() -> None:
    # Empty transport: generator should terminate cleanly.
    t = _LoopTransport(b"")
    lnk = link.Link(t)
    events = list(lnk.read_events())
    assert events == []


# ---------------------------------------------------------------------------
# Haptic-area Link methods
# ---------------------------------------------------------------------------

def _haptic_area_wire(*, id: int, tag: str, x0: int, y0: int, x1: int, y1: int,
                       enabled: bool, kind_val: int, haptic_val: int) -> "pb.HapticArea":
    """Helper: build and return a HapticArea protobuf message."""
    area = pb.HapticArea()
    area.id = id
    area.tag = tag
    area.rect.x0 = x0
    area.rect.y0 = y0
    area.rect.x1 = x1
    area.rect.y1 = y1
    area.enabled = enabled
    area.kind = kind_val
    area.haptic = haptic_val
    return area


def test_set_haptic_area_mode_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 10
    frame.response.set_haptic_area_mode.enabled = True
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 10)

    result = lnk.set_haptic_area_mode(enabled=True)
    assert result is True

    # Verify outgoing request.
    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.WhichOneof("kind") == "request"
    assert out_frame.request.request_id == 10
    assert out_frame.request.WhichOneof("payload") == "set_haptic_area_mode"
    assert out_frame.request.set_haptic_area_mode.enabled is True


def test_get_haptic_area_mode_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 11
    frame.response.get_haptic_area_mode.enabled = False
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 11)

    result = lnk.get_haptic_area_mode()
    assert result is False

    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "get_haptic_area_mode"


def test_add_haptic_area_round_trip() -> None:
    # Device assigns id=0xCAFEBABE on creation.
    area = _haptic_area_wire(
        id=0xCAFEBABE, tag="btn1",
        x0=10, y0=20, x1=110, y1=120,
        enabled=True,
        kind_val=pb.HAPTIC_AREA_KIND_FIRE,
        haptic_val=pb.HAPTIC_EFFECT_DEFAULT,
    )
    frame = pb.Frame()
    frame.response.request_id = 12
    frame.response.set_haptic_area.area.CopyFrom(area)
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 12)

    result = lnk.add_haptic_area(x0=10, y0=20, x1=110, y1=120,
                                  tag="btn1", kind="fire",
                                  enabled=True, haptic="default")

    assert result["id"] == 0xCAFEBABE
    assert result["tag"] == "btn1"
    assert result["x0"] == 10
    assert result["y1"] == 120
    assert result["kind"] == "fire"
    assert result["haptic"] == "default"
    assert result["enabled"] is True

    # Verify the outgoing request sent id=0 (new area).
    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "set_haptic_area"
    assert out_frame.request.set_haptic_area.area.id == 0


def test_get_haptic_area_round_trip() -> None:
    area = _haptic_area_wire(
        id=42, tag="zone-a",
        x0=0, y0=0, x1=100, y1=100,
        enabled=True,
        kind_val=pb.HAPTIC_AREA_KIND_BLOCK,
        haptic_val=pb.HAPTIC_EFFECT_DEFAULT,
    )
    frame = pb.Frame()
    frame.response.request_id = 13
    frame.response.get_haptic_area.area.CopyFrom(area)
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 13)

    result = lnk.get_haptic_area(42)

    assert result["id"] == 42
    assert result["tag"] == "zone-a"
    assert result["kind"] == "block"
    assert result["rect"]["x1"] == 100

    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "get_haptic_area"
    assert out_frame.request.get_haptic_area.id == 42


def test_get_haptic_area_not_found_raises_remote_error() -> None:
    frame = pb.Frame()
    frame.response.request_id = 14
    frame.response.error.code = 2
    frame.response.error.message = "not found"
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 14)

    with pytest.raises(link.RemoteError) as exc_info:
        lnk.get_haptic_area(999)
    assert exc_info.value.code == 2
    assert "not found" in str(exc_info.value)


def test_list_haptic_areas_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 15
    frame.response.get_haptic_area_list.ids.extend([1, 2, 3])
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 15)

    result = lnk.list_haptic_areas()
    assert result == [1, 2, 3]

    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "get_haptic_area_list"


def test_delete_haptic_area_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 16
    frame.response.delete_haptic_area.SetInParent()
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 16)

    result = lnk.delete_haptic_area(77)
    assert result is None

    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "delete_haptic_area"
    assert out_frame.request.delete_haptic_area.id == 77


def test_delete_haptic_area_not_found_raises_remote_error() -> None:
    frame = pb.Frame()
    frame.response.request_id = 17
    frame.response.error.code = 2
    frame.response.error.message = "not found"
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 17)

    with pytest.raises(link.RemoteError) as exc_info:
        lnk.delete_haptic_area(999)
    assert exc_info.value.code == 2


def test_delete_all_haptic_areas_round_trip() -> None:
    frame = pb.Frame()
    frame.response.request_id = 18
    frame.response.delete_all_haptic_areas.SetInParent()
    wire = link.cobs_encode(frame.SerializeToString()) + b"\x00"

    t = _LoopTransport(wire)
    lnk = link.Link(t, _next_id=lambda: 18)

    result = lnk.delete_all_haptic_areas()
    assert result is None

    sent = link.cobs_decode(bytes(t.outgoing)[:-1])
    out_frame = pb.Frame()
    out_frame.ParseFromString(sent)
    assert out_frame.request.WhichOneof("payload") == "delete_all_haptic_areas"


def test_update_haptic_area_does_get_then_set() -> None:
    # Fake get_haptic_area response.
    area = _haptic_area_wire(
        id=55, tag="orig",
        x0=0, y0=0, x1=50, y1=50,
        enabled=True,
        kind_val=pb.HAPTIC_AREA_KIND_FIRE,
        haptic_val=pb.HAPTIC_EFFECT_DEFAULT,
    )
    get_frame = pb.Frame()
    get_frame.response.request_id = 20
    get_frame.response.get_haptic_area.area.CopyFrom(area)
    get_wire = link.cobs_encode(get_frame.SerializeToString()) + b"\x00"

    # Fake set_haptic_area response (updated tag).
    updated_area = _haptic_area_wire(
        id=55, tag="updated",
        x0=0, y0=0, x1=50, y1=50,
        enabled=True,
        kind_val=pb.HAPTIC_AREA_KIND_FIRE,
        haptic_val=pb.HAPTIC_EFFECT_DEFAULT,
    )
    set_frame = pb.Frame()
    set_frame.response.request_id = 21
    set_frame.response.set_haptic_area.area.CopyFrom(updated_area)
    set_wire = link.cobs_encode(set_frame.SerializeToString()) + b"\x00"

    # Concatenate both responses; IDs increment as 20, 21.
    id_seq = iter([20, 21])
    t = _LoopTransport(get_wire + set_wire)
    lnk = link.Link(t, _next_id=lambda: next(id_seq))

    result = lnk.update_haptic_area(55, tag="updated")

    assert result["id"] == 55
    assert result["tag"] == "updated"

    # Verify two outgoing frames: Get first, Set second.
    raw_out = bytes(t.outgoing)
    # Split on 0x00 delimiters (each frame ends with 0x00).
    frames_raw = [b for b in raw_out.split(b"\x00") if b]
    assert len(frames_raw) == 2

    get_req = pb.Frame()
    get_req.ParseFromString(link.cobs_decode(frames_raw[0]))
    assert get_req.request.WhichOneof("payload") == "get_haptic_area"
    assert get_req.request.get_haptic_area.id == 55

    set_req = pb.Frame()
    set_req.ParseFromString(link.cobs_decode(frames_raw[1]))
    assert set_req.request.WhichOneof("payload") == "set_haptic_area"
    assert set_req.request.set_haptic_area.area.tag == "updated"
    assert set_req.request.set_haptic_area.area.id == 55
