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
