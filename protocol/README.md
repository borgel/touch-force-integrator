# touchforce.v1 Serial Protocol

Protobuf request/response/event protocol over USB CDC between the
TouchForceIntegrator firmware and a host.

## Wire format

    [ COBS-encoded( serialized Frame protobuf ) ] [ 0x00 ]

Every frame on the wire is a `Frame` message (defined in
`touch_force.proto`) carrying exactly one of `request`, `response`, or
`event`. COBS guarantees no `0x00` inside the encoded payload; a single
`0x00` separates frames; receivers resync to the next `0x00` after any
corruption or MCU reset.

## Schema

See `touch_force.proto`. Top-level messages:

- `Frame { oneof kind { Request, Response, Event } }` — every wire frame.
- `Request` — host → MCU. Carries `request_id` + a payload oneof.
- `Response` — MCU → host. Carries the same `request_id` + a payload oneof.
- `Event` — MCU → host async, no correlation.

### Conventions

- `request_id` is opaque, host-chosen, 32-bit. The MCU echoes it on Responses. Hosts SHOULD use non-zero IDs (proto3 elides zero defaults). Fire-and-forget commands have no Response so the id isn't used by the host.
- Field 2 in `Request.payload` and `Response.payload` is reserved for `Response.error`. Command oneof slots start at field 3 with parallel numbering when both directions have the slot. Gaps (a Request slot with no Response peer) are acceptable for fire-and-forget.
- proto3 forbids `reserved` inside `oneof`, so the gap at field 2 in `Request.payload` is convention only — don't try to add `reserved 2;` there.
- Empty messages (`GetUptimeRequest {}`) stay defined rather than collapsed; future fields are additive.
- Package is `touchforce.v1`. Additive evolution stays here; a real wire break would bump to `v2`.

## Command catalog

| Command            | Direction       | Field # | Notes                                                                |
|--------------------|-----------------|---------|----------------------------------------------------------------------|
| `GetUptime`        | Request → Response | 3    | empty Request; Response carries `uint32 uptime_ms`                   |
| `SetTouchStreaming`| Request only       | 4    | fire-and-forget; bool `enabled`. Default at boot is true.            |
| `GetTelemetry`     | Request → Response | 5    | empty Request; Response carries `streaming_events_sent` + `streaming_tx_fails` |
| `SetHapticArea`      | Request → Response | 6    | full overwrite at `area.id`; `id == 0` ⇒ create new (random uint32 from HW RNG); Response echoes the assigned record. Errors: `3` table-full, `4` invalid rect. |
| `GetHapticArea`      | Request → Response | 7    | by `id`; Response carries the full record or `Error{code=2}` if not found. |
| `GetHapticAreaList`  | Request → Response | 8    | empty Request; Response carries `repeated uint32 ids` (up to 1024). |
| `DeleteHapticArea`   | Request → Response | 9    | by `id`; Response is empty `OkResponse` or `Error{code=2}`. |
| `DeleteAllHapticAreas`| Request → Response| 10   | empty both ways; Response is `OkResponse`. |
| `SetHapticAreaMode`  | Request → Response | 11   | bool `enabled`; Response echoes the now-current state. Default at boot: false (every touch fires globally). |
| `GetHapticAreaMode`  | Request → Response | 12   | empty Request; Response carries the current `enabled` bool. |
| `TouchFrame`       | Event              | 1    | streamed per wisecoco frame when streaming enabled (see §Events)     |

`Response.payload` always carries one of the command-specific responses or `ErrorResponse` (field 2, `code` + `message`). Defined error codes:

- `1` — unknown command (Request payload tag the MCU doesn't recognize, or empty oneof).
- `2` — not found (`GetHapticArea` / `DeleteHapticArea` with an unknown id).
- `3` — table full (`SetHapticArea` with id=0 when 1024 areas already exist).
- `4` — invalid rect (`SetHapticArea` with x0≥x1, y0≥y1, or coords past the panel: x ∈ [0, 2880], y ∈ [0, 2160]).

## Events

`TouchFrameEvent` mirrors `USBH_LatestWisecocoData`:

- `live_touches` — count of fingers reported in any state this frame.
- `tips_touched_down` — count actively touching this frame.
- `repeated TouchFinger fingers` — bounded at 10 entries (`max_count:10`).

`TouchFinger`:

- `id` — stable per-touch identifier (uint32).
- `touching` — true while down; goes false on the up frame, then the entry disappears.
- `x`, `y` — pixels, post-rotation. Firmware default rotation is `USBH_WC_ROTATE_90`, so x ∈ [0, 2880], y ∈ [0, 2160]. Patch dimensions swap with axes for ROTATE_90/270.
- `patch_width`, `patch_height` — contact-patch size in pixels.
- `touch_duration` — microseconds since this id was first seen.

Streaming default-on. Drop-on-busy: if the firmware's CDC TX is full when an event needs to send (5ms timeout), the event is dropped silently and `streaming_tx_fails` increments. Render path is never blocked by a slow host.

## Haptic areas

`HapticArea` is a host-defined rectangle that, when haptic-area mode is enabled, decides whether a rising-edge touch fires a haptic.

- Coordinates are panel pixels (post-rotation, matching `TouchFinger.x/y`): x ∈ [0, 2880], y ∈ [0, 2160].
- Rect is half-open: `x0 ≤ x < x1 && y0 ≤ y < y1`. Adjacent rects tile without ambiguous edges.
- `kind`: `FIRE` (touch fires haptic) or `BLOCK` (touch suppresses fire on overlap). First match wins; BLOCK beats FIRE.
- `enabled=false` excludes the area from hit-testing (stage without arming).
- `id == 0` is the "create new" sentinel on `SetHapticArea`; stored ids are non-zero random uint32s (HW RNG).
- `tag` is host-supplied, bounded at 32 bytes.

### Modes

- **disabled** (boot default): every touch rising edge fires `HAPTIC_EFFECT_DEFAULT`. Areas are ignored.
- **enabled**: rising edge inside an enabled `FIRE` area fires; an enabled `BLOCK` area suppresses on overlap.

### Phase 0 placeholder haptic

`HAPTIC_EFFECT_DEFAULT` pulses LD1 (user LED) for 100 ms via a FreeRTOS one-shot timer. A burst within 100 ms extends the pulse. Real haptic hardware lands in a later phase; the enum exists so effects can be added without breaking the schema.

## nanopb caveats for host implementors

- `ErrorResponse.message` is bounded at 64 bytes (`touch_force.options`). Longer host-side messages are silently truncated by nanopb's decoder if they ever round-trip.
- `TouchFrameEvent.fingers` is bounded at 10 entries. Future variable-length fields need similar bounds in `.options` — the package-qualified form (`touchforce.v1.MessageName.field max_size:N`) is required because nanopb runs as a protoc plugin.
- `compile.sh` enforces this: a missing bound makes nanopb emit `pb_callback_t` and the script fails before copying artifacts into the firmware tree.

## Adding a new command

1. Define `XxxRequest` (and `XxxResponse` if not fire-and-forget) in `touch_force.proto`.
2. Add to `Request.payload` (and `Response.payload` if applicable) at the next free field number.
3. Add any `max_size`/`max_count` constraints to `touch_force.options`, fully qualified.
4. Run `./protocol/compile.sh`.
5. Firmware: add a case to the dispatch in `Appli/Src/protocol_task.c::handle_frame`. For fire-and-forget commands, return without calling `send_response`.
6. Host: add a `Link` method in `host/touch_force_cli/link.py` (with TDD tests in `host/tests/test_link.py`), plus a CLI subcommand in `cli.py` and/or a webapp button in `host/touch_force_webapp/index.html` if appropriate.
7. Update this catalog.
