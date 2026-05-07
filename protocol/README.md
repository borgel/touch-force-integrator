# touchforce.v1 Serial Protocol

A protobuf-based request/response protocol over USB CDC, between the
TouchForceIntegrator firmware (MCU) and a host.

## Wire format

Each frame on the wire is:

    [ COBS-encoded( protobuf bytes ) ] [ 0x00 ]

- COBS ([Consistent Overhead Byte Stuffing](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)) ensures the encoded bytes contain no `0x00`.
- A single `0x00` byte separates frames.
- Receivers resync after corruption or MCU reset by reading until the
  next `0x00`.

## Schema overview

Defined in `touch_force.proto`. Two top-level messages:

- `Request` — sent host -> MCU. Carries a `request_id` plus a
  `oneof payload` selecting one of the commands.
- `Response` — sent MCU -> host. Carries the same `request_id` and a
  `oneof payload` selecting the matching command's response or an
  `ErrorResponse`.

### Conventions

- **`request_id`** — opaque 32-bit value chosen by the host. The MCU
  echoes it verbatim. Hosts SHOULD use non-zero IDs because proto3
  elides zero defaults on the wire, so 0 is indistinguishable from
  "unset". The reference CLI uses a monotonic counter starting at 1.
- **Field 2 in both `Request` and `Response`** is reserved across the
  protocol for `Response.error`. Command oneof slots therefore start
  at field 3 and use parallel numbers in `Request` and `Response`.
  Note: proto3 forbids `reserved` statements inside a `oneof` block,
  so the field-2 gap in `Request.payload` is enforced by convention
  and a comment, not by the language. Don't try to add `reserved 2;`
  inside the oneof — protoc will reject it.
- **Empty messages are still defined** (e.g. `GetUptimeRequest {}`)
  rather than collapsed into a tag, so future versions can add
  fields without a wire-format break.
- **Package is `touchforce.v1`.** A future `v2` namespace can coexist
  if a hard break is ever needed; the ordinary path is additive.

## Command catalog (v1)

### GetUptime

Returns the MCU's uptime in milliseconds since boot.

| Direction   | Field             | Wire field # | Notes                                       |
|-------------|-------------------|--------------|---------------------------------------------|
| Request     | `get_uptime`      | 3            | empty `GetUptimeRequest`                    |
| Response    | `get_uptime`      | 3            | `uint32 uptime_ms`                          |
| Response    | `error`           | 2            | only on dispatch failure; see below         |

Note: `uptime_ms` is `HAL_GetTick()` cast to `uint32_t`. It wraps at
~49.7 days. Hosts that need to reason about "is this a fresh boot?"
should not assume monotonicity across that boundary.

## ErrorResponse

Every command can fail with an `ErrorResponse` instead of its
command-specific response. The `request_id` is still echoed, so host
correlation is unaffected.

| Field   | Type     | Notes                                              |
|---------|----------|----------------------------------------------------|
| `code`  | `uint32` | reserved code 0 = unspecified                      |
| `message` | `string` | human-readable, bounded at 64 bytes (nanopb)     |

Currently defined codes:
- `1` — unknown command (host sent a `Request.payload` tag the MCU
  doesn't recognize, e.g. host running a newer protocol version, or
  a Request with no payload set).

## nanopb-specific caveats for host implementors

- `ErrorResponse.message` is **bounded at 64 bytes** on the MCU
  side (`touch_force.options`). The MCU will silently truncate
  longer strings; full-protobuf hosts that send arbitrary lengths
  will succeed at encode time but the string they receive back as
  an echo (if one ever exists) would be truncated.
- Future `bytes` or `string` fields will need similar bounds. When
  adding them to `touch_force.proto`, also add a corresponding
  `<package>.<MessageName>.<field> max_size:N` line to
  `touch_force.options`. The fully-qualified package prefix
  (`touchforce.v1.`) is required because the nanopb generator runs
  in protoc plugin mode, which delivers field descriptors with
  their package prefix attached.
- `compile.sh` enforces this: a missing bound makes nanopb emit
  `pb_callback_t` for the field, and a guard in the script fails
  the build before any artifacts get copied into the firmware tree.

## Worked example: GetUptime exchange

Host sends:

    Request { request_id: 1, get_uptime: GetUptimeRequest{} }
    serialized:  08 01 1A 00            # 4 bytes (or close)
    COBS-encoded: <varies — encoder output>
    on wire: <COBS bytes> 00

MCU replies:

    Response { request_id: 1, get_uptime: GetUptimeResponse{ uptime_ms: 12345 } }
    serialized:  08 01 1A 04 08 B9 60   # depends on uptime
    COBS-encoded: <varies>
    on wire: <COBS bytes> 00

The reference Python CLI (`uv run python -m touch_force_cli
get-uptime --port <dev>`, run from `host/`) is the canonical
worked example.

## Adding a new command

1. Define `XxxRequest` and `XxxResponse` messages in
   `touch_force.proto`.
2. Add an entry to the `Request.payload` and `Response.payload`
   oneofs at the same field number (>= 3, never reusing a number).
3. Add any `max_size`/`max_count` constraints to
   `touch_force.options`, fully qualified with the
   `touchforce.v1.` package prefix.
4. Run `./protocol/compile.sh` from the repo root.
5. On the MCU side, add a `case` to the switch in
   `Appli/Src/protocol_task.c::handle_frame` and a fill helper
   (mirroring `fill_get_uptime_response`).
6. On the host side, add a method to `Link` in
   `host/touch_force_cli/link.py`, plus a CLI subcommand if
   appropriate.
7. Update this document's command catalog.
