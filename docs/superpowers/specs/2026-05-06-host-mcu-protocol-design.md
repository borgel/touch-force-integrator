# Host ↔ MCU Serial Protocol — v1 Design

**Status:** Approved 2026-05-06
**Scope:** Foundational design for a protobuf-based serial protocol between the touch-force-integrator firmware and a host. Covers framing, schema conventions, file layout, and the build script. v1 ships exactly one command (`GetUptime`) — the rest is the substrate it grows on.

---

## 1. Goals

- A serial protocol that is **easy to extend** without breaking compatibility.
- **Symmetric tooling** between firmware (nanopb, C) and host (stock protobuf, Python).
- **Self-synchronizing on the wire** so MCU resets and USB-CDC reconnects don't corrupt the host's state machine.
- **Static allocation on the MCU** — no `malloc` in the protobuf path.
- Start small: one command this iteration.

## 2. Non-goals (v1)

- Unsolicited events / telemetry from MCU → host. Will be added later as a third top-level message; explicitly designed to be additive.
- Streaming responses (single request → single response only).
- Multiple host transports (USB CDC is the only target).
- Versioned protocol negotiation (the `package touchforce.v1` namespace leaves room; nothing reads version on the wire yet).
- Webapp host client. The directory layout reserves space for it but no code is written.

## 3. Wire format

A frame on the wire is:

```
[ COBS-encoded( protobuf bytes ) ] [ 0x00 ]
```

- The protobuf payload is encoded with [Consistent Overhead Byte Stuffing](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) so the encoded bytes contain no `0x00`.
- A single `0x00` byte separates frames.
- Receivers resync after corruption, MCU reset, or USB reconnect by reading until the next `0x00`.
- COBS implementation: [`cmcqueen/cobs-c`](https://github.com/cmcqueen/cobs-c) added as a git submodule at `TouchForceIntegrator/Middlewares/cobs-c/`.

**Why COBS over a length prefix:** length-prefix framing has no recovery path if the receiver loses sync — every subsequent byte is misinterpreted as a length. COBS gives unambiguous resync at every frame boundary, which matters because the MCU may reset asynchronously and the host's serial buffer may straddle the reset.

## 4. Schema conventions

### 4.1 Top-level shape

Two top-level messages, each carrying a `oneof payload`:

```proto
syntax = "proto3";
package touchforce.v1;

message Request {
  uint32 request_id = 1;
  // field 2 reserved for symmetry with Response.error
  oneof payload {
    GetUptimeRequest get_uptime = 3;
  }
}

message Response {
  uint32 request_id = 1;
  oneof payload {
    ErrorResponse     error      = 2;
    GetUptimeResponse get_uptime = 3;
  }
}

message ErrorResponse {
  uint32 code    = 1;
  string message = 2;
}

message GetUptimeRequest  {}
message GetUptimeResponse { uint32 uptime_ms = 1; }
```

### 4.2 Conventions

- **Every command is a `XxxRequest` / `XxxResponse` pair**, slotted into the two `oneof payload` blocks at the *same* field number in both messages. Field 2 is reserved across both messages for `Response.error`, so command field numbers start at 3.
- **`request_id` is set by the host on every request and echoed verbatim by the MCU on every response.** This decouples request/response correlation from FIFO ordering and prepares for any future async message interleaving. The host is responsible for picking IDs; the MCU treats them as opaque. Hosts SHOULD use non-zero IDs because proto3 elides zero defaults on the wire, which makes "the host forgot to set this" indistinguishable from "the host set it to zero."
- **`ErrorResponse` is in the `Response` oneof.** Every command can fail uniformly. Error codes will be enumerated as the protocol grows; v1 reserves `code = 0` for "unspecified."
- **Empty messages stay defined** (`GetUptimeRequest {}`) rather than collapsing to a tag-only field, so future versions can add fields without a wire-format break.
- **Package is `touchforce.v1`.** A `v2` namespace can coexist if we ever need a hard break, but the ordinary path is additive evolution within v1.

### 4.3 nanopb field constraints

A `touch_force.options` file colocated with the `.proto` bounds every variable-length field so nanopb generates fixed-size structs:

```
ErrorResponse.message  max_size:64
```

Without bounds, nanopb either produces `pb_callback_t` fields (caller writes encode/decode callbacks — painful) or — if explicitly opted in — falls back to `malloc`. Bounded fields keep firmware allocation static and predictable.

## 5. File layout

```
touch-force-integrator/
├── protocol/                                    # bare schema + tooling
│   ├── touch_force.proto
│   ├── touch_force.options
│   ├── compile.sh
│   ├── README.md                                # protocol guide (separate deliverable)
│   └── generated/                               # gitignored staging dir
│       ├── c/
│       │   ├── touch_force.pb.h
│       │   └── touch_force.pb.c
│       └── python/
│           └── touch_force_pb2.py
│
├── host/                                        # host-side tooling root
│   └── touch_force_cli/
│       ├── __init__.py
│       ├── touch_force_pb2.py                   # generated, committed
│       ├── link.py                              # COBS framing + req/resp over pyserial
│       └── cli.py                               # entry point
│
└── TouchForceIntegrator/
    ├── Middlewares/
    │   ├── nanopb/                              # existing submodule
    │   └── cobs-c/                              # new submodule (cmcqueen/cobs-c)
    └── Appli/
        ├── Inc/proto/
        │   └── touch_force.pb.h                 # generated, committed
        └── Src/proto/
            └── touch_force.pb.c                 # generated, committed
```

**Decisions encoded in the layout:**

- **Generated files are committed** in their final firmware/host locations. The firmware build (STM32CubeIDE-driven) does not run `protoc`. CI may add a drift check later.
- **`protocol/generated/`** is the single inspection point for raw generator output. Gitignored, wiped and regenerated on every `compile.sh` run. Useful for debugging `pb_callback_t` surprises and for diffing across regenerations.
- **`host/`** sits at the repo root, not under `TouchForceIntegrator/`, because it is host-side tooling, not firmware.
- **Future sibling host packages** (e.g. `host/touch_force_webapp/`) will live next to `touch_force_cli/`. Generated code and framing will *not* be pre-extracted into a shared package; that refactor waits until there is a real second consumer to design against.

## 6. `compile.sh`

A bash script at `protocol/compile.sh`. Idempotent, hermetic with respect to the repo, fails fast.

**Behavior:**

1. `set -euo pipefail`.
2. `cd` to the script's own directory (`protocol/`) at the top, so all relative paths in the script resolve regardless of how the script is invoked.
3. Sanity checks: `protoc` on PATH, `python3` on PATH, nanopb submodule initialized.
4. Wipe and recreate `protocol/generated/{c,python}/`.
5. Generate C with nanopb's generator into `protocol/generated/c/`:
   ```
   python3 ../TouchForceIntegrator/Middlewares/nanopb/generator/nanopb_generator.py \
       --output-dir=generated/c touch_force.proto
   ```
6. Generate Python with stock `protoc` into `protocol/generated/python/`:
   ```
   protoc --python_out=generated/python touch_force.proto
   ```
7. Copy artifacts to their committed locations:
   - `generated/c/touch_force.pb.h` → `TouchForceIntegrator/Appli/Inc/proto/`
   - `generated/c/touch_force.pb.c` → `TouchForceIntegrator/Appli/Src/proto/`
   - `generated/python/touch_force_pb2.py` → `host/touch_force_cli/`
8. Each copy logs `src → dst` so a regen leaves an audit trail.

**Why bash:** the script is small shell glue; Python adds a runtime requirement without simplifying the work. Can be revisited if the script grows non-trivial logic.

## 7. Documentation

Two documents accompany the implementation:

- **This spec** — frozen design rationale.
- **`protocol/README.md`** (the *protocol guide*) — a living how-to-speak-the-protocol reference for humans and agents. Covers wire format, framing rules, the message catalog, examples of encoding/decoding a `GetUptime` exchange, and the nanopb bounded-field caveats. Updated whenever the schema grows.

## 8. Open questions / deferred decisions

- **Error code enumeration.** v1 reserves `code = 0` as unspecified; a `proto3 enum ErrorCode` will be introduced when there is more than one real error to distinguish.
- **Request ID generation strategy on the host.** The host implementation will start with a monotonic counter; documented in the protocol guide, not the wire format.
- **Drift detection in CI.** Not in v1. If generated-file drift becomes a real source of bugs, add a CI step that re-runs `compile.sh` and `git diff --exit-code`s.
- **Shared host-side package.** Defer until the webapp arrives.
