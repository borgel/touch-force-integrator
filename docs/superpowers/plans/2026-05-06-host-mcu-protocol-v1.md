# Host ↔ MCU Serial Protocol v1 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a working request/response protocol over USB CDC where `uv run python -m touch_force_cli get-uptime` (run from `host/`) returns the MCU's uptime in milliseconds, on a feature branch with modular commits, end-to-end.

**Architecture:** Protobuf schema (nanopb on MCU, stock protobuf on host) with `Request`/`Response` envelope messages each carrying a `oneof payload`. COBS-encoded frames terminated by `0x00` for self-synchronizing framing over the USB CDC byte stream. Generated C files committed into the firmware tree; generated Python committed into the host package. The existing `_CDC_Task` in `Appli/Src/main.c` is repurposed from echo to protocol dispatcher; the existing blocking `CDC_Read`/`CDC_Write` API in `usbd_cdc_if.c` is reused unchanged.

**Tech Stack:** nanopb (existing submodule), [`cmcqueen/cobs-c`](https://github.com/cmcqueen/cobs-c) (new submodule), STM32CubeIDE/GCC for firmware, Python 3 with `protobuf` and `pyserial` for the host CLI managed by [`uv`](https://docs.astral.sh/uv/), `pytest` for host unit tests.

**Reference spec:** [`docs/superpowers/specs/2026-05-06-host-mcu-protocol-design.md`](../specs/2026-05-06-host-mcu-protocol-design.md)

---

## Working assumptions

- Firmware build is done by the user in STM32CubeIDE. The plan edits `.project` and `.cproject` directly to register new sources and include paths so the user does not need to add files via the IDE GUI.
- The firmware already has a working USB CDC stack with blocking helpers `CDC_Read(buf, maxLen, timeoutMs)` and `CDC_Write(buf, len, timeoutMs)` defined in `Appli/Src/usbd_cdc_if.c`.
- The existing `_CDC_Task` is currently a "echo: " demo loop in `Appli/Src/main.c`; we replace its body, keep its FreeRTOS attributes.
- nanopb is already a submodule at `TouchForceIntegrator/Middlewares/nanopb`. Its sources `pb_common.c`, `pb_encode.c`, `pb_decode.c` are not yet registered with the IDE project.
- All commits in this plan land on a single `protocol-v1` feature branch.
- All `git` commands assume cwd = repo root `/Users/borgel/working/touch-force-integrator`. All `protoc`/`compile.sh` invocations assume cwd = `protocol/`. Each task notes its assumed cwd.
- The host Python project is managed with [`uv`](https://docs.astral.sh/uv/) — install it first (e.g. `brew install uv` on macOS) before running Phase 3. `uv` reads the `pyproject.toml` directly and produces a committed `uv.lock` for reproducible installs; no manual venv management required.

---

## Phase 0 — Branch

### Task 0.1: Create the feature branch

**Files:** none — branch only.

- [ ] **Step 1: Create and switch to `protocol-v1`**

cwd: repo root.

```bash
git checkout -b protocol-v1
git status
```

Expected: `On branch protocol-v1`, working tree clean (or only the unrelated untracked items already noted in `git status` at session start).

---

## Phase 1 — Schema, options, and compile script

### Task 1.1: Add cobs-c as a submodule

**Files:**
- Modify: `.gitmodules`
- Create: `TouchForceIntegrator/Middlewares/cobs-c/` (submodule contents)

- [ ] **Step 1: Add the submodule**

cwd: repo root.

```bash
git submodule add https://github.com/cmcqueen/cobs-c.git TouchForceIntegrator/Middlewares/cobs-c
```

Expected: clones into `TouchForceIntegrator/Middlewares/cobs-c/`, updates `.gitmodules`, stages both.

- [ ] **Step 2: Verify the expected source files exist**

```bash
ls TouchForceIntegrator/Middlewares/cobs-c/cobs.h \
   TouchForceIntegrator/Middlewares/cobs-c/cobs.c
```

Expected: both paths print without error. (cmcqueen's repo ships `cobs.c`/`cobs.h` at the top level.)

- [ ] **Step 3: Commit**

```bash
git add .gitmodules TouchForceIntegrator/Middlewares/cobs-c
git commit -m "$(cat <<'EOF'
Add cmcqueen/cobs-c as a submodule

Provides the COBS encoder/decoder used by the new host-MCU serial
protocol's framing layer. Self-synchronizing framing matters because
the MCU may reset asynchronously and the host's serial buffer can
straddle the reset; with COBS the host resyncs on the next 0x00 byte.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 1.2: Create the `.proto` and `.options` files

**Files:**
- Create: `protocol/touch_force.proto`
- Create: `protocol/touch_force.options`

- [ ] **Step 1: Write `protocol/touch_force.proto`**

```proto
syntax = "proto3";

package touchforce.v1;

// Top-level wrapper for every host -> MCU message.
//
// Conventions (locked in by 2026-05-06 design spec):
//   - request_id: opaque uint32 chosen by the host, echoed by the MCU
//     in Response.request_id. SHOULD be non-zero (proto3 elides
//     zero defaults on the wire, so 0 is indistinguishable from
//     "unset").
//   - Field 2 in both Request and Response is reserved across the
//     whole protocol for Response.error, so command oneof slots
//     start at field 3 with parallel numbers across Request/Response.
message Request {
  uint32 request_id = 1;
  // field 2 reserved for symmetry with Response.error
  oneof payload {
    GetUptimeRequest get_uptime = 3;
  }
}

// Top-level wrapper for every MCU -> host message (in v1, only
// solicited responses; future versions may add an Event sibling).
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

message GetUptimeResponse {
  // Milliseconds since MCU boot (HAL_GetTick wraps at ~49.7 days,
  // documented in protocol/README.md).
  uint32 uptime_ms = 1;
}
```

- [ ] **Step 2: Write `protocol/touch_force.options`**

```
# nanopb field-size constraints. Without these, nanopb emits
# pb_callback_t for variable-length fields, which forces the caller
# to write encode/decode callbacks. We want fixed-size structs and
# zero malloc on the MCU.
ErrorResponse.message  max_size:64
```

### Task 1.3: Write `protocol/compile.sh`

**Files:**
- Create: `protocol/compile.sh`
- Create: `protocol/.gitignore`

- [ ] **Step 1: Write `protocol/compile.sh`**

```bash
#!/usr/bin/env bash
#
# Regenerate nanopb C and stock-protoc Python sources from
# touch_force.proto. Outputs land in protocol/generated/{c,python}/
# (gitignored, for inspection), and are then copied to their
# committed locations in the firmware and host trees.

set -euo pipefail

# Always run from the directory this script lives in so all
# relative paths below resolve regardless of the caller's cwd.
cd "$(dirname "$0")"

# --- Sanity checks ----------------------------------------------------------

if ! command -v protoc >/dev/null 2>&1; then
  echo "error: protoc not on PATH (install protobuf-compiler)" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 not on PATH" >&2
  exit 1
fi

NANOPB_GEN="../TouchForceIntegrator/Middlewares/nanopb/generator/nanopb_generator.py"
if [[ ! -f "$NANOPB_GEN" ]]; then
  echo "error: nanopb generator not found at $NANOPB_GEN" >&2
  echo "       run: git submodule update --init --recursive" >&2
  exit 1
fi

# --- Staging directory ------------------------------------------------------

rm -rf generated
mkdir -p generated/c generated/python

# --- Generate ---------------------------------------------------------------

echo "==> nanopb generator -> generated/c/"
python3 "$NANOPB_GEN" --output-dir=generated/c touch_force.proto

echo "==> protoc --python_out -> generated/python/"
protoc --python_out=generated/python touch_force.proto

# --- Copy to committed locations -------------------------------------------

copy() {
  local src=$1 dst=$2
  echo "    $src -> $dst"
  install -m 0644 -D "$src" "$dst"
}

echo "==> copying artifacts to committed locations"
copy generated/c/touch_force.pb.h \
     ../TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h
copy generated/c/touch_force.pb.c \
     ../TouchForceIntegrator/Appli/Src/proto/touch_force.pb.c
copy generated/python/touch_force_pb2.py \
     ../host/touch_force_cli/touch_force_pb2.py

echo "==> done"
```

- [ ] **Step 2: Make it executable**

cwd: repo root.

```bash
chmod +x protocol/compile.sh
```

- [ ] **Step 3: Write `protocol/.gitignore`**

```
# generator staging output; the canonical copies live in the
# firmware and host trees and are committed there.
generated/
```

- [ ] **Step 4: Commit**

```bash
git add protocol/touch_force.proto protocol/touch_force.options \
        protocol/compile.sh protocol/.gitignore
git commit -m "$(cat <<'EOF'
Add v1 protocol schema and codegen script

touch_force.proto carries the Request/Response envelope messages
plus the GetUptime command. touch_force.options bounds the only
variable-length field so nanopb emits fixed-size structs.

compile.sh regenerates nanopb C and stock-protoc Python into a
gitignored protocol/generated/ staging dir, then copies the
artifacts to their committed locations under Appli/ and host/.
The staging dir is the single inspection point if the generator
does something surprising.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 1.4: Create the destination directories and run the generator

**Files:**
- Create: `TouchForceIntegrator/Appli/Inc/proto/` (with `.gitkeep`)
- Create: `TouchForceIntegrator/Appli/Src/proto/` (with `.gitkeep`)
- Create: `host/touch_force_cli/__init__.py`
- Create (via generator): `TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h`
- Create (via generator): `TouchForceIntegrator/Appli/Src/proto/touch_force.pb.c`
- Create (via generator): `host/touch_force_cli/touch_force_pb2.py`

- [ ] **Step 1: Create destination directories**

cwd: repo root.

```bash
mkdir -p TouchForceIntegrator/Appli/Inc/proto \
         TouchForceIntegrator/Appli/Src/proto \
         host/touch_force_cli
```

- [ ] **Step 2: Create `host/touch_force_cli/__init__.py`** (empty file marks it as a Python package)

```bash
: > host/touch_force_cli/__init__.py
```

- [ ] **Step 3: Verify nanopb's `protoc-gen-nanopb` plugin discovers the .options file**

The nanopb generator looks for `<basename>.options` next to the `.proto`. Confirm prerequisites for the run:

```bash
git submodule update --init --recursive
which protoc python3
```

Expected: `protoc` and `python3` print absolute paths. If `protoc` is missing on this machine, install via `brew install protobuf` (macOS) or `apt install protobuf-compiler` (Linux) before continuing.

- [ ] **Step 4: Run the script**

cwd: repo root.

```bash
./protocol/compile.sh
```

Expected output (paths and exact ordering):

```
==> nanopb generator -> generated/c/
==> protoc --python_out -> generated/python/
==> copying artifacts to committed locations
    generated/c/touch_force.pb.h -> ../TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h
    generated/c/touch_force.pb.c -> ../TouchForceIntegrator/Appli/Src/proto/touch_force.pb.c
    generated/python/touch_force_pb2.py -> ../host/touch_force_cli/touch_force_pb2.py
==> done
```

- [ ] **Step 5: Spot-check the generated C**

```bash
grep -E "ErrorResponse_message_size|max_size" \
     TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h
```

Expected: at least one match showing the `message` field is bounded (e.g., `char message[64];`). If the grep returns nothing, the `.options` file was not picked up — re-check that it's named exactly `touch_force.options` and lives next to the `.proto`.

- [ ] **Step 6: Spot-check the generated Python**

```bash
python3 -c "import sys; sys.path.insert(0, 'host'); from touch_force_cli import touch_force_pb2 as p; r = p.Request(request_id=42); print(r)"
```

Expected: prints `request_id: 42`.

- [ ] **Step 7: Commit**

```bash
git add host/touch_force_cli/__init__.py \
        host/touch_force_cli/touch_force_pb2.py \
        TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h \
        TouchForceIntegrator/Appli/Src/proto/touch_force.pb.c
git commit -m "$(cat <<'EOF'
Generate v1 protocol artifacts

First run of protocol/compile.sh, producing the committed nanopb
C sources, the stock-protoc Python module, and the host package
skeleton. ErrorResponse.message is bounded at 64 bytes per the
.options file, so the generated structs are fixed-size.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Phase 2 — Firmware integration

### Task 2.1: Register cobs-c, nanopb, generated proto, and protocol_task sources with the CubeIDE project

**Files:**
- Modify: `TouchForceIntegrator/STM32CubeIDE/Appli/.project` (add `<link>` entries)
- Modify: `TouchForceIntegrator/STM32CubeIDE/Appli/.cproject` (add include paths)

We add seven source-file links and four include paths. Sources: `cobs.c`, `pb_common.c`, `pb_encode.c`, `pb_decode.c`, `touch_force.pb.c`, plus the protocol task module created in Task 2.2 (`protocol_task.c`). Include paths: cobs-c root, nanopb root, generated proto inc, protocol task inc (which is `Appli/Inc` — already registered, skip).

- [ ] **Step 1: Add `<link>` entries to `.project`**

Open `/Users/borgel/working/touch-force-integrator/TouchForceIntegrator/STM32CubeIDE/Appli/.project` and locate the last existing `<link>` block before `</linkedResources>`. Insert these new entries immediately before the closing `</linkedResources>` tag:

```xml
		<link>
			<name>Application/User/protocol_task.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Appli/Src/protocol_task.c</locationURI>
		</link>
		<link>
			<name>Application/User/proto/touch_force.pb.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Appli/Src/proto/touch_force.pb.c</locationURI>
		</link>
		<link>
			<name>Middlewares/nanopb/pb_common.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Middlewares/nanopb/pb_common.c</locationURI>
		</link>
		<link>
			<name>Middlewares/nanopb/pb_encode.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Middlewares/nanopb/pb_encode.c</locationURI>
		</link>
		<link>
			<name>Middlewares/nanopb/pb_decode.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Middlewares/nanopb/pb_decode.c</locationURI>
		</link>
		<link>
			<name>Middlewares/cobs-c/cobs.c</name>
			<type>1</type>
			<locationURI>PARENT-2-PROJECT_LOC/Middlewares/cobs-c/cobs.c</locationURI>
		</link>
```

Use the Edit tool with `old_string` = `\t</linkedResources>` and `new_string` containing the six `<link>` blocks above followed by `\t</linkedResources>`. (Existing siblings end with `\t\t</link>\n` — tabs, not spaces — so match the surrounding indentation.)

- [ ] **Step 2: Add include paths to `.cproject`**

The `.cproject` defines two `<listOptionValue>` lists for include paths — one for Debug, one for Release. Both currently end with the entry `../../../Middlewares/ST/STM32_USBPD_Library/Core/lib`. We insert four new entries immediately after that one in BOTH lists:

```xml
									<listOptionValue builtIn="false" value="../../../Middlewares/nanopb"/>
									<listOptionValue builtIn="false" value="../../../Middlewares/cobs-c"/>
									<listOptionValue builtIn="false" value="../../../Appli/Inc/proto"/>
```

(`Appli/Inc` is already on the include list, so `protocol_task.h` is reachable without a fourth entry.)

Use the Edit tool with `replace_all=true` and:
- `old_string`: `<listOptionValue builtIn="false" value="../../../Middlewares/ST/STM32_USBPD_Library/Core/lib"/>`
- `new_string`: same line followed by the three new `listOptionValue` lines (preserving the leading whitespace).

- [ ] **Step 3: Confirm the file edits parsed (XML is well-formed)**

```bash
python3 -c "import xml.etree.ElementTree as E; E.parse('TouchForceIntegrator/STM32CubeIDE/Appli/.project'); E.parse('TouchForceIntegrator/STM32CubeIDE/Appli/.cproject'); print('ok')"
```

Expected: `ok` printed. If a parse error fires, the XML edit was malformed — re-read the file at the offending line and fix.

- [ ] **Step 4: Confirm new entries are present**

```bash
grep -c "Middlewares/nanopb/pb_encode.c" TouchForceIntegrator/STM32CubeIDE/Appli/.project
grep -c "../../../Middlewares/nanopb"     TouchForceIntegrator/STM32CubeIDE/Appli/.cproject
```

Expected: first command prints `1`, second prints `2` (Debug + Release).

- [ ] **Step 5: Commit**

```bash
git add TouchForceIntegrator/STM32CubeIDE/Appli/.project \
        TouchForceIntegrator/STM32CubeIDE/Appli/.cproject
git commit -m "$(cat <<'EOF'
Wire nanopb, cobs-c, and protocol sources into the IDE project

Adds source-file linked resources for nanopb (pb_common, pb_encode,
pb_decode), cobs-c, the generated touch_force.pb.c, and the new
protocol_task.c. Adds include paths for nanopb, cobs-c, and the
generated proto headers in both Debug and Release configurations.

Editing .project/.cproject directly avoids forcing IDE GUI clicks.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 2.2: Implement the protocol task module (frame I/O + dispatch)

**Files:**
- Create: `TouchForceIntegrator/Appli/Inc/protocol_task.h`
- Create: `TouchForceIntegrator/Appli/Src/protocol_task.c`

The module exposes one entry point (`Protocol_RunForever`) intended to be called from `_CDC_Task` after CDC enumeration. It:

1. Drains bytes from `CDC_Read` into a frame buffer until it sees `0x00`.
2. Decodes the frame with COBS into a raw protobuf buffer.
3. nanopb-decodes the raw bytes as a `touchforce_v1_Request`.
4. Dispatches on the `which_payload` tag.
5. nanopb-encodes a `touchforce_v1_Response`, COBS-encodes it, appends `0x00`, and writes it via `CDC_Write`.

Frames longer than `PROTO_FRAME_MAX_BYTES` are dropped; the next `0x00` resyncs.

- [ ] **Step 1: Write `Appli/Inc/protocol_task.h`**

```c
#ifndef PROTOCOL_TASK_H
#define PROTOCOL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Run the protocol dispatcher in the calling FreeRTOS task forever.
 * Blocks on CDC_Read, decodes COBS-framed protobuf Requests, and
 * writes COBS-framed protobuf Responses back via CDC_Write.
 *
 * Expects the caller (currently _CDC_Task) to have already waited
 * for USB enumeration before calling.
 */
void Protocol_RunForever(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_TASK_H */
```

- [ ] **Step 2: Write `Appli/Src/protocol_task.c`**

```c
#include "protocol_task.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include "cobs.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "touch_force.pb.h"

#include <stdint.h>
#include <string.h>

/* Worst-case sizes for v1.
 *
 *   Request encoded:           request_id (5) + oneof tag (1) + sub-msg
 *                              length (1) + GetUptimeRequest body (0)
 *                              = 7 bytes typical.
 *   Response encoded:          request_id (5) + oneof tag (1) + sub-msg
 *                              length (1) + GetUptimeResponse uptime_ms
 *                              (5) = 12 bytes typical, ErrorResponse
 *                              with full 64-byte message ~75 bytes.
 *
 * COBS overhead: 1 byte per 254 bytes plus a leading overhead byte.
 * For our sizes that means +2 bytes worst case.
 *
 * 128 is comfortably oversized and easy to reason about. */
#define PROTO_FRAME_MAX_BYTES   128U
#define PROTO_DECODED_MAX_BYTES 128U

/* Read timeout for assembling a single frame. We don't actually need
 * a deadline here -- the loop just blocks on CDC_Read until the host
 * sends something -- so use osWaitForever via the HAL_MAX_DELAY
 * sentinel that CDC_Read recognizes. */
#define PROTO_RX_BLOCK_FOREVER  HAL_MAX_DELAY

/* CDC_Write timeout, milliseconds. Generous; the existing _CDC_Task
 * echo path uses 100ms with no observed problems. */
#define PROTO_TX_TIMEOUT_MS     200U

static uint8_t s_rxFrame[PROTO_FRAME_MAX_BYTES];
static size_t  s_rxFrameLen;
static uint8_t s_decodedBuf[PROTO_DECODED_MAX_BYTES];
static uint8_t s_responseBuf[PROTO_DECODED_MAX_BYTES];
static uint8_t s_txFrame[PROTO_FRAME_MAX_BYTES];

/* Reset the frame accumulator. Called after every successful frame
 * dispatch and after every overflow drop. */
static void rx_reset(void)
{
  s_rxFrameLen = 0U;
}

/* Append one byte to the frame accumulator. Returns false (and
 * resets the accumulator) on overflow. */
static bool rx_append(uint8_t b)
{
  if (s_rxFrameLen >= PROTO_FRAME_MAX_BYTES)
  {
    rx_reset();
    return false;
  }
  s_rxFrame[s_rxFrameLen++] = b;
  return true;
}

/* Build the GetUptime response. */
static void fill_get_uptime_response(touchforce_v1_Response *resp)
{
  resp->which_payload = touchforce_v1_Response_get_uptime_tag;
  resp->payload.get_uptime.uptime_ms = (uint32_t)HAL_GetTick();
}

/* Build a generic error response. msg may be NULL. */
static void fill_error_response(touchforce_v1_Response *resp,
                                uint32_t code, const char *msg)
{
  resp->which_payload = touchforce_v1_Response_error_tag;
  resp->payload.error.code = code;
  resp->payload.error.message[0] = '\0';
  if (msg != NULL)
  {
    /* The .options file bounds .message at 64 bytes. */
    size_t cap = sizeof(resp->payload.error.message);
    strncpy(resp->payload.error.message, msg, cap - 1U);
    resp->payload.error.message[cap - 1U] = '\0';
  }
}

/* Encode resp into s_responseBuf, COBS-encode that into s_txFrame
 * with a trailing 0x00, and CDC_Write it. */
static void send_response(const touchforce_v1_Response *resp)
{
  pb_ostream_t os = pb_ostream_from_buffer(s_responseBuf,
                                           sizeof(s_responseBuf));
  if (!pb_encode(&os, touchforce_v1_Response_fields, resp))
  {
    /* Encoder ran out of buffer or hit a logic bug. Nothing useful
     * we can transmit; drop. */
    return;
  }

  cobs_encode_result enc = cobs_encode(s_txFrame, sizeof(s_txFrame) - 1U,
                                       s_responseBuf, os.bytes_written);
  if (enc.status != COBS_ENCODE_OK)
  {
    return;
  }
  s_txFrame[enc.out_len] = 0x00U;

  (void)CDC_Write(s_txFrame, (uint16_t)(enc.out_len + 1U),
                  PROTO_TX_TIMEOUT_MS);
}

/* Process a single COBS-delimited frame from s_rxFrame. */
static void handle_frame(void)
{
  if (s_rxFrameLen == 0U)
  {
    return;
  }

  /* COBS-decode in place into s_decodedBuf. */
  cobs_decode_result dec = cobs_decode(s_decodedBuf, sizeof(s_decodedBuf),
                                       s_rxFrame, s_rxFrameLen);
  if (dec.status != COBS_DECODE_OK)
  {
    /* Frame was malformed; we don't echo a Response because we have
     * no request_id to echo into it. Just drop. */
    return;
  }

  touchforce_v1_Request req = touchforce_v1_Request_init_zero;
  pb_istream_t is = pb_istream_from_buffer(s_decodedBuf, dec.out_len);
  if (!pb_decode(&is, touchforce_v1_Request_fields, &req))
  {
    return;
  }

  touchforce_v1_Response resp = touchforce_v1_Response_init_zero;
  resp.request_id = req.request_id;

  switch (req.which_payload)
  {
    case touchforce_v1_Request_get_uptime_tag:
      fill_get_uptime_response(&resp);
      break;
    default:
      /* Unknown command tag (host running newer protocol than MCU).
       * Reply with ErrorResponse so the host can correlate. */
      fill_error_response(&resp, 1U, "unknown command");
      break;
  }

  send_response(&resp);
}

void Protocol_RunForever(void)
{
  rx_reset();

  uint8_t chunk[64];

  for (;;)
  {
    size_t n = CDC_Read(chunk, sizeof(chunk), PROTO_RX_BLOCK_FOREVER);
    if (n == 0U)
    {
      continue;
    }

    for (size_t i = 0U; i < n; i++)
    {
      uint8_t b = chunk[i];
      if (b == 0x00U)
      {
        handle_frame();
        rx_reset();
      }
      else
      {
        (void)rx_append(b);
      }
    }
  }
}
```

- [ ] **Step 3: Confirm the new files compile-syntax against the generated header**

We can't run the firmware build from this plan, but we can sanity-check the references against the generated header from the host:

```bash
grep -E "touchforce_v1_Request_get_uptime_tag|touchforce_v1_Response_get_uptime_tag|touchforce_v1_Response_error_tag|touchforce_v1_Request_fields|touchforce_v1_Response_fields|touchforce_v1_Request_init_zero|touchforce_v1_Response_init_zero" \
  TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h | sort -u
```

Expected: every one of those identifiers appears at least once. If any is missing, the symbol may differ in case or naming — open the header and fix the reference in `protocol_task.c`. (nanopb names oneof tags as `<MessageName>_<oneof_field_name>_tag` and uses the package-qualified prefix when `package` is set.)

- [ ] **Step 4: Commit**

```bash
git add TouchForceIntegrator/Appli/Inc/protocol_task.h \
        TouchForceIntegrator/Appli/Src/protocol_task.c
git commit -m "$(cat <<'EOF'
Add protocol_task: COBS frame I/O + nanopb dispatch

Owns the full request/response loop on the firmware side: drain
bytes from CDC_Read into a frame accumulator, COBS-decode at each
0x00, nanopb-decode as Request, dispatch by which_payload, then
encode/COBS-encode/transmit the Response. Frame buffer sized at
128 bytes which is comfortably oversized for v1 messages.

Unknown command tags are answered with ErrorResponse code 1 so a
newer host running against an older MCU still gets a correlated
response instead of a timeout.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 2.3: Replace `_CDC_Task` body with `Protocol_RunForever`

**Files:**
- Modify: `TouchForceIntegrator/Appli/Src/main.c`

The current `_CDC_Task` in `Appli/Src/main.c:417-457` is the echo demo. Replace its body (after USB enumeration wait) with a call to `Protocol_RunForever`.

- [ ] **Step 1: Add the include at the top of main.c**

Insert `#include "protocol_task.h"` next to the other Appli-local includes near the top of the file. Specifically, add it on its own line immediately after `#include "usbh_hid_wisecoco.h"` (currently line 12).

Use the Edit tool with:
- `old_string`: `#include "usbh_hid_wisecoco.h"\n\n#include <stdio.h>`
- `new_string`: `#include "usbh_hid_wisecoco.h"\n#include "protocol_task.h"\n\n#include <stdio.h>`

- [ ] **Step 2: Replace the echo body of `_CDC_Task`**

Use the Edit tool to replace the function body. `old_string`:

```c
void _CDC_Task(void *argument)
{
  (void)argument;

  /* MX_USB_DEVICE_Init runs from _USB_Task and calls CDC_AppInit before
   * starting the USB stack, so by the time the host has enumerated the
   * device the FreeRTOS objects backing CDC_Read / CDC_Write definitely
   * exist.  hUsbDeviceFS lives in usb_device.c. */
  extern USBD_HandleTypeDef hUsbDeviceFS;

  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    osDelay(100);
  }

  static const char greeting[] = "[CDC ready - type something]\r\n";
  (void)CDC_Write((const uint8_t *)greeting, sizeof(greeting) - 1U, 100U);

  uint8_t  rxBuf[64];
  char     txBuf[80];

  for (;;)
  {
    size_t n = CDC_Read(rxBuf, sizeof(rxBuf), osWaitForever);
    if (n == 0U)
    {
      continue;
    }

    /* Cap the printed payload at what fits after "echo: " and "\r\n". */
    if (n > sizeof(txBuf) - 9U)
    {
      n = sizeof(txBuf) - 9U;
    }
    int len = snprintf(txBuf, sizeof(txBuf), "echo: %.*s\r\n", (int)n, (const char *)rxBuf);
    if (len > 0)
    {
      (void)CDC_Write((const uint8_t *)txBuf, (uint16_t)len, 100U);
    }
  }
}
```

`new_string`:

```c
/* Drives the touchforce.v1 serial protocol over USB CDC. Runs the
 * COBS frame accumulator + nanopb dispatcher forever after the host
 * has enumerated the device. */
void _CDC_Task(void *argument)
{
  (void)argument;

  /* MX_USB_DEVICE_Init runs from _USB_Task and calls CDC_AppInit before
   * starting the USB stack, so by the time the host has enumerated the
   * device the FreeRTOS objects backing CDC_Read / CDC_Write definitely
   * exist.  hUsbDeviceFS lives in usb_device.c. */
  extern USBD_HandleTypeDef hUsbDeviceFS;

  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    osDelay(100);
  }

  Protocol_RunForever();
}
```

- [ ] **Step 3: Verify no stray references to the old echo locals remain**

```bash
grep -nE "txBuf|\\[CDC ready" TouchForceIntegrator/Appli/Src/main.c
```

Expected: no output. (Both belonged only to the old echo body.)

- [ ] **Step 4: Commit**

```bash
git add TouchForceIntegrator/Appli/Src/main.c
git commit -m "$(cat <<'EOF'
Repurpose _CDC_Task from echo demo to protocol dispatcher

The echo body served as a proof-of-life for the CDC stack; now
that the protocol task module exists, _CDC_Task simply waits for
USB enumeration and hands control to Protocol_RunForever.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Phase 3 — Host CLI

The host package layout after Task 1.4 already has `host/touch_force_cli/__init__.py` and the generated `touch_force_pb2.py`. This phase fills in `link.py` (TDD) and `cli.py`.

### Task 3.1: Set up host test harness

**Files:**
- Create: `host/pyproject.toml`
- Create: `host/.python-version`
- Create: `host/tests/__init__.py`
- Create: `host/tests/test_link.py` (added in Task 3.2)
- Create (via `uv`): `host/uv.lock`

The `pyproject.toml` makes `host/` a proper Python project. `uv` reads it, resolves dependencies, and writes `uv.lock` for reproducible installs. Run everything (pytest, the CLI itself) via `uv run` from `host/` so the locked environment is always used.

- [ ] **Step 1: Write `host/pyproject.toml`**

```toml
[project]
name = "touch_force_cli"
version = "0.1.0"
description = "Host CLI for the touchforce.v1 serial protocol"
requires-python = ">=3.10"
dependencies = [
  "protobuf>=4",
  "pyserial>=3.5",
]

[project.scripts]
touch-force-cli = "touch_force_cli.cli:main"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.hatch.build.targets.wheel]
packages = ["touch_force_cli"]

[dependency-groups]
dev = ["pytest>=7"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 2: Pin a Python version**

cwd: `host/`. (Optional but makes the project reproducible across machines.)

```bash
cd host && echo "3.12" > .python-version && cd ..
```

- [ ] **Step 3: Create the empty test package init**

cwd: repo root.

```bash
mkdir -p host/tests
: > host/tests/__init__.py
```

- [ ] **Step 4: Resolve and install with `uv`**

cwd: `host/`.

```bash
cd host && uv sync && cd ..
```

Expected: `uv` creates `host/.venv/`, writes `host/uv.lock`, installs `touch_force_cli` in editable mode plus `protobuf`, `pyserial`, and `pytest` (the dev group is included by default). Final line resembles `Resolved N packages in Xms` followed by `Installed N packages in Yms`.

If `uv` is not yet installed, install it first: `brew install uv` (macOS) or follow [the install instructions](https://docs.astral.sh/uv/getting-started/installation/).

- [ ] **Step 5: Confirm pytest runs and discovers nothing yet**

cwd: `host/`.

```bash
cd host && uv run pytest -v ; cd ..
```

Expected: exit code 5 (no tests collected) — that's the next task. Output includes `no tests ran`.

- [ ] **Step 6: Confirm `host/.venv/` is gitignored**

uv creates `host/.venv/` automatically. Add a `host/.gitignore`:

```
.venv/
__pycache__/
*.egg-info/
```

cwd: repo root.

- [ ] **Step 7: Commit**

```bash
git add host/pyproject.toml host/.python-version host/.gitignore \
        host/tests/__init__.py host/uv.lock
git commit -m "$(cat <<'EOF'
Set up host package metadata, uv, and test scaffolding

pyproject.toml describes the touch_force_cli package; uv manages
the venv and produces a committed uv.lock for reproducible
installs. pytest runs under `uv run pytest` so the locked deps
are always in effect.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 3.2: TDD the framing layer in `link.py`

**Files:**
- Create: `host/touch_force_cli/link.py`
- Create: `host/tests/test_link.py`

We TDD the COBS framing functions first, then the higher-level `Link` class against an in-memory transport. The Link class accepts any object with `write(bytes)` and `read(n)` methods so tests can substitute a fake.

#### Sub-task 3.2.a — COBS encode/decode

- [ ] **Step 1: Write the failing tests for COBS encode/decode**

Create `host/tests/test_link.py` with the following content:

```python
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
```

- [ ] **Step 2: Run the tests to confirm they fail**

cwd: `host/`.

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: `ImportError` or `AttributeError` because `link.py` doesn't exist yet, OR the `cobs_encode`/`cobs_decode`/`FramingError` symbols are missing. This is the expected red state.

- [ ] **Step 3: Implement `link.py` with COBS primitives**

Create `host/touch_force_cli/link.py`:

```python
"""Wire-level helpers for the touchforce.v1 serial protocol.

This module handles three concerns:
  * COBS encode / decode (cobs_encode, cobs_decode)
  * Reading and writing whole COBS frames over a byte transport
    (read_frame, write_frame)
  * A Link wrapper that binds the framing layer to nanopb-compatible
    Request/Response messages and adds request_id correlation
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol


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
```

- [ ] **Step 4: Run the tests to confirm they pass**

cwd: `host/`.

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: all 9 parametrized round-trip cases plus the rejection test pass.

- [ ] **Step 5: Commit**

```bash
git add host/touch_force_cli/link.py host/tests/test_link.py
git commit -m "$(cat <<'EOF'
Add COBS encode/decode primitives with round-trip tests

Pure-Python COBS implementation covering the standard cases plus
the 254-byte run edge case and the zero-in-payload rejection.
Lives in link.py because the framing layer and the higher-level
Link wrapper share the same module.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

#### Sub-task 3.2.b — Frame I/O over a byte transport

- [ ] **Step 1: Add the failing frame-I/O tests**

Append to `host/tests/test_link.py`:

```python
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
```

- [ ] **Step 2: Run to see failure**

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: the three new tests fail with `AttributeError: module 'touch_force_cli.link' has no attribute 'read_frame'` (or similar).

- [ ] **Step 3: Implement frame I/O in `link.py`**

Append to `host/touch_force_cli/link.py`:

```python
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
```

- [ ] **Step 4: Run to confirm green**

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: all tests (5 from sub-task 3.2.a + 3 new) pass.

- [ ] **Step 5: Commit**

```bash
git add host/touch_force_cli/link.py host/tests/test_link.py
git commit -m "$(cat <<'EOF'
Add framed read/write helpers over a byte transport

write_frame/read_frame are the layer between the COBS primitives
and the higher-level request/response Link. They take any object
with read(n)/write(bytes), so tests substitute an in-memory
loopback. read_frame tolerates leading 0x00 bytes so the host
recovers from mid-frame corruption or stale buffer data.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

#### Sub-task 3.2.c — Request/response Link with correlation

- [ ] **Step 1: Add failing tests for the Link class**

Append to `host/tests/test_link.py`:

```python
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
```

- [ ] **Step 2: Run to see failure**

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: three failures with `AttributeError: module 'touch_force_cli.link' has no attribute 'Link'` etc.

- [ ] **Step 3: Implement Link class**

Append to `host/touch_force_cli/link.py`:

```python
# ---------------------------------------------------------------------------
# Link: request/response correlation
# ---------------------------------------------------------------------------

import itertools
from typing import Callable

from touch_force_cli import touch_force_pb2 as _pb


class ProtocolError(RuntimeError):
    """Raised when a wire frame violates the protocol contract."""


@dataclass
class RemoteError(RuntimeError):
    """Raised when the MCU returns ErrorResponse instead of the expected reply."""

    code: int
    message: str

    def __str__(self) -> str:
        return f"remote error code={self.code}: {self.message}"


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
        """Send GetUpdateRequest, return uptime_ms from the response."""
        rid = self._next_id()
        req = _pb.Request()
        req.request_id = rid
        req.get_uptime.SetInParent()  # selects the empty oneof slot
        write_frame(self._t, req.SerializeToString())

        raw = read_frame(self._t)
        if raw is None:
            raise ProtocolError("transport closed before response arrived")

        resp = _pb.Response()
        resp.ParseFromString(raw)

        if resp.request_id != rid:
            raise ProtocolError(
                f"request_id mismatch: sent {rid}, got {resp.request_id}"
            )

        which = resp.WhichOneof("payload")
        if which == "error":
            raise RemoteError(code=resp.error.code, message=resp.error.message)
        if which == "get_uptime":
            return resp.get_uptime.uptime_ms
        raise ProtocolError(f"unexpected response payload: {which!r}")
```

- [ ] **Step 4: Run to confirm green**

```bash
cd host && uv run pytest tests/test_link.py -v && cd ..
```

Expected: all tests pass (8 from earlier + 3 new = 11 total, plus the parametrized COBS cases).

- [ ] **Step 5: Commit**

```bash
git add host/touch_force_cli/link.py host/tests/test_link.py
git commit -m "$(cat <<'EOF'
Add Link with request/response correlation and error propagation

Link wraps the framing layer with nanopb-compatible Request and
Response handling: monotonic request_id assignment, mismatch
detection, and ErrorResponse to RemoteError conversion. The
_next_id hook keeps tests deterministic; production callers get
the default monotonic counter.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

### Task 3.3: Add the CLI entry point

**Files:**
- Create: `host/touch_force_cli/cli.py`
- Modify: `host/touch_force_cli/__init__.py` (no — leave empty)
- Modify: `host/pyproject.toml` (add `__main__` shortcut)
- Create: `host/touch_force_cli/__main__.py`

- [ ] **Step 1: Write `host/touch_force_cli/cli.py`**

```python
"""Command-line interface for the touchforce.v1 protocol.

Usage:

    python -m touch_force_cli get-uptime --port /dev/cu.usbmodem...

The single subcommand opens the serial port, sends a GetUptime
request, and prints the result in milliseconds.
"""

from __future__ import annotations

import argparse
import sys

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
        # argparse with required=True ensures we never reach here.
        raise AssertionError(f"unhandled command: {args.command!r}")


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Write `host/touch_force_cli/__main__.py`**

```python
"""Allow ``python -m touch_force_cli ...`` to dispatch to the CLI."""

from touch_force_cli.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 3: Smoke-test the CLI argument parser without a real device**

cwd: `host/`.

```bash
cd host && uv run python -m touch_force_cli --port /dev/null get-uptime ; cd ..
```

Expected: a non-zero exit and a stderr/stdout error mentioning that the device couldn't be opened. The point is that argparse and module wiring don't blow up before the open. (On macOS/Linux opening `/dev/null` as a serial port will fail with a `serial.SerialException` traceback or similar — that's acceptable and proves the module loads.)

- [ ] **Step 4: Commit**

```bash
git add host/touch_force_cli/cli.py host/touch_force_cli/__main__.py
git commit -m "$(cat <<'EOF'
Add CLI entry point with get-uptime subcommand

`uv run python -m touch_force_cli get-uptime --port <dev>` opens
the CDC-as-serial device, runs Link.get_uptime, prints the
result, and exits non-zero on protocol/remote error so shell
scripts can react. argparse surface is one subcommand for v1.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Phase 4 — Protocol guide doc

### Task 4.1: Write `protocol/README.md`

**Files:**
- Create: `protocol/README.md`

This is the human/agent reference for speaking the protocol. Lives next to the schema.

- [ ] **Step 1: Write `protocol/README.md`**

```markdown
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
  doesn't recognize, e.g. host running a newer protocol version).

## nanopb-specific caveats for host implementors

- `ErrorResponse.message` is **bounded at 64 bytes** on the MCU
  side (`touch_force.options`). The MCU will silently truncate
  longer strings; full-protobuf hosts that send arbitrary lengths
  will succeed at encode time but the string they receive back as
  an echo (if one ever exists) would be truncated.
- Future `bytes` or `string` fields will need similar bounds. When
  adding them to `touch_force.proto`, also add a corresponding
  `<MessageName>.<field> max_size:N` line to `touch_force.options`.

## Worked example: GetUptime exchange

Host sends:

    Request { request_id: 1, get_uptime: GetUptimeRequest{} }
    serialized:  08 01 1A 00            # 4 bytes
    COBS-encoded: 02 08 01 02 1A 01     # actual encoder output may differ
    on wire: <COBS bytes> 00

MCU replies:

    Response { request_id: 1, get_uptime: GetUptimeResponse{ uptime_ms: 12345 } }
    serialized:  08 01 1A 04 08 B9 60   # depends on uptime
    COBS-encoded: ...
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
   `touch_force.options`.
4. Run `./protocol/compile.sh` from the repo root.
5. On the MCU side, add a `case` to the switch in
   `Appli/Src/protocol_task.c::handle_frame` and a fill helper
   (mirroring `fill_get_uptime_response`).
6. On the host side, add a method to `Link` in
   `host/touch_force_cli/link.py`, plus a CLI subcommand if
   appropriate.
7. Update this document's command catalog.
```

- [ ] **Step 2: Commit**

```bash
git add protocol/README.md
git commit -m "$(cat <<'EOF'
Add protocol guide: how to speak touchforce.v1 over USB CDC

Living reference for humans and agents covering wire format,
schema conventions, the v1 command catalog, ErrorResponse
semantics, nanopb caveats relevant to host implementors, and a
step-by-step recipe for adding a new command.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Phase 5 — End-to-end smoke test

### Task 5.1: Build the firmware and run the CLI against it

This task is the only one the user (Kerry) executes manually, since flashing the H7S78-DK requires physical access. The implementer prepares the verification recipe.

- [ ] **Step 1: Implementer — print the verification recipe**

The implementer runs no tools here; the output of this step is a message to the user with the exact procedure below. The implementer must paste this verbatim:

> **End-to-end smoke test**
>
> 1. Open `TouchForceIntegrator/STM32CubeIDE/Appli` in STM32CubeIDE.
> 2. Right-click the project, **Refresh** (F5). The Project Explorer should now show:
>    - `Application/User/protocol_task.c`
>    - `Application/User/proto/touch_force.pb.c`
>    - `Middlewares/nanopb/pb_*.c` (three files)
>    - `Middlewares/cobs-c/cobs.c`
> 3. **Project → Clean...**, then **Build All**.
>    Expected: clean build, no missing-include errors. If you see
>    `pb_decode.h: No such file or directory`, the include path
>    edits in `.cproject` didn't take — verify with
>    `grep "Middlewares/nanopb" TouchForceIntegrator/STM32CubeIDE/Appli/.cproject`
>    (should print 2 lines).
> 4. Flash the resulting `.elf`/`.hex` to the H7S78-DK as you
>    normally do for this project.
> 5. After enumeration, find the CDC device:
>    - macOS: `ls /dev/cu.usbmodem*`
>    - Linux: `ls /dev/ttyACM*`
> 6. From `host/`, with the uv environment synced (Task 3.1
>    Step 4):
>    ```
>    cd host
>    uv run python -m touch_force_cli get-uptime --port /dev/cu.usbmodemXXXX
>    ```
> 7. Expected output: `uptime: <number> ms`, where the number is
>    nonzero and increases by roughly the elapsed wall time when
>    you run the command twice in succession.
> 8. If it hangs at `Link.get_uptime`: the MCU did not respond.
>    Check that the device is actually enumerated (`lsusb` /
>    macOS System Information), and that the CDC interface is
>    `Configured` (`hUsbDeviceFS.dev_state` in the firmware).
> 9. If `protocol error: request_id mismatch`: the MCU echoed a
>    different ID than the host sent. Most likely cause: stale
>    bytes in the receive buffer from a prior session. Reopen the
>    port (the CLI does this on each invocation) — if it persists,
>    add a `link.read_frame` call in a loop discarding frames with
>    unexpected IDs. (Not in v1; flag for a follow-up.)

- [ ] **Step 2: User runs the recipe and reports results**

(Manual step — implementer waits for user feedback.)

- [ ] **Step 3: Merge the branch**

After the user confirms `uptime: <ms>` prints correctly, on the user's command:

```bash
git checkout main
git merge --no-ff protocol-v1
```

(Do not push without explicit instruction.)

---

## Self-review checklist

Run through this once after writing the plan, fix issues inline, and only then hand off.

- **Spec coverage:** Every numbered section of `2026-05-06-host-mcu-protocol-design.md` has a corresponding task above? Yes —
  - §3 wire format → Tasks 1.1, 2.2, 3.2
  - §4 schema conventions → Task 1.2
  - §5 file layout → Tasks 1.3, 1.4, 2.1, 3.1
  - §6 compile.sh → Task 1.3
  - §7 documentation → Task 4.1
  - §8 deferred decisions: confirmed deferred, no task needed
- **Placeholder scan:** No "TBD", "implement later", or vague step. Each step has either a code block, an exact command with expected output, or a tightly-scoped manual instruction.
- **Type consistency:**
  - C symbol: `touchforce_v1_Request`, `touchforce_v1_Response` consistently (nanopb naming for `package touchforce.v1`).
  - C oneof tags: `touchforce_v1_Request_get_uptime_tag`, `touchforce_v1_Response_get_uptime_tag`, `touchforce_v1_Response_error_tag`.
  - C field-array: `touchforce_v1_Request_fields`, `touchforce_v1_Response_fields`.
  - C init macros: `touchforce_v1_Request_init_zero`, `touchforce_v1_Response_init_zero`.
  - Python module: `touch_force_cli.touch_force_pb2` (the file `protoc` produces from `touch_force.proto`).
  - Function: `Protocol_RunForever` consistent across header and call site.
- **Scope:** Single feature branch, single PR-worthy unit, end-to-end working. No orthogonal refactors snuck in.
