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

# --- Guard: every variable-length field must be bounded -------------------
# nanopb emits pb_callback_t for unbounded string/bytes/repeated fields.
# That forces the caller to write encode/decode callbacks and breaks the
# fixed-size struct invariant the firmware relies on. Fail loudly.
if grep -q "pb_callback_t" ../TouchForceIntegrator/Appli/Inc/proto/touch_force.pb.h; then
  echo "error: generated header contains pb_callback_t" >&2
  echo "       a variable-length field is missing a bound in touch_force.options" >&2
  exit 1
fi

echo "==> done"
