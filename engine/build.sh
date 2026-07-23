#!/usr/bin/env bash
# zaltz → wasm32. Zig binary tarball (ziglang.org) — no LLVM install.
set -e
cd "$(dirname "$0")"
# Resolution order: $ZIG override → zig on PATH → the maintainer's tarball
# install (~/.cache, not /tmp — macOS periodic cleanup gutted /tmp once).
# No zig? Grab the 0.16 tarball for your OS from ziglang.org and run
# `ZIG=/path/to/zig ./build.sh`.
ZIG="${ZIG:-$(command -v zig || echo "$HOME/.cache/zig-aarch64-macos-0.16.0/zig")}"
if ! command -v "$ZIG" >/dev/null; then
  echo "zig not found — install from ziglang.org and run: ZIG=/path/to/zig ./build.sh" >&2
  exit 1
fi
# NO -msimd128: Safari only ships wasm SIMD from 16.4 — the flag silently
# bricked the engine on 16.0–16.3 (DSP is scalar C; nothing needed it).
# bulk-memory stays (Safari 15+); older still gets music via the bridge's
# superdough boot-failure fallback (lib/zaltz.ts bootFailed).
# -fdebug-prefix-map: keep the builder's home directory out of the shipped
# wasm's debug strings (reproducible across machines).
$ZIG cc \
  -O2 -Wall -std=gnu99 \
  --target=wasm32-freestanding \
  -mbulk-memory \
  -nostdlib \
  -Wl,--no-entry \
  -fdebug-prefix-map="$(cd .. && pwd)"=zaltz \
  -fdebug-prefix-map="$HOME"=~ \
  -o zaltz.wasm zaltz.c
echo -n "zaltz.wasm: " && wc -c < zaltz.wasm
cp zaltz.wasm ../dist/zaltz.wasm
echo "copied → dist/zaltz.wasm"
