/**
 * zaltz — a Strudel-compatible audio engine in one file of C, rendering in an
 * AudioWorklet. This module is the thin browser host: it boots the wasm inside
 * the worklet and speaks its message protocol. Pattern languages, samplers and
 * schedulers live a layer above (see https://github.com/eliyahuleinkram/klappn
 * for the reference integration — Klappn's Strudel bridge).
 *
 * The engine is a superdough derivative (AGPL-3.0-or-later) — see NOTICE.md.
 */

/** Serialize a finite number without scientific notation (the engine's parser
 *  reads plain decimals; "3e-9" must never arrive on the wire). */
function fnum(x) {
  if (!Number.isFinite(x)) return "0";
  const s = String(x);
  if (!s.includes("e") && !s.includes("E")) return s;
  if (Math.abs(x) >= 1e15) x = Math.sign(x) * 1e15;
  const fixed = x.toFixed(12);
  const trimmed = fixed.includes(".") ? fixed.replace(/0+$/, "").replace(/\.$/, "") : fixed;
  return trimmed === "" || trimmed === "-" ? "0" : trimmed;
}

/** Flatten an event's params into the engine's `key/value/…` wire string. */
function kv(params) {
  const parts = [];
  for (const [k, v] of Object.entries(params)) {
    if (v == null) continue;
    parts.push(k, typeof v === "number" ? fnum(v) : String(v));
  }
  return parts.join("/");
}

export class Zaltz {
  /** @private — use Zaltz.create() */
  constructor(ctx, node) {
    this.ctx = ctx;
    this.node = node;
    this.onerror = null;
    this.onclock = null;
    node.port.onmessage = (e) => {
      const d = e.data;
      if (d?.error && this.onerror) this.onerror(String(d.error));
      else if (d?.clock != null && this.onclock) this.onclock(d.clock);
    };
  }

  /**
   * Boot the engine on `ctx`. Loads the worklet module and the wasm (both
   * resolved relative to this package by default), instantiates it INSIDE the
   * worklet — the engine owns its memory; no SharedArrayBuffer, no
   * cross-origin-isolation requirements.
   *
   * @param {AudioContext} ctx
   * @param {{ workletUrl?: string|URL, wasmUrl?: string|URL, destination?: AudioNode }} [opts]
   * @returns {Promise<Zaltz>}
   */
  static async create(ctx, opts = {}) {
    const workletUrl = opts.workletUrl ?? new URL("./dist/zaltz.worklet.js", import.meta.url);
    const wasmUrl = opts.wasmUrl ?? new URL("./dist/zaltz.wasm", import.meta.url);
    await ctx.audioWorklet.addModule(workletUrl);
    const wasm = await fetch(wasmUrl).then((r) => {
      if (!r.ok) throw new Error(`zaltz: wasm fetch failed (${r.status})`);
      return r.arrayBuffer();
    });
    const node = new AudioWorkletNode(ctx, "zaltz", { outputChannelCount: [2] });
    node.connect(opts.destination ?? ctx.destination);
    await new Promise((resolve, reject) => {
      const to = setTimeout(() => reject(new Error("zaltz: worklet never became ready")), 8000);
      node.port.onmessage = (e) => {
        if (e.data?.ready) { clearTimeout(to); resolve(); }
        else if (e.data?.error) { clearTimeout(to); reject(new Error(e.data.error)); }
      };
      node.port.postMessage({ wasm });
    });
    return new Zaltz(ctx, node);
  }

  /**
   * Schedule one voice. `when` is absolute AudioContext time (seconds);
   * `params` mirrors superdough's control names — s, note/freq, duration,
   * gain, attack/decay/sustain/release, lpf/lpq/hpf/hpq, room/roomsize,
   * delay/delaytime/delayfeedback, shape, orbit, pan, speed, duck…
   * Synth sources: sine | sawtooth | square | triangle | supersaw |
   * white | pink | brown | crackle. Sample playback: s:"sample" with a
   * `sample` id previously uploaded via loadSample().
   */
  schedule(params, when) {
    this.node.port.postMessage({ ev: kv(params), t: when });
  }

  /** Schedule a batch in ONE message (kinder to the audio thread's GC). */
  scheduleAll(events) {
    this.node.port.postMessage({
      evs: events.map(({ params, when }) => ({ ev: kv(params), t: when })),
    });
  }

  /** Silence: voices, pending events and bus states clear; samples survive. */
  hush() {
    this.node.port.postMessage({ hush: true });
  }

  /** Crossfade takeover: fade everything currently sounding over `seconds`,
   *  leaving anything scheduled after this call untouched. */
  retire(seconds = 1.2) {
    this.node.port.postMessage({ retire: seconds });
  }

  /** Glided per-orbit output gain (0..2) — the "channel kill" primitive. */
  setOrbitGain(orbit, gain) {
    this.node.port.postMessage({ orbitGains: [{ o: orbit, g: gain }] });
  }

  /**
   * Upload PCM for sample id `id` (interleaved if stereo). Chunked so the
   * copy never lands on the audio thread in one burst; the engine keeps the
   * sample on HOLD until the last chunk arrives (a half-copied buffer must
   * never sound). Ids are never freed — the store grows with what you load.
   */
  loadSample(id, pcm, { frames, channels = 1 } = {}) {
    const n = frames ?? Math.floor(pcm.length / channels);
    this.node.port.postMessage({ sampleAlloc: id, frames: n, channels });
    const CHUNK = 65536;
    for (let off = 0; off < pcm.length; off += CHUNK) {
      const slice = pcm.slice(off, Math.min(off + CHUNK, pcm.length));
      this.node.port.postMessage({ sampleChunk: id, offset: off, pcm: slice.buffer }, [slice.buffer]);
    }
  }

  /** Convenience: decode + upload an AudioBuffer (downmixes to ≤2 channels). */
  loadAudioBuffer(id, buf) {
    const ch = Math.min(2, buf.numberOfChannels);
    const pcm = new Float32Array(buf.length * ch);
    for (let c = 0; c < ch; c++) {
      const data = buf.getChannelData(c); // engines may hand back a GC-mutable view — copy out
      for (let i = 0; i < buf.length; i++) pcm[i * ch + c] = data[i];
    }
    this.loadSample(id, pcm, { frames: buf.length, channels: ch });
  }

  /** Detach from the graph. The worklet stops rendering on GC. */
  dispose() {
    this.node.port.postMessage("stop");
    try { this.node.disconnect(); } catch { /* already detached */ }
  }
}

export default Zaltz;
