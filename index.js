/**
 * zaltz — a Strudel-compatible audio engine in one file of C, rendering in an
 * AudioWorklet. This module is the thin browser host: it boots the wasm inside
 * the worklet and speaks its message protocol. Pattern languages, samplers and
 * schedulers live a layer above (see https://github.com/eliyahuleinkram/klappn
 * for the reference integration — Klappn's Strudel bridge).
 *
 * Copyright (C) 2026 Eliyahu Moshe Leinkram. The engine is a superdough
 * derivative (AGPL-3.0-or-later) — see NOTICE.md.
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
// superdough's own spellings → what the engine reads. The engine speaks numbers;
// these are the few controls whose Strudel values are names or strings.
const NOTE_SEM = { c: 0, d: 2, e: 4, f: 5, g: 7, a: 9, b: 11 };
const NOTE_ACC = { "#": 1, s: 1, b: -1, f: -1 };
/** superdough util.mjs noteToMidi: "bf3" → 58, octave defaults to 3. */
function noteToMidi(n) {
  const m = /^([a-gA-G])([#bsf]*)(-?[0-9]*)$/.exec(n);
  if (!m) return null;
  let sem = NOTE_SEM[m[1].toLowerCase()];
  for (const a of m[2]) sem += NOTE_ACC[a];
  return ((m[3] ? Number(m[3]) : 3) + 1) * 12 + sem;
}
const VOWELS = ["a", "e", "i", "o", "u", "ae", "aa", "oe", "ue", "y", "uh", "un", "en", "an", "on"];
const VOWEL_ALIAS = { "æ": "ae", "ø": "oe", "ɑ": "aa", "å": "aa", "ö": "oe", "ü": "ue", "ı": "y" };
const FM_WAVES = ["sine", "square", "sawtooth", "triangle", "white", "pink", "brown", "crackle"];
const LFO_SHAPES = { tri: 0, triangle: 0, sine: 1, ramp: 2, saw: 3, square: 4 };

/** Params in Strudel's spellings → the engine's. Returns null for a param set
 *  superdough itself would refuse (an unknown vowel or note throws there). */
function normalize(params) {
  const p = { ...params };
  if (typeof p.note === "string") {
    const m = noteToMidi(p.note);
    if (m == null) return null;
    p.note = m;
  }
  if (typeof p.vowel === "string") {
    const i = VOWELS.indexOf(VOWEL_ALIAS[p.vowel] ?? p.vowel);
    if (i < 0) return null;
    p.vowel = i;
  }
  if (typeof p.fmwave === "string") p.fmwave = Math.max(0, FM_WAVES.indexOf(p.fmwave));
  if (typeof p.fmenv === "string") p.fmenv = p.fmenv === "lin" ? 1 : 0;
  if (typeof p.tremoloshape === "string") p.tremoloshape = LFO_SHAPES[p.tremoloshape] ?? 0;
  const s = p.s === "saw" ? "sawtooth" : p.s === "tri" ? "triangle" : p.s;
  // getFrequencyFromValue (synths): `note || 36`, a truthy freq wins, ×2^octave
  if (s && s !== "sample") {
    const oct = typeof p.octave === "number" ? p.octave : 0;
    if (p.freq) p.freq = p.freq * Math.pow(2, oct);
    else p.note = (p.note || 36) + 12 * oct;
    delete p.octave;
  }
  // getOscillator: `partials ?? n` on saw/square/triangle; the supersaw's detune ?? n
  if (typeof p.n === "number" && p.n) {
    if ((s === "sawtooth" || s === "square" || s === "triangle") && p.partials == null) p.partials = p.n;
    if (s === "supersaw" && p.detune == null) p.detune = p.n;
  }
  // synth.mjs 'pulse': a rate without a sweep sweeps 0.3
  if (s === "pulse" && p.pwrate != null && p.pwsweep == null) p.pwsweep = 0.3;
  return p;
}

function kv(params) {
  const norm = normalize(params);
  if (!norm) return null;
  const parts = [];
  for (const [k, v] of Object.entries(norm)) {
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
    this.onscrub = null;
    this.onstems = null;
    this._stemEnd = null;
    node.port.onmessage = (e) => {
      const d = e.data;
      if (d?.error && this.onerror) this.onerror(String(d.error));
      else if (d?.scrubbed != null && this.onscrub) this.onscrub(d.scrubbed);
      else if (d?.clock != null && this.onclock) this.onclock(d.clock);
      else if (d?.stemBatch && this.onstems) this.onstems(d);
      else if (d?.stemEnd && this._stemEnd) {
        const done = this._stemEnd;
        this._stemEnd = null;
        done();
      }
    };
    // A wasm trap kills the processor with no port message — without this
    // hook the engine dies silently and every later schedule() is a no-op.
    node.onprocessorerror = () => {
      if (this.onerror) this.onerror("zaltz: audio processor crashed — recreate the engine");
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
   * Synth sources: sine | sawtooth | square | triangle | supersaw | pulse |
   * white | pink | brown | crackle. Sample playback: s:"sample" with a
   * `sample` id previously uploaded via loadSample(). Strudel's own spellings
   * work — note:"bf3", vowel:"a", fmwave:"square" — and a param set superdough
   * would refuse (an unknown note or vowel) is skipped, as it is upstream.
   */
  schedule(params, when) {
    const ev = kv(params);
    if (ev) this.node.port.postMessage({ ev, t: when });
  }

  /** Schedule a batch in ONE message (kinder to the audio thread's GC). */
  scheduleAll(events) {
    const evs = [];
    for (const { params, when } of events) {
      const ev = kv(params);
      if (ev) evs.push({ ev, t: when });
    }
    if (evs.length) this.node.port.postMessage({ evs });
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
   * THE STEM TAP — live track separation, taken at the engine's own mix
   * point: while armed, every used orbit's post-FX stereo block (delay,
   * reverb, duck and kill included — exactly what that orbit contributes to
   * the master) streams back through `onstems` in ~85 ms batches. Each batch
   * is { stemBatch, orbits, quanta, startFrame, slotFloats }: slot k holds
   * orbit `orbits[k]`, interleaved stereo, at
   * stemBatch[k*slotFloats … k*slotFloats + quanta*256). `startFrame` is the
   * context's own sample clock, so batches align sample-exactly across
   * orbits — and against anything else you capture on this context. Hand
   * each batch's buffer back with recycleStemBatch() and the tap allocates
   * nothing in steady state. Disarming flushes the tail, then `done` fires.
   */
  stems(on, done) {
    if (on) this.node.port.postMessage({ stemsOn: true });
    else {
      this._stemEnd = done ?? null;
      this.node.port.postMessage({ stemsOn: false });
    }
  }

  /** Return a spent stem batch buffer to the worklet's pool (zero-alloc tap). */
  recycleStemBatch(stemBatch) {
    this.node.port.postMessage({ stemRecycle: stemBatch.buffer }, [stemBatch.buffer]);
  }

  /**
   * Upload PCM for sample id `id` (interleaved if stereo). Chunked so the
   * copy never lands on the audio thread in one burst; the engine keeps the
   * sample on HOLD until the last chunk arrives (a half-copied buffer must
   * never sound). Ids are never freed — the store grows with what you load.
   */
  loadSample(id, pcm, { frames, channels = 1 } = {}) {
    const n = frames ?? Math.floor(pcm.length / channels);
    // Ship exactly the allocated floats — an explicit `frames` smaller than
    // the buffer must truncate the upload, never overrun the arena slot.
    const total = Math.min(pcm.length, n * channels);
    this.node.port.postMessage({ sampleAlloc: id, frames: n, channels });
    const CHUNK = 65536;
    for (let off = 0; off < total; off += CHUNK) {
      const slice = pcm.slice(off, Math.min(off + CHUNK, total));
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
