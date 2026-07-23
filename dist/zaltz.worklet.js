// ZALTZ's AudioWorklet host — the audio-thread half of the engine.
// The wasm lives ENTIRELY in here (worklet-owned memory, exported by the
// module): no SharedArrayBuffer, no COOP/COEP, every AudioWorklet browser.
// Main-thread jank can only delay message ARRIVAL (absorbed by the bridge's
// lookahead) — it can never glitch the render loop below.
//
// Messages in:
//   { wasm: ArrayBuffer }                     → instantiate + sd_init(sampleRate)
//   { sampleId, frames, channels, pcm }       → sd_sample_alloc + copy (pcm: Float32Array)
//   { ev: "key/value/…", t: <ctx seconds> }   → schedule (time made engine-relative here)
//   { hush: true }                            → sd_init re-init: voices/events cleared,
//                                                sample arena KEPT (instant panic)
//   "stop"                                    → processor retires
/* global AudioWorkletProcessor, registerProcessor, sampleRate, currentTime */

// AudioWorkletGlobalScope has NO TextEncoder/TextDecoder — events are
// pure-ASCII key/value strings written CHAR-BY-CHAR into wasm memory
// (pushEvent) so nothing allocates on the audio thread.

class ZaltzProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.active = true;
    this.ex = null;
    this.memU8 = null; // cached views — NOTHING allocates per event/quantum
    this.memF32 = null;
    this.scrubbed = 0; // lifetime non-finite output samples zeroed
    this.scrubLast = 0; // scrubbed count at the last report — report only growth
    this.scrubReported = 0; // blocks stamp of the last scrub report
    this.port.onmessage = (e) => {
      try {
        this.handle(e.data);
      } catch (err) {
        // a dead message must be LOUD — the bridge surfaces it and play fails
        // into the normal error path instead of hanging
        this.port.postMessage({ error: String(err) });
      }
    };
  }

  handle(d) {
    {
      if (d === "stop") {
        this.active = false;
        return;
      }
      if (d.wasm) {
        WebAssembly.instantiate(d.wasm, {}).then(
          ({ instance }) => {
            this.ex = instance.exports;
            this.ex.sd_init(sampleRate);
            // cached views: rebuilt ONLY when the sample store memory.grow's
            // (see syncViews) — per-quantum churn stays zero
            this.syncViews();
            this.port.postMessage({ ready: true, sampleRate });
          },
          (err) => this.port.postMessage({ error: "wasm: " + String(err) }),
        );
        return;
      }
      if (!this.ex) return;
      if (d.hush) {
        // sd_hush, NOT sd_init: init rebuilds the wavetable bank (~2.5M sinf)
        // and starves the render thread mid-playback — hush must be near-free
        this.ex.sd_hush(); // voices/events/buses cleared; samples + tables survive
        return;
      }
      if (d.sampleAlloc != null) {
        // PACED uploads: chunk messages arrive in BURSTS (a section's zones
        // all load at once) and the worklet drains its whole message queue
        // between render quanta — copying megabytes there IS the tick the
        // ear catches. So: alloc + HOLD here, queue the chunks, and copy a
        // BOUNDED slice per process() below; sd_sample_ready fires when the
        // last float lands (held samples are skipped by the engine, never
        // half-played).
        const ptr = this.ex.sd_sample_alloc(d.sampleAlloc, d.frames, d.channels);
        this.syncViews(); // alloc may memory.grow — old views are detached
        this.samplePtrs = this.samplePtrs || {};
        this.samplePtrs[d.sampleAlloc] = ptr || 0;
        this.sampleLeft = this.sampleLeft || {};
        this.sampleLeft[d.sampleAlloc] = d.frames * d.channels;
        if (ptr) this.ex.sd_sample_hold(d.sampleAlloc);
        else this.port.postMessage({ error: "sample store grow failed (OOM)" });
        return;
      }
      if (d.sampleChunk != null) {
        if (!this.samplePtrs?.[d.sampleChunk]) return;
        this.copyQueue = this.copyQueue || [];
        this.copyQueue.push({ id: d.sampleChunk, offset: d.offset, pcm: new Float32Array(d.pcm), pos: 0 });
        return;
      }
      if (d.retire != null) {
        // CROSSFADE TAKEOVER: fade the old music over d.retire seconds; the
        // new loop's events (posted after this) are untouched
        this.ex.sd_retire(d.retire);
        return;
      }
      if (d.orbitGains) {
        // deck channel kills: [{o, g}, …] → engine-side glided output gains
        for (const x of d.orbitGains) this.ex.sd_orbit_gain(x.o, x.g);
        return;
      }
      if (d.evs) {
        // BATCHED: one message per bridge tick (Safari's worklet GC chokes on
        // per-hap message bursts — its pauses are the Safari-only ticks)
        for (const e of d.evs) this.pushEvent(e.ev, e.t);
        return;
      }
      if (d.ev) this.pushEvent(d.ev, d.t);
    }
  }

  syncViews() {
    // one identity compare; views rebuild only on an actual grow, so the
    // audio thread still allocates nothing quantum-to-quantum
    const b = this.ex.memory.buffer;
    if (b !== this.memBuf) {
      this.memBuf = b;
      this.memU8 = new Uint8Array(b);
      this.memF32 = new Float32Array(b);
    }
  }

  pushEvent(ev, t) {
    // chars go STRAIGHT into the wasm event buffer — no intermediate
    // Uint8Array per hap (audio-thread allocations feed WebKit's GC)
    const rel = Math.max(0, (t ?? 0) - currentTime);
    // toFixed: a bare `rel` can stringify as "3e-9", which the engine's
    // exponent-naive parser once read as 3 SECONDS; 10µs precision is finer
    // than a sample
    const head = "time/" + rel.toFixed(5) + "/";
    if (head.length + ev.length >= 2047) {
      // engine event buffer is 2048 bytes — never write past it
      this.port.postMessage({ eventError: -99 });
      return;
    }
    const base = this.ex.sd_event_ptr();
    const m = this.memU8;
    let w = base;
    for (let i = 0; i < head.length; i++) m[w++] = head.charCodeAt(i) & 0xff;
    for (let i = 0; i < ev.length; i++) m[w++] = ev.charCodeAt(i) & 0xff;
    m[w] = 0;
    const rc = this.ex.sd_event();
    if (rc !== 0) this.port.postMessage({ eventError: rc });
  }

  drainCopies() {
    // ≤16384 floats (~64KB, well under 100µs) per quantum — upload work can
    // never blow the render budget BY CONSTRUCTION, no matter the burst
    let budget = 16384;
    const q = this.copyQueue;
    const f = this.memF32;
    while (q && q.length && budget > 0) {
      const c = q[0];
      const ptr = this.samplePtrs[c.id];
      if (!ptr) { q.shift(); continue; }
      const n = Math.min(budget, c.pcm.length - c.pos);
      const dst = (ptr >> 2) + c.offset + c.pos;
      const src = c.pcm;
      const pos = c.pos;
      for (let i = 0; i < n; i++) f[dst + i] = src[pos + i]; // cached view, zero alloc
      c.pos += n;
      budget -= n;
      this.sampleLeft[c.id] -= n;
      if (c.pos >= c.pcm.length) q.shift();
      if (this.sampleLeft[c.id] <= 0) this.ex.sd_sample_ready(c.id);
    }
  }

  process(_inputs, outputs) {
    const out = outputs[0];
    if (this.ex && out && out[0]) {
      // AUDIO-THREAD CLOCK → bridge (~every 85ms): hidden tabs throttle the
      // bridge's setInterval to ≥1s and starve the lookahead; port messages
      // from a running worklet are never throttled, so scheduling survives
      // backgrounding on every browser.
      this.blocks = (this.blocks | 0) + 1;
      if (this.blocks % 32 === 0) this.port.postMessage({ clock: currentTime });
      this.syncViews(); // rebuilds only if the sample store grew
      this.drainCopies();
      this.ex.sd_dsp();
      // sd_dsp itself can memory.grow (a new orbit's first delay line takes
      // from the arena mid-render) — the view captured above is DETACHED then,
      // and copying through it wrote one full quantum of NaN to the output:
      // the audible click at a new song's first delay beat. Re-sync AFTER the
      // render; it's one identity compare when nothing grew.
      this.syncViews();
      // CACHED view — a fresh Float32Array per quantum (375/s) feeds the
      // worklet-thread GC, and its pauses are audible ticks. syncViews
      // rebuilds it only on an actual memory.grow.
      const f = this.memF32;
      const o = this.ex.sd_out_ptr() >> 2;
      const L = out[0], R = out[1] ?? out[0];
      // NON-FINITE SCRUB: the master chain runs through
      // DynamicsCompressorNodes, and a single NaN OR Infinity sample poisons
      // a compressor's envelope PERMANENTLY (Chromium/WebKit) — the whole mix
      // then stays silent until reload ("the set just turned off"). Whatever
      // ever goes wrong upstream, it must reach the graph as at worst a
      // click, never as a non-finite sample. Scrubbed samples are counted and
      // reported (rate-limited) so real engine corruption stays diagnosable
      // instead of degrading to indistinguishable silence.
      for (let i = 0; i < L.length; i++) {
        const l = f[o + i * 2], r = f[o + i * 2 + 1];
        if (Number.isFinite(l)) L[i] = l;
        else { L[i] = 0; this.scrubbed++; }
        if (Number.isFinite(r)) R[i] = r;
        else { R[i] = 0; this.scrubbed++; }
      }
      if (this.scrubbed > this.scrubLast && this.blocks - this.scrubReported >= 375) {
        this.scrubReported = this.blocks; // ~1s between reports
        this.scrubLast = this.scrubbed; // quiet again until NEW scrubs land
        this.port.postMessage({ scrubbed: this.scrubbed });
      }
    }
    return this.active;
  }
}

registerProcessor("zaltz", ZaltzProcessor);
