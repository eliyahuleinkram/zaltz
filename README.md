<img src="zaltz-icon.svg" alt="" width="88" align="left" />

# zaltz

**The whole synthesizer is one file of C. It compiles to 165 KB of wasm, lives
on the audio thread, and does not glitch. Ever.**

Browser audio engines synthesize on the main thread, and the main thread has
other plans — one garbage-collection pause, one fat framework commit, one
backgrounded tab, and the music cracks. Every fix is an apology: bigger
buffers, simplified mixes for phones, reverb rationing on mobile.

zaltz deletes the problem instead of managing it. Oscillators, filters,
envelopes, the sampler, FDN reverb, delay lines, sidechain compression — all
of it is [one file of freestanding C](engine/zaltz.c), no libc, no
SharedArrayBuffer, no cross-origin headers, rendering inside an AudioWorklet
where your UI thread physically cannot reach it. The page can stutter, sleep,
or die mid-animation. The music does not.

**Hear it: [zaltz.klappn.com](https://zaltz.klappn.com)** — type a pattern,
it renders live.

## Born in production, not in a demo

zaltz is the engine under every song at [Klappn](https://klappn.com). It
replaced [superdough](https://github.com/tidalcycles/strudel) — Strudel's
sampler/synth layer — after being measured against it, case by case, until a
golden-gate harness couldn't tell them apart: envelope correlation, level,
brightness, per feature, against superdough's own offline render. Then an
8-song, 70-loop corpus of real music was A/B'd by ear before it became the
default.

The day it shipped, Klappn deleted its entire mobile apology layer: the
AI-generated "lite" rewrites of every song for phones, the reverb caps, the
per-device mixes. Phones now play the same full mix as desktops, because the
render doesn't live where phones are slow.

## What's in the box

- [`engine/zaltz.c`](engine/zaltz.c) — ~1,600 lines. Band-limited wavetable
  oscillators (triangle at sample-exact 90° phase — a phase-blind test suite
  hid that bug once, never again), four noise colors, ADSR + filter envelopes,
  ladder/12dB/24dB filters, a growable sample store, looping soundfont voices,
  and per-orbit buses: FDN reverb, delay, sidechain duck, phaser, waveshaping.
- [`dist/zaltz.worklet.js`](dist/zaltz.worklet.js) — the AudioWorklet host.
  Events are written straight into engine memory (zero allocation per event on
  the audio thread — worklet GC pauses are audible, so there are none), sample
  uploads drain under a strict per-quantum budget, and an audio-thread clock
  posts outward so scheduling survives background-tab timer throttling.
- [`index.js`](index.js) — a thin, typed host API: boot, schedule, hush,
  crossfade-retire, orbit gains, sample upload.

```js
import Zaltz from "zaltz";

const ctx = new AudioContext();
const z = await Zaltz.create(ctx);

// A bar of dub: bass on the beat, stabs on the off-beat ringing
// through a synced delay into the FDN reverb on orbit 2.
const t0 = ctx.currentTime + 0.1;
const spb = 60 / 122; // seconds per beat
const bass = [33, 33, 36, 31]; // A1 A1 C2 G1
z.scheduleAll(bass.flatMap((note, beat) => [
  { params: { s: "sawtooth", note, duration: spb * 0.9, attack: 0.005,
      release: 0.1, lpf: 500, gain: 0.8 },
    when: t0 + beat * spb },
  { params: { s: "sawtooth", note: note + 24, duration: 0.15, release: 0.12,
      lpf: 1500, room: 0.45, roomsize: 6, delay: 0.4, delaytime: spb * 0.75,
      delayfeedback: 0.5, orbit: 2, gain: 0.5 },
    when: t0 + (beat + 0.5) * spb },
]));
```

Params speak superdough's language — `s`, `note`, ADSR, `lpf`, `room`,
`delay`, `shape`, `orbit`, `duck` — so anything that already talks to Strudel
can learn to talk to zaltz in an afternoon. The pattern layer stays upstream;
this package is the sound.

## The laws

Every rule in the engine was paid for with a click somebody heard:

- **Buses ramp, never step** — including feedback-network coefficients. A
  retuned reverb glides its damping and size *while the tail rings*. Stepping
  a live coefficient is a click by construction, so it is impossible by
  construction.
- **Nothing unbounded on the audio thread.** Event writes are length-guarded,
  uploads budgeted per render quantum, memory growth chunked and rare.
- **Parity is the floor, not the ceiling.** Where the reference architecture
  glitches — a shared convolver regenerated mid-ring, per-event GC churn —
  zaltz deliberately diverges.
- **The ear is the acceptance test.** Metrics gate regressions. A human
  listening decides quality.

## Runs everywhere

No SIMD (Safari 15 runs it natively), no SharedArrayBuffer (no COOP/COEP
headers to negotiate), no main-thread synthesis (phones keep up). Memory
starts at a few MB and grows only with what a song actually loads. Rebuild
from source with `npm run build` — one `zig cc` invocation, no toolchain
ceremony.

## License

AGPL-3.0-or-later — zaltz is a derivative of superdough (the
[Strudel](https://strudel.cc) project), and it honors its ancestor's terms;
see [NOTICE.md](NOTICE.md).
