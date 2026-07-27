/** Superdough-style control names, plus anything numeric the engine knows. */
export type ZaltzParams = {
  /** Sound source: sine | sawtooth | square | triangle | supersaw | white |
   *  pink | brown | crackle — or "sample" (with `sample` set to a loaded id). */
  s?: string;
  note?: number;
  freq?: number;
  /** Uploaded sample id (see loadSample / loadAudioBuffer). */
  sample?: number;
  duration?: number;
  gain?: number;
  velocity?: number;
  attack?: number;
  decay?: number;
  sustain?: number;
  release?: number;
  lpf?: number;
  lpq?: number;
  hpf?: number;
  hpq?: number;
  ftype?: "ladder" | "12db" | "24db";
  room?: number;
  roomsize?: number;
  delay?: number;
  delaytime?: number;
  delayfeedback?: number;
  shape?: number;
  crush?: number;
  coarse?: number;
  pan?: number;
  speed?: number;
  loop?: number;
  loopBegin?: number;
  loopEnd?: number;
  orbit?: number;
  cut?: number;
  /** Sidechain-duck target orbit list, e.g. "2:3". */
  duck?: string;
  duckdepth?: number;
  duckattack?: number;
  duckonset?: number;
  phaserrate?: number;
  phaserdepth?: number;
  phasercenter?: number;
  phasersweep?: number;
  [key: string]: string | number | undefined;
};

/** One ~85 ms batch from the stem tap (see Zaltz.stems). Slot k holds orbit
 *  `orbits[k]` as interleaved stereo at
 *  `stemBatch[k*slotFloats … k*slotFloats + quanta*256)`. */
export interface ZaltzStemBatch {
  stemBatch: Float32Array;
  /** Engine orbit index per occupied slot, in slot order. */
  orbits: number[];
  /** 128-frame render quanta in this batch. */
  quanta: number;
  /** Context sample clock at the batch's first frame — sample-exact alignment. */
  startFrame: number;
  /** Stride between slots in `stemBatch` (floats). */
  slotFloats: number;
}

export interface ZaltzCreateOptions {
  /** Override the worklet module URL (defaults to the packaged file). */
  workletUrl?: string | URL;
  /** Override the wasm URL (defaults to the packaged binary). */
  wasmUrl?: string | URL;
  /** Where the engine pours; defaults to ctx.destination. */
  destination?: AudioNode;
}

export declare class Zaltz {
  readonly ctx: AudioContext;
  readonly node: AudioWorkletNode;
  /** Engine-reported errors (dropped events, OOM) — one line per incident. */
  onerror: ((message: string) => void) | null;
  /** Audio-thread clock ticks (~every 85 ms) — survives background-tab timer
   *  clamps; drive your scheduler's lookahead from this. */
  onclock: ((currentTime: number) => void) | null;
  /** Non-finite output samples the engine zeroed before they could reach the
   *  graph (lifetime count, reported at most ~1/s and only when it grows) —
   *  at worst a click instead of a poisoned compressor, but a sign of real
   *  upstream corruption worth reporting. */
  onscrub: ((scrubbedSamples: number) => void) | null;
  /** Stem-tap batches while armed (see stems()) — one call per ~85 ms batch. */
  onstems: ((batch: ZaltzStemBatch) => void) | null;

  static create(ctx: AudioContext, opts?: ZaltzCreateOptions): Promise<Zaltz>;

  /** Schedule one voice at absolute AudioContext time `when` (seconds). */
  schedule(params: ZaltzParams, when: number): void;
  /** Schedule a batch in one message (kinder to the audio-thread GC). */
  scheduleAll(events: { params: ZaltzParams; when: number }[]): void;
  /** Silence voices/events/buses; uploaded samples survive. */
  hush(): void;
  /** Crossfade takeover: fade current sound over `seconds`; later events untouched. */
  retire(seconds?: number): void;
  /** Glided per-orbit output gain (0..2) — the channel-kill primitive. */
  setOrbitGain(orbit: number, gain: number): void;
  /** Arm/disarm the live stem tap: per-orbit post-FX audio streams through
   *  `onstems` while armed. Disarm flushes the tail, then `done` fires. */
  stems(on: boolean, done?: () => void): void;
  /** Return a spent batch's buffer to the worklet's pool (zero-alloc tap). */
  recycleStemBatch(stemBatch: Float32Array): void;
  /** Upload interleaved PCM under sample id `id`. */
  loadSample(
    id: number,
    pcm: Float32Array,
    opts?: { frames?: number; channels?: number },
  ): void;
  /** Decode-and-upload convenience for an AudioBuffer (≤2 channels kept). */
  loadAudioBuffer(id: number, buf: AudioBuffer): void;
  /** Detach from the graph. */
  dispose(): void;
}

export default Zaltz;
