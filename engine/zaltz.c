// ZALTZ — Klappn's own synthesis engine. C99, freestanding, wasm32.
//
// FAITHFULNESS CONTRACT: this engine implements SUPERDOUGH's semantics — the
// engine every Klappn song was written against — not a new sound. Every
// formula is ported from superdough's JS (file:line noted inline). dough
// (codeberg.org/uzu/dough) is the architectural reference (single C file,
// worklet host, string event protocol), not the DSP oracle.
//
// v0.1 scope (golden-gated vs superdough on synth notes):
//   sources: sine | sawtooth (polyBLEP) | square (polyBLEP) | triangle
//   chain:   osc ×0.3 → ADSR env → ×(gain·velocity) → lowpass biquad →
//            equal-power pan → out ×postgain          (synth.mjs:42-80)
//   protocol: latin-1 "key/value/key/value\0" written at sd_event_ptr(),
//             then sd_event(); sd_dsp() renders one 128-frame stereo block
//             at sd_out_ptr(). Memory is EXPORTED (worklet-owned): no
//             SharedArrayBuffer, no COOP/COEP, runs on every AudioWorklet
//             browser. No imports at all — errors are return codes.
//
// Build: engine/build.sh (zig cc, wasm32-freestanding).

#include <stdbool.h>
#include <stdint.h>

#define SR_MAX 96000
#define BLOCK 128
#define MAX_VOICES 128
#define MAX_EVENTS 256
#define EVENT_BUF 2048
#define OUT_CH 2

// ---------- tiny libm (freestanding: no libc/libm on wasm32) ----------------
// sin: range-reduced 7th-order minimax on [-pi,pi] (max err ~1e-6 — used per
// sample for sine voices and once per note for pan/filter coefficients).
static const float PI_F = 3.14159265358979f;
static const float TWO_PI = 6.28318530717959f;

static float sd_sinf(float x) {
  // quadrant reduction: sin(x) = ±sin(y), y = x − k·π ∈ [−π/2, π/2]
  float k = (float)(int)(x * 0.31830988618f + (x >= 0 ? 0.5f : -0.5f)); // round(x/π)
  float y = x - k * PI_F;
  float y2 = y * y;
  // Taylor deg-7 on [−π/2, π/2]: max err ~1.6e-5 (−96 dB)
  float r = y * (1.0f + y2 * (-0.166666667f + y2 * (0.008333333f + y2 * -0.000198413f)));
  // odd k flips the sign
  return ((int)k & 1) ? -r : r;
}
static float sd_cosf(float x) { return sd_sinf(x + 1.57079632679f); }

// exp2: split int/frac, 5th-order poly on [0,1) (coefficient-time only).
static float sd_exp2f(float x) {
  if (x < -126.0f) return 0.0f;
  int ip = (int)x;
  if (x < 0 && x != (float)ip) ip -= 1;
  float fr = x - (float)ip;
  float p = 1.0f + fr * (0.69314718f + fr * (0.24022651f + fr * (0.05550411f + fr * (0.00961813f + fr * 0.00133336f))));
  union { uint32_t u; float f; } sc;
  sc.u = (uint32_t)(ip + 127) << 23;
  return p * sc.f;
}
static float sd_pow10f(float x) { return sd_exp2f(x * 3.32192809489f); }
static float sd_log2f(float x) {
  if (x <= 0) return -126.0f;
  union { uint32_t u; float f; } g; g.f = x;
  int e = (int)((g.u >> 23) & 0xFF) - 127;
  g.u = (g.u & 0x007FFFFFu) | 0x3F800000u; // mantissa in [1,2)
  float m = g.f;
  // poly for log2(m) on [1,2), err ~2e-4 (control-rate only)
  float t = m - 1.0f;
  float l = t * (1.442695f + t * (-0.7213475f + t * (0.4809f + t * -0.2987f)));
  return (float)e + l;
}

static float sd_fabsf(float x) { return x < 0 ? -x : x; }
static float sd_sqrtf(float x) {
  if (x <= 0) return 0;
  // Newton on 1/√x, seeded by exponent halving
  union { uint32_t u; float f; } g; g.f = x;
  g.u = 0x5f3759dfu - (g.u >> 1);
  float y = g.f;
  y = y * (1.5f - 0.5f * x * y * y);
  y = y * (1.5f - 0.5f * x * y * y);
  y = y * (1.5f - 0.5f * x * y * y);
  return x * y;
}
static float sd_fminf(float a, float b) { return a < b ? a : b; }
// DENORMAL FLUSH — reverb/delay tails decaying toward 1e-38 push CPUs into
// software-float mode (intermittent crackle, worst in quiet tails). Flush.
static inline float undenorm(float x) { return (x < 1e-15f && x > -1e-15f) ? 0.0f : x; }
static float sd_fmaxf(float a, float b) { return a > b ? a : b; }

// ---------- ADSR — exact port of superdough getADSRValues + getParamADSR ----
// helpers.mjs:167 getADSRValues: envmin .001, releaseMin .01, envmax 1;
// all-null → defaults; sustain rule: s ?? ((a&&!d)||(!a&&!d) ? envmax : envmin)
// synth defaults (synth.mjs:47): [0.001, 0.05, 0.6, 0.01]
typedef struct {
  float attack, decay, sustain, release;
} Adsr;

static const float NAN_F = __builtin_nanf("");
static bool is_nan(float x) { return x != x; }

static Adsr adsr_values(float a, float d, float s, float r,
                        float da, float dd, float ds, float dr) {
  const float envmin = 0.001f, relmin = 0.01f, envmax = 1.0f;
  Adsr out;
  if (is_nan(a) && is_nan(d) && is_nan(s) && is_nan(r)) {
    out.attack = da; out.decay = dd; out.sustain = ds; out.release = dr;
    return out;
  }
  float sus = !is_nan(s) ? s
              : ((!is_nan(a) && is_nan(d)) || (is_nan(a) && is_nan(d))) ? envmax
                                                                        : envmin;
  out.attack = sd_fmaxf(is_nan(a) ? 0 : a, envmin);
  out.decay = sd_fmaxf(is_nan(d) ? 0 : d, envmin);
  out.sustain = sd_fminf(sus, envmax);
  out.release = sd_fmaxf(is_nan(r) ? 0 : r, relmin);
  return out;
}

// getParamADSR linear shape (helpers.mjs:40-99), evaluated analytically at
// time `tt` since note start, with hold end `dur` (superdough: end=t+duration):
//   [0,a): min→max · [a,a+d): max→sustainVal · [a+d,dur): sustainVal
//   [dur, dur+r): →min · after: min       (min=0, max=1 for the amp env)
static float adsr_at(const Adsr *e, float tt, float dur) {
  const float min = 0.0f, max = 1.0f;
  const float susv = min + e->sustain * (max - min);
  if (tt < 0) return min;
  if (tt >= dur) { // release from whatever value held at `dur`
    float held;
    if (dur < e->attack) held = min + (max - min) * (dur / e->attack);
    else if (dur < e->attack + e->decay)
      held = max + (susv - max) * ((dur - e->attack) / e->decay);
    else held = susv;
    float rt = (tt - dur) / e->release;
    if (rt >= 1.0f) return min;
    return held + (min - held) * rt;
  }
  if (tt < e->attack) return min + (max - min) * (tt / e->attack);
  if (tt < e->attack + e->decay)
    return max + (susv - max) * ((tt - e->attack) / e->decay);
  return susv;
}

// ---------- biquad — WebAudio BiquadFilterNode lowpass semantics -------------
// Spec: lowpass resonance Q is in dB → linear q = 10^(Q/20); RBJ cookbook.
typedef struct {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
  bool active;
} Biquad;

static void biquad_lowpass(Biquad *f, float freq, float qdb, float sr) {
  float w0 = TWO_PI * sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / sr;
  float q = sd_fmaxf(sd_pow10f(qdb / 20.0f), 0.0001f);
  float alpha = sd_sinf(w0) / (2.0f * q);
  float cosw = sd_cosf(w0);
  float a0 = 1.0f + alpha;
  f->b0 = ((1.0f - cosw) / 2.0f) / a0;
  f->b1 = (1.0f - cosw) / a0;
  f->b2 = f->b0;
  f->a1 = (-2.0f * cosw) / a0;
  f->a2 = (1.0f - alpha) / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0;
  f->active = true;
}
// notch — WebAudio Q is LINEAR here (only lp/hp speak dB); RBJ cookbook
static void biquad_notch(Biquad *f, float freq, float q, float sr) {
  float w0 = TWO_PI * sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / sr;
  if (q < 0.0001f) q = 0.0001f;
  float alpha = sd_sinf(w0) / (2.0f * q);
  float cosw = sd_cosf(w0);
  float a0 = 1.0f + alpha;
  float x1 = f->x1, x2 = f->x2, y1 = f->y1, y2 = f->y2; // retune keeps state
  f->b0 = 1.0f / a0;
  f->b1 = (-2.0f * cosw) / a0;
  f->b2 = 1.0f / a0;
  f->a1 = (-2.0f * cosw) / a0;
  f->a2 = (1.0f - alpha) / a0;
  f->x1 = x1; f->x2 = x2; f->y1 = y1; f->y2 = y2;
  f->active = true;
}

static void biquad_highpass(Biquad *f, float freq, float qdb, float sr) {
  float w0 = TWO_PI * sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / sr;
  float q = sd_fmaxf(sd_pow10f(qdb / 20.0f), 0.0001f);
  float alpha = sd_sinf(w0) / (2.0f * q);
  float cosw = sd_cosf(w0);
  float a0 = 1.0f + alpha;
  f->b0 = ((1.0f + cosw) / 2.0f) / a0;
  f->b1 = (-(1.0f + cosw)) / a0;
  f->b2 = f->b0;
  f->a1 = (-2.0f * cosw) / a0;
  f->a2 = (1.0f - alpha) / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0;
  f->active = true;
}
static void biquad_lowpass_coeffs(Biquad *f, float freq, float qdb, float sr) {
  float x1 = f->x1, x2 = f->x2, y1 = f->y1, y2 = f->y2;
  bool act = f->active;
  biquad_lowpass(f, freq, qdb, sr);
  f->x1 = x1; f->x2 = x2; f->y1 = y1; f->y2 = y2;
  f->active = act || true;
}
static inline float biquad_run(Biquad *f, float x) {
  float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;
  return y;
}

// NOISE prng — xorshift32; superdough noise is Math.random (noise.mjs), so
// statistical equivalence is the ceiling by construction
static unsigned int nz_seed = 0x9E3779B9u;
static inline float nz_rand(void) {
  nz_seed ^= nz_seed << 13;
  nz_seed ^= nz_seed >> 17;
  nz_seed ^= nz_seed << 5;
  return (float)(nz_seed & 0xFFFFFF) * (2.0f / 16777216.0f) - 1.0f;
}

// LADDER lowpass — ftype("ladder"): 4-pole tanh cascade with a 4-tap output
// mix, VERBATIM from superdough's LadderProcessor (worklets.mjs:365-427).
// q is RAW here (k = q·0.13), unlike the biquad's Q-in-dB.
typedef struct { float p0, p1, p2, p3, p32, p33, p34; } Ladder;
static inline float fast_tanh(float x) {
  float x2 = x * x;
  return (x * (27.0f + x2)) / (27.0f + 9.0f * x2);
}
static inline float ladder_run(Ladder *f, float in, float cut, float k, float drive) {
  // `out` reads the PREVIOUS state first — the worklet's exact order
  float out = f->p3 * 0.360891f + f->p32 * 0.41729f + f->p33 * 0.177896f + f->p34 * 0.0439725f;
  f->p34 = f->p33; f->p33 = f->p32; f->p32 = f->p3;
  f->p0 += (fast_tanh(in * drive - k * out) - fast_tanh(f->p0)) * cut;
  f->p1 += (fast_tanh(f->p0) - fast_tanh(f->p1)) * cut;
  f->p2 += (fast_tanh(f->p1) - fast_tanh(f->p2)) * cut;
  f->p3 += (fast_tanh(f->p2) - fast_tanh(f->p3)) * cut;
  return out;
}
static inline float ladder_cut(float freq, float sr) {
  float c = freq * TWO_PI / sr; // worklets.mjs:401-403
  return c > 1.0f ? 1.0f : c;
}

// getParamADSR 'exponential' shape on [vmin..vmax] evaluated analytically —
// exponentialRamp interpolation v1·(v2/v1)^u between the same schedule points
// as the linear amp env. vmin/vmax floored at 0.001 (helpers.mjs:60).
static float adsr_exp_at(const Adsr *e, float tt, float dur, float vmin, float vmax) {
  if (vmin < 0.001f) vmin = 0.001f;
  if (vmax < 0.001f) vmax = 0.001f;
  const float susv = vmin + e->sustain * (vmax - vmin);
  float from, to, u;
  if (tt < 0) return vmin;
  if (tt >= dur) {
    float held;
    if (dur < e->attack) held = vmin * sd_exp2f((dur / e->attack) * sd_log2f(vmax / vmin));
    else if (dur < e->attack + e->decay)
      held = vmax * sd_exp2f(((dur - e->attack) / e->decay) * sd_log2f(sd_fmaxf(susv, 0.001f) / vmax));
    else held = sd_fmaxf(susv, 0.001f);
    float rt = (tt - dur) / e->release;
    if (rt >= 1.0f) return vmin;
    return held * sd_exp2f(rt * sd_log2f(vmin / held));
  }
  if (tt < e->attack) { from = vmin; to = vmax; u = tt / e->attack; }
  else if (tt < e->attack + e->decay) { from = vmax; to = sd_fmaxf(susv, 0.001f); u = (tt - e->attack) / e->decay; }
  else return sd_fmaxf(susv, 0.001f);
  return from * sd_exp2f(u * sd_log2f(to / from));
}

// ---------- voices ------------------------------------------------------------
enum Source { SRC_SINE = 0, SRC_SAW, SRC_SQUARE, SRC_TRIANGLE, SRC_SUPERSAW, SRC_SAMPLE,
              SRC_WHITE, SRC_PINK, SRC_BROWN, SRC_CRACKLE };
#define MAX_UNISON 16 /* superdough clamps 1..100; corpus uses the default 5 */

// xorshift32 — random initial phases per supersaw voice (worklets.mjs:559
// uses Math.random(); exact values don't matter, decorrelation does)
static uint32_t rng_state = 0x9E3779B9u;
static float frandf(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 17;
  rng_state ^= rng_state << 5;
  return (float)(rng_state >> 8) * (1.0f / 16777216.0f);
}

typedef struct {
  bool active;
  int src;
  double phase;      // 0..1
  double phase_inc;  // freq / sr
  double start_frame;
  float dur;         // seconds (hold end)
  float end;         // dur + release + 0.01 (synth.mjs:69 envEnd)
  Adsr env;
  float amp;         // 0.3 (synth headroom) × gain × velocity
  float pan_l, pan_r;
  bool pan_set;      // StereoPanner exists only when pan was given
  float pan_x;       // 2·pan−1 (stereo pan law needs the raw x)
  float postgain;
  int wt_lvl;
  int orbit;
  float room_send, delay_send;
  float shape_k, shapevol; // (1+k)x/(1+k|x|) drive (ShapeProcessor port)
  bool shape_on;
  float crush; // bit reduce: round(x·2^(crush−1))/2^(crush−1) (webdirt)
  int coarse;  // sample-hold every N samples (webdirt)
  float coarse_hold_l, coarse_hold_r;
  int coarse_ctr;
  int cut_group;              // choke group (sampler.mjs:341-347)
  double cutkill_at;          // 10ms linear kill starts here (frames)
  bool cutkill;
  unsigned int vid;           // generation stamp — cut registry safety
  // supersaw (worklets.mjs SuperSawOscillatorProcessor)
  int unison;
  float fan[MAX_UNISON];       // per-voice semitone offsets (getDetuner)
  double ss_phase[MAX_UNISON]; // random initial phases
  float ss_gl, ss_gr;          // alternating √panspread gains
  Biquad lpf;
  Biquad hpf;
  Biquad lpf_r; // right-channel twins for the stereo (supersaw) path
  Biquad hpf_r;
  // ftype("ladder") replaces the lp biquad; ftype("24db") cascades a second
  bool ladder_on;
  Ladder lad, lad_r;
  float lad_cut, lad_k, lad_drive, lad_makeup;
  bool lp24;
  Biquad lpf2, lpf2_r;
  // superdough QUIRK kept verbatim (superdough.mjs:706 model:'ftype'): under
  // ftype("ladder"), hpf() builds ANOTHER ladder — a LOWPASS topology — at
  // the hpf frequency. The corpus was authored with this sound.
  bool hladder_on;
  Ladder hlad, hlad_r;
  float hlad_cut, hlad_k, hlad_drive, hlad_makeup;
  bool hp24;
  Biquad hpf2, hpf2_r;
  // FILTER ENVELOPE (superdough createFilter, helpers.mjs:219 — exponential
  // curve, defaults [0.005, 0.14, 0, 0.1]): frequency glides fmin↔fmax
  bool lp_env_active;
  Adsr lp_env;
  float lp_fmin, lp_fmax, lp_qdb;
  // RETIRE (the crossfade takeover): the OLD music fades out under the new
  // loop instead of being hushed — set on live voices by sd_retire and
  // inherited by voices spawned from pre-retire events
  double retire_start; // frame the fade began (-1 = not retiring)
  // PHASER (superdough.mjs getPhaser): ONE notch at center+282, LFO ±sweep
  // cents on its frequency at `rate` Hz; Q = 2 − clamp(2·depth, 0, 1.9)
  bool phaser_on;
  Biquad ph_l, ph_r;
  float ph_center, ph_q, ph_sweep, ph_rate;
  // NOISE (noise.mjs getNoiseBuffer — Math.random buffer, ported as a
  // per-sample xorshift generator; statistically identical, never bit-equal)
  float nz_last;    // brown integrator
  float nz_b[7];    // pink (Paul Kellet)
  float nz_density; // crackle
  // VIB (helpers.mjs:346): detune cents = sin(2π·vib·t)·vibmod·100
  float vib_hz, vibmod;
  float base_freq;
  // SAMPLE voice
  const float *pcm;
  int pcm_frames, pcm_channels;
  double pos;      // frames into the buffer
  double rate;     // playbackRate (buffer frames per output frame)
  double base_rate;
  double end_frame; // end fraction × frames
  bool smp_loop;
  double loop_a, loop_b;
} Voice;

typedef struct {
  float time, freq, duration;
  float attack, decay, sustain, release;
  float gain, velocity, postgain, pan;
  float lpf, lpq, hpf, hpq;
  float unison, spread, detune;
  float lpattack, lpdecay, lpsustain, lprelease, lpenv;
  int ftype;   // 0 = 12db biquad, 1 = ladder, 2 = 24db (superdough.mjs:362)
  float drive; // ladder drive, default 0.69 (helpers.mjs:227)
  float density; // crackle probability × 100 (noise.mjs:37)
  float phaserrate, phaserdepth, phasercenter, phasersweep;
  bool retire; // queued before sd_retire → its voice fades with the old music
  float vib, vibmod;
  int sample_id;
  float speed, begin, endf, loopv, loop_begin, loop_end;
  int orbit;
  float room, roomsize, roomlp, delay, delaytime, delayfeedback;
  float shape, shapevol;
  int duck_targets[8];
  int duck_n;
  float duckonset, duckattack, duckdepth;
  float crush, coarse;
  int cut;
  int src;
  bool used;
  double at_frame;
} Event;

// ---------- band-limited sawtooth — WebAudio's own definition ---------------
// The spec's sawtooth IS a truncated Fourier series: sum (2/π)·(−1)^(k+1)/k ·
// sin(2πkft), partials to Nyquist. polyBLEP has a different phase spectrum and
// measurably beats against it through the golden gate — so we build the exact
// series as an octave mipmap at init (one-time cost, ~ms).
#define WT_LEN 2048
#define WT_LEVELS 11
static float wt_saw[WT_LEVELS][WT_LEN + 1];

static void build_wavetables(float sr) {
  for (int lvl = 0; lvl < WT_LEVELS; lvl++) {
    // band top for this level: 40·2^lvl Hz — partials chosen so the TOP of the
    // band still clears Nyquist
    float band_top = 40.0f;
    for (int i = 0; i < lvl; i++) band_top *= 2.0f;
    int N = (int)((sr * 0.5f) / band_top);
    if (N < 1) N = 1;
    if (N > 1024) N = 1024;
    for (int i = 0; i < WT_LEN; i++) {
      float t = (float)i / (float)WT_LEN;
      float acc = 0.0f;
      float sign = 1.0f;
      for (int k = 1; k <= N; k++) {
        acc += sign * sd_sinf(TWO_PI * (float)k * t) / (float)k;
        sign = -sign;
      }
      wt_saw[lvl][i] = acc * 0.63661977f; // 2/π
    }
    wt_saw[lvl][WT_LEN] = wt_saw[lvl][0]; // lerp guard
  }
}

static int wt_level_for(float freq) {
  int lvl = 0;
  float top = 40.0f;
  while (lvl < WT_LEVELS - 1 && freq > top) {
    top *= 2.0f;
    lvl++;
  }
  return lvl;
}

// ---------- ORBIT BUSES (M4) — per-orbit delay, FDN reverb, duck ------------
// superdough routing (superdough.mjs:938-955 + superdoughoutput.mjs): each
// voice lives on an orbit; .room()/.delay() are SENDS into the orbit's shared
// reverb/delay; duck dips the orbit's OUTPUT gain. The rebus guarantees one
// effect signature per orbit, so each bus is configured once.
static float sr_f; // defined below (tentative — orbit code needs it early)

#define MAX_ORBITS 40 /* the deck's kill decades reach 10..39 (set-live.ts) */
#define FDN_LINES 8
#define FDN_MAX 3072
// ---------- growable arena — delay lines AND sample PCM live here ----------
// Fixed arenas were cliffs: 16MB of delay = ~11 orbits at 48k (a set's kill
// decades blew past it → later delays silently DRY), 48MB of PCM couldn't
// hold one piano's zones. The arena claims the end of linear memory at first
// take and memory.grow's in >=4MB chunks (rare, bounded work — some 32-bit
// engines COPY on grow). Nothing here is ever freed; delay lines are cached
// per-orbit across hush (same lifetime the old delay_used=0 reuse gave),
// samples persist by design. Host views detach on every grow — the worklet
// re-derives them (syncViews).
static float *arena_base = 0;
static long arena_used = 0, arena_cap = 0;
static float *arena_take(long need) {
  if (need <= 0) return 0;
  if (!arena_base) arena_base = (float *)(__builtin_wasm_memory_size(0) * 65536ul);
  if (arena_used + need > arena_cap) {
    unsigned long want = ((unsigned long)(arena_used + need - arena_cap) * 4ul + 65535ul) / 65536ul;
    unsigned long pages = want < 64 ? 64 : want; /* >=4MB per grow */
    if ((long)__builtin_wasm_memory_grow(0, pages) < 0) {
      if (pages == want || (long)__builtin_wasm_memory_grow(0, want) < 0) return 0; /* true OOM */
    }
    /* recompute from truth: cap = floats between arena_base and memory end */
    arena_cap = (long)(__builtin_wasm_memory_size(0) * (65536ul / 4ul)) - (long)((unsigned long)arena_base / 4ul);
  }
  float *p = arena_base + arena_used;
  arena_used += need;
  return p;
}

typedef struct {
  bool used;
  // duck: exponential dip on the orbit output (superdoughoutput.mjs:102-121)
  bool duck_active;
  double duck_t0, duck_t1, duck_t2; // start, dip end (onset), recovery end
  float duck_from, duck_low;
  // delay bus (send): ring + feedback (feedbackdelay.mjs), smoothed moves
  bool delay_on;
  float *dl_l, *dl_r;
  int dl_len, dl_pos;
  int dl_fill; // samples written since enable — reads beyond this are SILENCE
               // (rings are never zeroed: clearing 768KB per delay orbit in the
               // first-beat render quantum was an audible stall at loop start)
  float dt_cur, dt_tgt; // delay seconds (smoothed — the click war's lesson)
  float fb_cur, fb_tgt;
  // reverb bus (send): 8-line Householder FDN tuned to the IR's T60 + damping
  bool verb_on;
  float t60, lp_hz;
  float fdn[FDN_LINES][FDN_MAX];
  int fdn_len[FDN_LINES], fdn_pos[FDN_LINES];
  int fdn_fill; // samples written since enable — same no-zero law as dl_fill
  float fdn_g[FDN_LINES];
  float damp[FDN_LINES];
  float damp_a;
  // MODE SMEAR: fixed-length FDN lines ring at their own faintly-PITCHED
  // modal frequencies ("off-pitch" metallic tails on percussive sends — the
  // convolver's random IR has no modes). Classic fix: diffuse the input
  // (series allpasses) and MODULATE each line's read tap a few samples at
  // slow incommensurate rates — the modes never sit still long enough to ring.
  float ap[2][768];   // two allpass diffusers (5.4ms/12.7ms at 48k), rings
  int ap_pos[2];
  float mod_ph[FDN_LINES];
  float ai[FDN_LINES]; // allpass-interp state (linear interp in a feedback
                       // loop is a per-pass LOWPASS — it ate 4dB of tail)
  float wet_scale; // ConvolverNode normalize=true analog — see orbit_config_verb
  float duck_g;    // smoothed applied duck gain (never steps)
  // deck CHANNEL KILLS: per-orbit output gain, glided (~10ms tau) — a kill
  // silences already-ringing tails instantly-but-clicklessly, like kill EQ
  float out_g, out_g_tgt;
  // RETUNE targets: room params glide (authored fx sweep roomsize per hap!)
  // — coefficients ramp toward these per block; the ring NEVER resets live
  float fdn_g_tgt[FDN_LINES];
  float damp_a_tgt, wet_tgt;
  // block accumulators
  float dry[BLOCK * OUT_CH];
  float vin[BLOCK];  // reverb send (mono in, stereo out)
  float din[BLOCK * OUT_CH];
} Orbit;
static Orbit orbits[MAX_ORBITS];

// FDN line lengths: primes ≈ 21–60ms at 48k, scaled to sr
static const int FDN_PRIMES[FDN_LINES] = { 1031, 1327, 1523, 1801, 2053, 2311, 2617, 2903 };

static void orbit_config_verb(Orbit *o, float t60, float lp) {
  float t = t60 > 0.05f ? t60 : 2.0f;
  float l = lp > 0 ? lp : 15000.0f;
  if (o->verb_on && o->t60 == t && o->lp_hz == l) return;
  // RETUNE = new TARGETS, glided per block in the bus pass. The old code
  // ZEROED the ring memory and reset positions on every param change — an
  // instant tail truncation = the "random horrible ticks" the ear caught
  // (authored fx glide roomsize PER HAP; every section boundary retunes).
  // Line lengths depend only on sr, so a live retune touches nothing else.
  bool init = !o->verb_on;
  o->verb_on = true;
  o->t60 = t;
  o->lp_hz = l;
  for (int i = 0; i < FDN_LINES; i++) {
    int len = (int)((float)FDN_PRIMES[i] * (sr_f / 48000.0f));
    if (len >= FDN_MAX) len = FDN_MAX - 1;
    o->fdn_len[i] = len;
    // g = 10^(−3·lineSeconds/T60): −60dB after T60 (reverbGen decayBase)
    o->fdn_g_tgt[i] = sd_pow10f(-3.0f * ((float)len / sr_f) / t);
  }
  float w = TWO_PI * sd_fminf(l, sr_f * 0.45f) / sr_f;
  o->damp_a_tgt = 1.0f - sd_exp2f(-w * 1.442695f); // one-pole coefficient
  // WET NORMALIZATION: the ConvolverNode normalizes IR energy, making wet
  // level roughly size-independent. An FDN's steady-state energy grows as
  // 1/(1−ḡ²) with T60 (measured: +5-6dB at roomsize 7-9 vs the size-2
  // calibration). Normalize by sqrt((1−ḡ²)/(1−g_ref²)) with g_ref at T60=2
  // for a ~42ms mean line — the golden room case's calibration point.
  float g2 = 0;
  for (int i = 0; i < FDN_LINES; i++) g2 += o->fdn_g_tgt[i] * o->fdn_g_tgt[i];
  g2 /= (float)FDN_LINES;
  const float ref = 0.241f; // 1 − g² at T60=2, mean line ≈ 42ms
  // 0.50: recalibrated after input diffusion + 2-line modulation (the same
  // convolver reference read the diffused network ~3dB lower at 0.35)
  o->wet_tgt = 0.50f * sd_sqrtf((1.0f - g2) / ref);
  if (init) {
    for (int i = 0; i < FDN_LINES; i++) {
      o->fdn_pos[i] = 0;
      o->damp[i] = 0;
      o->fdn_g[i] = o->fdn_g_tgt[i];
      o->mod_ph[i] = (float)i * 0.7853f; // spread the mod phases
      o->ai[i] = 0;
      // NO line zeroing — ~100KB per orbit in the first-beat quantum was an
      // audible stall; fdn_fill gates unwritten reads to silence instead
    }
    o->fdn_fill = 0;
    for (int a = 0; a < 2; a++) {
      o->ap_pos[a] = 0;
      for (int j = 0; j < 768; j++) o->ap[a][j] = 0; // 6KB — negligible
    }
    o->damp_a = o->damp_a_tgt;
    o->wet_scale = o->wet_tgt;
  }
}

static void orbit_config_delay(Orbit *o, float dt, float fb) {
  if (!o->delay_on) {
    int len = (int)(sr_f * 2.0f); // 2s max line
    if (!o->dl_l) { // first use EVER for this orbit — line cached across hush
      float *mem = arena_take((long)len * 2);
      if (!mem) return; // OOM → dry
      o->dl_l = mem;
      o->dl_r = mem + len;
    }
    o->dl_len = len;
    o->dl_pos = 0;
    o->dl_fill = 0; // NO memset — unwritten reads are gated to silence below
    o->delay_on = true;
    o->dt_cur = dt;
    o->fb_cur = fb;
  }
  o->dt_tgt = sd_fminf(sd_fmaxf(dt, 0.001f), 1.99f);
  o->fb_tgt = sd_fminf(sd_fmaxf(fb, 0.0f), 0.98f); // superdoughoutput clamp
}

// duck: exponential dip to clamp(1−√depth, .01, cur) over `onset`, back over
// `attack` (superdoughoutput.mjs:118-120)
static void orbit_duck(Orbit *o, double t_frame, float onset, float attack, float depth) {
  float low = 1.0f - sd_sqrtf(depth);
  if (low < 0.01f) low = 0.01f;
  o->duck_active = true;
  // start the dip from the CURRENT smoothed gain (superdough reads the
  // param's current value) — a dip landing mid-recovery must not snap to 1
  o->duck_from = o->duck_g > 0.01f ? o->duck_g : 0.01f;
  o->duck_low = low;
  o->duck_t0 = t_frame;
  o->duck_t1 = t_frame + (double)(onset * sr_f);
  o->duck_t2 = o->duck_t1 + (double)(sd_fmaxf(attack, 0.002f) * sr_f);
}

static inline float orbit_duck_env(Orbit *o, double f) {
  if (!o->duck_active) return 1.0f;
  if (f >= o->duck_t2) { o->duck_active = false; return 1.0f; }
  if (f < o->duck_t0) return o->duck_from;
  if (f < o->duck_t1 || o->duck_t1 <= o->duck_t0) {
    if (o->duck_t1 <= o->duck_t0) return o->duck_low;
    float u = (float)((f - o->duck_t0) / (o->duck_t1 - o->duck_t0));
    return o->duck_from * sd_exp2f(u * sd_log2f(o->duck_low / o->duck_from));
  }
  float u = (float)((f - o->duck_t1) / (o->duck_t2 - o->duck_t1));
  return o->duck_low * sd_exp2f(u * sd_log2f(1.0f / o->duck_low));
}
static inline float orbit_duck_gain(Orbit *o, double f) {
  // 1ms one-pole follower: whatever the envelope does (overlaps, re-triggers,
  // registration races), the APPLIED gain never steps
  float env = orbit_duck_env(o, f);
  o->duck_g += (env - o->duck_g) * 0.02f;
  return o->duck_g;
}

// ---------- sample store — PCM uploaded by the host, referenced by id ------
// The store GROWS: pitched multisample maps (piano = ~12 zones × 16s stereo)
// dwarf any fixed arena a phone should pay for up front. First alloc claims
// the current end of linear memory, then memory.grow extends it on demand —
// footprint tracks the song's actual kit, exactly like superdough's decoded
// AudioBuffers. Host views detach on every grow (worklet re-derives them).
#define MAX_SAMPLES 2048 /* ids are never freed — a long set's gm zones alone pass 512 */
typedef struct { long offset; int frames, channels; bool ready; } Sample;
static Sample samples[MAX_SAMPLES];

// returns the write pointer for `frames×channels` floats, or 0 when OOM
__attribute__((export_name("sd_sample_alloc"))) float *sd_sample_alloc(int id, int frames, int channels) {
  if (id < 0 || id >= MAX_SAMPLES || frames <= 0 || channels <= 0 || channels > 2) return 0;
  float *ptr = arena_take((long)frames * channels);
  if (!ptr) return 0;
  samples[id].offset = (long)(ptr - arena_base);
  samples[id].frames = frames;
  samples[id].channels = channels;
  samples[id].ready = true; // synchronous uploaders (harness) never call hold
  return ptr;
}

// PACED uploads (the worklet): a half-copied buffer must never sound — a
// voice reading the zero→content boundary IS a click. hold at alloc, ready
// when the last chunk lands; events referencing a held sample are SKIPPED
// (superdough's own "still loading" semantics — the next cycle's hap plays).
// CROSSFADE TAKEOVER: fade everything that is currently sounding or queued
// over `seconds`, leaving anything scheduled AFTER this call untouched — the
// old loop retires under the new one instead of being cut to silence.
static float retire_inv = 0; // 1/(fade frames); 0 = no retire configured
static double retire_frame = -1;
void sd_retire(float seconds);

// deck kill hook: ramps the orbit's OUTPUT gain toward g (bus law: never step)
__attribute__((export_name("sd_orbit_gain"))) void sd_orbit_gain(int orbit, float g) {
  if (orbit < 0 || orbit >= MAX_ORBITS) return;
  if (g < 0) g = 0; else if (g > 2.0f) g = 2.0f;
  orbits[orbit].out_g_tgt = g;
  // an idle orbit never glides (the bus pass skips it) — snap so a kill set
  // while the channel is silent doesn't blip 10ms of its first note
  if (!orbits[orbit].used) orbits[orbit].out_g = g;
}

__attribute__((export_name("sd_sample_hold"))) void sd_sample_hold(int id) {
  if (id >= 0 && id < MAX_SAMPLES) samples[id].ready = false;
}
__attribute__((export_name("sd_sample_ready"))) void sd_sample_ready(int id) {
  if (id >= 0 && id < MAX_SAMPLES) samples[id].ready = true;
}

static float sr_f = 48000.0f;
static double engine_frame = 0;
static Voice voices[MAX_VOICES];
#define MAX_CUT_GROUPS 64
static int cut_last[MAX_CUT_GROUPS]; // voice index or -1
// slot indexes RECYCLE: without a generation stamp a cut event kills whatever
// UNRELATED voice reuses the slot (superdough holds a node ref, which dies
// harmlessly) — heard as wrong hats choked in cut(1) rides
static unsigned int cut_last_vid[MAX_CUT_GROUPS];
static unsigned int vid_ctr;
static Event events[MAX_EVENTS];
static float out_buf[BLOCK * OUT_CH];
static char event_buf[EVENT_BUF];

// ---------- event string parsing ("key/value/key/value\0") -------------------
static int str_eq(const char *a, const char *b) {
  while (*a && *b) { if (*a != *b) return 0; a++; b++; }
  return *a == *b;
}
static float parse_f(const char *s) {
  float sign = 1.0f, v = 0.0f;
  if (*s == '-') { sign = -1.0f; s++; }
  while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
  if (*s == '.') {
    s++;
    float p = 0.1f;
    while (*s >= '0' && *s <= '9') { v += (float)(*s - '0') * p; p *= 0.1f; s++; }
  }
  // JS Number→string emits exponents outside ~[1e-6, 1e21) — "3e-9" read as
  // "3" once put an event SECONDS late; the host formats those away now, but
  // the parser must never mis-scale a number again
  if (*s == 'e' || *s == 'E') {
    s++;
    int esign = 1, ex = 0;
    if (*s == '-') { esign = -1; s++; }
    else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') { ex = ex * 10 + (*s - '0'); s++; }
    if (ex > 60) ex = 60; /* far past float range either way */
    float scale = 1.0f, ten = esign > 0 ? 10.0f : 0.1f;
    while (ex--) scale *= ten;
    v *= scale;
  }
  return sign * v;
}

__attribute__((export_name("sd_event_ptr"))) char *sd_event_ptr(void) { return event_buf; }
__attribute__((export_name("sd_out_ptr"))) float *sd_out_ptr(void) { return out_buf; }
__attribute__((export_name("sd_time"))) double sd_time(void) { return engine_frame / (double)sr_f; }

// sd_retire (declared above): live voices + already-queued events fade over
// `seconds`; anything scheduled after this call is the NEW music, untouched.
__attribute__((export_name("sd_retire"))) void sd_retire(float seconds) {
  if (seconds < 0.05f) seconds = 0.05f;
  retire_inv = 1.0f / (seconds * sr_f);
  retire_frame = engine_frame;
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].active && voices[i].retire_start < 0) voices[i].retire_start = engine_frame;
  for (int i = 0; i < MAX_EVENTS; i++)
    if (events[i].used) events[i].retire = true;
}

// HUSH — the transport reset: voices/events/buses cleared, samples AND the
// wavetable bank kept. This runs in the worklet's message handler between
// render quanta at EVERY fresh play, so it must be near-free: rebuilding the
// wavetables here (~2.5M sinf) starved the render thread for tens of ms —
// the "glitches for a moment at play start" the ear caught.
__attribute__((export_name("sd_hush"))) void sd_hush(void) {
  for (int i = 0; i < MAX_VOICES; i++) voices[i].active = false;
  for (int i = 0; i < MAX_EVENTS; i++) events[i].used = false;
  for (int i = 0; i < MAX_CUT_GROUPS; i++) cut_last[i] = -1;
  for (int i = 0; i < MAX_ORBITS; i++) {
    orbits[i].used = false;
    orbits[i].delay_on = false;
    orbits[i].verb_on = false;
    orbits[i].duck_active = false;
    orbits[i].duck_g = 1.0f;
    orbits[i].out_g = 1.0f;
    orbits[i].out_g_tgt = 1.0f;
  }
}

__attribute__((export_name("sd_init"))) void sd_init(float sample_rate) {
  sr_f = sample_rate;
  engine_frame = 0;
  build_wavetables(sample_rate); // ONCE per boot — never on the hush path
  sd_hush();
}

// returns 0 ok, negative = error code (no imports — the host reads the code)
__attribute__((export_name("sd_event"))) int sd_event(void) {
  Event ev;
  ev.time = 0; ev.freq = 440; ev.duration = 0.25f;
  ev.attack = NAN_F; ev.decay = NAN_F; ev.sustain = NAN_F; ev.release = NAN_F;
  ev.gain = 0.8f; // defaultControls gain (superdough.mjs:182) — NOT 1: unset layers read +1.94dB
  ev.velocity = 1; ev.postgain = 1; ev.pan = NAN_F; // NaN = unset → no panner (superdough.mjs:843)
  ev.lpf = 0; ev.lpq = 1; ev.hpf = 0; ev.hpq = 1;
  ev.unison = 5; ev.spread = 0.6f; ev.detune = NAN_F; // supersaw defaults (synth.mjs:157-158)
  ev.lpattack = NAN_F; ev.lpdecay = NAN_F; ev.lpsustain = NAN_F; ev.lprelease = NAN_F; ev.lpenv = NAN_F;
  ev.ftype = 0; ev.drive = 0.69f; ev.density = 0.02f;
  ev.phaserrate = 0; ev.phaserdepth = 0.75f; ev.phasercenter = 1000.0f; ev.phasersweep = 2000.0f;
  ev.retire = false;
  ev.vib = 0; ev.vibmod = 0.5f; // helpers.mjs:347
  ev.sample_id = -1; ev.speed = 1; ev.begin = 0; ev.endf = 1; ev.loopv = 0; ev.loop_begin = 0; ev.loop_end = 1;
  ev.orbit = 1;
  ev.room = 0; ev.roomsize = 2; ev.roomlp = 15000; ev.delay = 0;
  ev.delaytime = 0.25f; ev.delayfeedback = 0.5f; // superdough defaults
  ev.shape = NAN_F; ev.shapevol = 1;
  ev.duck_n = 0; ev.duckonset = 0; ev.duckattack = 0.1f; ev.duckdepth = 1;
  ev.crush = 0; ev.coarse = 0; ev.cut = -1;
  ev.src = SRC_TRIANGLE; // superdough default osc type (synth getOscillator)

  char *p = event_buf;
  char key[32], val[64];
  while (*p) {
    int i = 0;
    while (*p && *p != '/' && i < 31) key[i++] = *p++;
    key[i] = 0;
    if (*p != '/') return -1;
    p++;
    i = 0;
    while (*p && *p != '/' && i < 63) val[i++] = *p++;
    val[i] = 0;
    if (*p == '/') p++;

    if (str_eq(key, "time") || str_eq(key, "t")) ev.time = parse_f(val);
    else if (str_eq(key, "freq")) ev.freq = parse_f(val);
    else if (str_eq(key, "note")) { // midi → hz (superdough getFrequencyFromValue)
      float n = parse_f(val);
      ev.freq = 440.0f * sd_exp2f((n - 69.0f) / 12.0f);
    } else if (str_eq(key, "duration")) ev.duration = parse_f(val);
    else if (str_eq(key, "attack")) ev.attack = parse_f(val);
    else if (str_eq(key, "decay")) ev.decay = parse_f(val);
    else if (str_eq(key, "sustain")) ev.sustain = parse_f(val);
    else if (str_eq(key, "release")) ev.release = parse_f(val);
    else if (str_eq(key, "gain")) ev.gain = parse_f(val);
    else if (str_eq(key, "velocity")) ev.velocity = parse_f(val);
    else if (str_eq(key, "postgain")) ev.postgain = parse_f(val);
    else if (str_eq(key, "pan")) ev.pan = parse_f(val);
    else if (str_eq(key, "lpf") || str_eq(key, "cutoff")) ev.lpf = parse_f(val);
    else if (str_eq(key, "lpq") || str_eq(key, "resonance")) ev.lpq = parse_f(val);
    else if (str_eq(key, "hpf") || str_eq(key, "hcutoff")) ev.hpf = parse_f(val);
    else if (str_eq(key, "hpq") || str_eq(key, "hresonance")) ev.hpq = parse_f(val);
    else if (str_eq(key, "unison")) ev.unison = parse_f(val);
    else if (str_eq(key, "spread")) ev.spread = parse_f(val);
    else if (str_eq(key, "detune")) ev.detune = parse_f(val);
    else if (str_eq(key, "lpattack") || str_eq(key, "lpa")) ev.lpattack = parse_f(val);
    else if (str_eq(key, "lpdecay") || str_eq(key, "lpd")) ev.lpdecay = parse_f(val);
    else if (str_eq(key, "lpsustain") || str_eq(key, "lps")) ev.lpsustain = parse_f(val);
    else if (str_eq(key, "lprelease") || str_eq(key, "lpr")) ev.lprelease = parse_f(val);
    else if (str_eq(key, "lpenv") || str_eq(key, "lpe")) ev.lpenv = parse_f(val);
    else if (str_eq(key, "ftype")) ev.ftype = str_eq(val, "ladder") ? 1 : str_eq(val, "24db") ? 2 : 0;
    else if (str_eq(key, "drive")) ev.drive = parse_f(val);
    else if (str_eq(key, "density")) ev.density = parse_f(val);
    else if (str_eq(key, "phaserrate") || str_eq(key, "phaser")) ev.phaserrate = parse_f(val);
    else if (str_eq(key, "phaserdepth")) ev.phaserdepth = parse_f(val);
    else if (str_eq(key, "phasercenter")) ev.phasercenter = parse_f(val);
    else if (str_eq(key, "phasersweep")) ev.phasersweep = parse_f(val);
    else if (str_eq(key, "vib") || str_eq(key, "vibrato")) ev.vib = parse_f(val);
    else if (str_eq(key, "vibmod") || str_eq(key, "vmod")) ev.vibmod = parse_f(val);
    else if (str_eq(key, "sample")) ev.sample_id = (int)parse_f(val);
    else if (str_eq(key, "speed")) ev.speed = parse_f(val);
    else if (str_eq(key, "begin")) ev.begin = parse_f(val);
    else if (str_eq(key, "end")) ev.endf = parse_f(val);
    else if (str_eq(key, "loop")) ev.loopv = parse_f(val);
    else if (str_eq(key, "loopBegin") || str_eq(key, "loopbegin")) ev.loop_begin = parse_f(val);
    else if (str_eq(key, "loopEnd") || str_eq(key, "loopend")) ev.loop_end = parse_f(val);
    else if (str_eq(key, "orbit")) ev.orbit = (int)parse_f(val);
    else if (str_eq(key, "room")) ev.room = parse_f(val);
    else if (str_eq(key, "roomsize")) ev.roomsize = parse_f(val);
    else if (str_eq(key, "roomlp")) ev.roomlp = parse_f(val);
    else if (str_eq(key, "delay")) ev.delay = parse_f(val);
    else if (str_eq(key, "delaytime")) ev.delaytime = parse_f(val);
    else if (str_eq(key, "delayfeedback")) ev.delayfeedback = parse_f(val);
    else if (str_eq(key, "shape")) ev.shape = parse_f(val);
    else if (str_eq(key, "shapevol")) ev.shapevol = parse_f(val);
    else if (str_eq(key, "duckonset")) ev.duckonset = parse_f(val);
    else if (str_eq(key, "duckattack")) ev.duckattack = parse_f(val);
    else if (str_eq(key, "duckdepth")) ev.duckdepth = parse_f(val);
    else if (str_eq(key, "crush")) ev.crush = parse_f(val);
    else if (str_eq(key, "coarse")) ev.coarse = parse_f(val);
    else if (str_eq(key, "cut")) ev.cut = (int)parse_f(val);
    else if (str_eq(key, "duck")) {
      // colon-joined orbit list, e.g. "2:3" (wireSidechain format)
      const char *q = val;
      while (*q && ev.duck_n < 8) {
        int t = 0;
        bool any = false;
        while (*q >= '0' && *q <= '9') { t = t * 10 + (*q - '0'); q++; any = true; }
        if (any) ev.duck_targets[ev.duck_n++] = t;
        if (*q == ':') q++;
        else break;
      }
    }
    else if (str_eq(key, "s") || str_eq(key, "sound")) {
      if (str_eq(val, "sine")) ev.src = SRC_SINE;
      else if (str_eq(val, "sawtooth") || str_eq(val, "saw")) ev.src = SRC_SAW;
      else if (str_eq(val, "square")) ev.src = SRC_SQUARE;
      else if (str_eq(val, "triangle") || str_eq(val, "tri")) ev.src = SRC_TRIANGLE;
      else if (str_eq(val, "supersaw")) ev.src = SRC_SUPERSAW;
      else if (str_eq(val, "sample")) ev.src = SRC_SAMPLE;
      else if (str_eq(val, "white")) ev.src = SRC_WHITE;
      else if (str_eq(val, "pink")) ev.src = SRC_PINK;
      else if (str_eq(val, "brown")) ev.src = SRC_BROWN;
      else if (str_eq(val, "crackle")) ev.src = SRC_CRACKLE;
      else return -2; // unknown source (v0.1)
    }
    // unknown keys ignored — the bridge counts what it sends
  }

  for (int i = 0; i < MAX_EVENTS; i++) {
    if (!events[i].used) {
      ev.used = true;
      ev.at_frame = engine_frame + (double)(ev.time * sr_f);
      events[i] = ev;
      return 0;
    }
  }
  return -3; // queue full
}

static void start_voice(const Event *ev) {
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active) continue;
    Voice *v = &voices[i];
    v->active = true;
    v->src = ev->src;
    v->phase = 0;
    v->phase_inc = (double)ev->freq / (double)sr_f;
    v->base_freq = ev->freq;
    v->wt_lvl = wt_level_for(ev->freq);
    v->start_frame = ev->at_frame;
    v->dur = ev->duration;
    if (ev->src == SRC_SAMPLE) {
      // sampler ADSR defaults [.001, .001, 1, .01] (sampler.mjs:287, bare
      // getADSRValues) and NO 0.3 headroom (only synths turn down)
      v->env = adsr_values(ev->attack, ev->decay, ev->sustain, ev->release,
                           0.001f, 0.001f, 1.0f, 0.01f);
      v->amp = ev->gain * ev->velocity;
    } else {
      // synth defaults [0.001, 0.05, 0.6, 0.01] — synth.mjs:47
      v->env = adsr_values(ev->attack, ev->decay, ev->sustain, ev->release,
                           0.001f, 0.05f, 0.6f, 0.01f);
      v->amp = 0.3f * ev->gain * ev->velocity; // 0.3 headroom, synth.mjs:54
    }
    v->end = ev->duration + v->env.release + 0.01f; // envEnd, synth.mjs:69
    // StereoPanner equal-power ONLY when pan is set (superdough.mjs:843) —
    // an unpanned voice upmixes mono→stereo at full gain on both channels.
    v->pan_set = !is_nan(ev->pan);
    if (!v->pan_set) {
      v->pan_l = 1.0f;
      v->pan_r = 1.0f;
      v->pan_x = 0.0f;
    } else {
      float x = sd_fminf(sd_fmaxf(2.0f * ev->pan - 1.0f, -1.0f), 1.0f);
      v->pan_x = x;
      float ang = ((x + 1.0f) * 0.5f) * (PI_F * 0.5f);
      v->pan_l = sd_cosf(ang);
      v->pan_r = sd_sinf(ang);
    }
    v->postgain = ev->postgain;
    v->lpf.active = false;
    v->ladder_on = false;
    v->lp24 = false;
    if (ev->lpf > 0) {
      if (ev->ftype == 1) {
        // ladder replaces the biquad wholesale (helpers.mjs:238-240)
        v->ladder_on = true;
        v->lad = (Ladder){0}; v->lad_r = (Ladder){0};
        v->lad_cut = ladder_cut(ev->lpf, sr_f);
        float k = sd_fminf(8.0f, ev->lpq * 0.13f); // worklets.mjs:405
        v->lad_k = k;
        float dr = sd_exp2f(ev->drive * 1.44269504089f); // exp(drive)
        if (dr < 0.1f) dr = 0.1f; else if (dr > 2000.0f) dr = 2000.0f;
        v->lad_drive = dr;
        v->lad_makeup = (1.0f / dr) * sd_fminf(1.75f, 1.0f + k); // worklets.mjs:407-409
      } else {
        biquad_lowpass(&v->lpf, ev->lpf, ev->lpq, sr_f);
        if (ev->ftype == 2) { v->lp24 = true; v->lpf2 = v->lpf; } // 24db = 2nd identical biquad (superdough.mjs:687)
      }
    }
    v->hpf.active = false;
    v->hladder_on = false;
    v->hp24 = false;
    if (ev->hpf > 0) {
      if (ev->ftype == 1) {
        v->hladder_on = true;
        v->hlad = (Ladder){0}; v->hlad_r = (Ladder){0};
        v->hlad_cut = ladder_cut(ev->hpf, sr_f);
        float hk = sd_fminf(8.0f, ev->hpq * 0.13f);
        v->hlad_k = hk;
        float hdr = sd_exp2f(ev->drive * 1.44269504089f);
        if (hdr < 0.1f) hdr = 0.1f; else if (hdr > 2000.0f) hdr = 2000.0f;
        v->hlad_drive = hdr;
        v->hlad_makeup = (1.0f / hdr) * sd_fminf(1.75f, 1.0f + hk);
      } else {
        biquad_highpass(&v->hpf, ev->hpf, ev->hpq, sr_f);
        if (ev->ftype == 2) { v->hp24 = true; v->hpf2 = v->hpf; }
      }
    }
    // FILTER ENVELOPE (createFilter, helpers.mjs:250): active when any lp-env
    // param is set; defaults [0.005, 0.14, 0, 0.1] exp; anchor 0 →
    // fmin = lpf, fmax = 2^|lpenv| · lpf (swap when lpenv < 0)
    v->lp_env_active = false;
    if (ev->lpf > 0 &&
        (!is_nan(ev->lpattack) || !is_nan(ev->lpdecay) || !is_nan(ev->lpsustain) ||
         !is_nan(ev->lprelease) || !is_nan(ev->lpenv))) {
      v->lp_env_active = true;
      v->lp_env = adsr_values(ev->lpattack, ev->lpdecay, ev->lpsustain, ev->lprelease,
                              0.005f, 0.14f, 0.0f, 0.1f);
      float envv = is_nan(ev->lpenv) ? 1.0f : ev->lpenv;
      float envabs = sd_fabsf(envv);
      float fmin = ev->lpf;
      float fmax = sd_fminf(sd_exp2f(envabs) * ev->lpf, 20000.0f);
      if (envv < 0) { float tmpf = fmin; fmin = fmax; fmax = tmpf; }
      v->lp_fmin = fmin;
      v->lp_fmax = fmax;
      v->lp_qdb = ev->lpq;
    }
    v->nz_last = 0;
    for (int nzi = 0; nzi < 7; nzi++) v->nz_b[nzi] = 0;
    v->nz_density = ev->density;
    v->retire_start = ev->retire ? (double)(ev->at_frame > retire_frame ? retire_frame : ev->at_frame) : -1.0;
    v->phaser_on = ev->phaserrate > 0 && ev->phaserdepth > 0;
    if (v->phaser_on) {
      v->ph_center = ev->phasercenter + 282.0f; // fOffset, superdough.mjs:344
      float pq = 2.0f - sd_fminf(sd_fmaxf(ev->phaserdepth * 2.0f, 0.0f), 1.9f);
      v->ph_q = pq;
      v->ph_sweep = ev->phasersweep; // LFO = ±sweep CENTS on the notch
      v->ph_rate = ev->phaserrate;
      v->ph_l = (Biquad){0}; v->ph_r = (Biquad){0};
      biquad_notch(&v->ph_l, v->ph_center, pq, sr_f);
      biquad_notch(&v->ph_r, v->ph_center, pq, sr_f);
    }
    v->vib_hz = ev->vib;
    v->vibmod = ev->vibmod;
    v->lpf_r = v->lpf;
    v->hpf_r = v->hpf;
    v->lpf2_r = v->lpf2;
    v->hpf2_r = v->hpf2;
    // ORBIT + SENDS (M4)
    int ob = ev->orbit;
    if (ob < 0) ob = 0;
    if (ob >= MAX_ORBITS) ob = MAX_ORBITS - 1;
    v->orbit = ob;
    orbits[ob].used = true;
    v->room_send = sd_fminf(sd_fmaxf(ev->room, 0.0f), 1.0f);
    v->delay_send = sd_fminf(sd_fmaxf(ev->delay, 0.0f), 1.0f);
    if (v->room_send > 0) orbit_config_verb(&orbits[ob], ev->roomsize, ev->roomlp);
    if (v->delay_send > 0) orbit_config_delay(&orbits[ob], ev->delaytime, ev->delayfeedback);
    v->shape_on = !is_nan(ev->shape) && ev->shape > 0;
    if (v->shape_on) {
      float sh = ev->shape >= 1.0f ? 1.0f - 4e-10f : ev->shape; // ShapeProcessor clamp
      v->shape_k = (2.0f * sh) / (1.0f - sh);
      float pg = ev->shapevol;
      v->shapevol = sd_fminf(sd_fmaxf(pg, 0.001f), 1.0f);
    }
    v->crush = ev->crush;
    v->coarse = ev->coarse >= 2 ? (int)ev->coarse : 0;
    v->coarse_ctr = 0;
    v->coarse_hold_l = v->coarse_hold_r = 0;
    v->cutkill = false;
    v->cut_group = ev->cut;
    v->vid = ++vid_ctr;
    if (ev->cut >= 0 && ev->cut < MAX_CUT_GROUPS) {
      int prev = cut_last[ev->cut];
      if (prev >= 0 && prev < MAX_VOICES && voices[prev].active && &voices[prev] != v &&
          voices[prev].vid == cut_last_vid[ev->cut]) {
        voices[prev].cutkill = true;                 // 1→0 over 10ms from OUR start
        voices[prev].cutkill_at = ev->at_frame;      // (sampler.mjs:344-345)
      }
      cut_last[ev->cut] = (int)(v - voices);
      cut_last_vid[ev->cut] = v->vid;
    }
    // a ducker fires its dip on the TARGET orbits at its own start time
    for (int di = 0; di < ev->duck_n; di++) {
      int tgt = ev->duck_targets[di];
      if (tgt >= 0 && tgt < MAX_ORBITS) {
        orbits[tgt].used = true;
        orbit_duck(&orbits[tgt], ev->at_frame, ev->duckonset, ev->duckattack, ev->duckdepth);
      }
    }
    if (ev->src == SRC_SAMPLE) {
      if (ev->sample_id < 0 || ev->sample_id >= MAX_SAMPLES || samples[ev->sample_id].frames == 0 ||
          !samples[ev->sample_id].ready) {
        v->active = false;
        return;
      }
      const Sample *sm = &samples[ev->sample_id];
      v->pcm = arena_base + sm->offset;
      v->pcm_frames = sm->frames;
      v->pcm_channels = sm->channels;
      double rate = ev->speed < 0 ? -ev->speed : ev->speed; // |speed| (sampler.mjs:36)
      if (ev->speed < 0) rate = -rate; // REVERSE: read backwards
      v->rate = rate;
      v->base_rate = rate;
      v->pos = ev->speed < 0
                 ? (double)ev->endf * (double)sm->frames - 1.0 // reverse starts at the end
                 : (double)ev->begin * (double)sm->frames;
      v->end_frame = (double)ev->endf * (double)sm->frames;
      v->smp_loop = ev->loopv > 0;
      v->loop_a = (double)ev->loop_begin * (double)sm->frames;
      v->loop_b = (double)ev->loop_end * (double)sm->frames;
    }
    if (ev->src == SRC_SUPERSAW) {
      // synth.mjs:156-170 + worklets.mjs SuperSawOscillatorProcessor
      int u = (int)ev->unison;
      if (u < 1) u = 1;
      if (u > MAX_UNISON) u = MAX_UNISON;
      v->unison = u;
      float fs = is_nan(ev->detune) ? 0.18f : ev->detune; // freqspread, semitones
      // getDetuner (worklets.mjs:38): idx·(fs/(u−1)) − fs/2
      for (int k = 0; k < u; k++) {
        v->fan[k] = u < 2 ? 0.0f : (float)k * (fs / (float)(u - 1)) - fs * 0.5f;
        v->ss_phase[k] = (double)frandf();
      }
      // panspread → alternating √ gains (worklets.mjs:544-548):
      // ps = spread·0.5+0.5; gainL = √(1−ps), gainR = √ps, swapped per voice
      float ps = u > 1 ? sd_fminf(sd_fmaxf(ev->spread, 0.0f), 1.0f) : 0.0f;
      ps = ps * 0.5f + 0.5f;
      v->ss_gl = sd_sqrtf(1.0f - ps);
      v->ss_gr = sd_sqrtf(ps);
      // env max is 0.3·(1/√voices) (synth.mjs:186,195): fold 1/√u into amp
      v->amp /= sd_sqrtf((float)u);
    }
    return;
  }
  // voice pool exhausted — drop (superdough steals; v0.2)
}

// ShapeProcessor port (worklets.mjs:259-295): y = (1+k)x / (1+k|x|), ×shapevol
static inline float shape_drive(float x, float k, float vol) {
  return ((1.0f + k) * x) / (1.0f + k * (x < 0 ? -x : x)) * vol;
}

static inline float sd_roundf(float x) { return (float)(int)(x + (x >= 0 ? 0.5f : -0.5f)); }

// crush (bit reduce) + coarse (sample-hold) + the cut-group 10ms kill —
// applied at the one output chokepoint so every voice path gets them
static inline void voice_fx(Voice *v, double f, float *al, float *ar) {
  if (v->retire_start >= 0) {
    float k = 1.0f - (float)(f - v->retire_start) * retire_inv;
    if (k <= 0) { v->active = false; *al = 0; *ar = 0; return; }
    *al *= k;
    *ar *= k;
  }
  if (v->phaser_on) {
    *al = biquad_run(&v->ph_l, *al);
    *ar = biquad_run(&v->ph_r, *ar);
  }
  if (v->crush >= 1.0f) {
    float x = sd_exp2f(v->crush - 1.0f); // webdirt: round(x·2^(crush−1))/2^(crush−1)
    *al = sd_roundf(*al * x) / x;
    *ar = sd_roundf(*ar * x) / x;
  }
  if (v->coarse >= 2) {
    if (v->coarse_ctr == 0) {
      v->coarse_hold_l = *al;
      v->coarse_hold_r = *ar;
    }
    *al = v->coarse_hold_l;
    *ar = v->coarse_hold_r;
    v->coarse_ctr++;
    if (v->coarse_ctr >= v->coarse) v->coarse_ctr = 0;
  }
  if (v->cutkill) {
    double dt = f - v->cutkill_at;
    if (dt >= 0) {
      float g = 1.0f - (float)(dt / (0.01 * (double)sr_f)); // 10ms linear (sampler.mjs:345)
      if (g <= 0) {
        v->active = false;
        g = 0;
      }
      *al *= g;
      *ar *= g;
    }
  }
}

// a voice's stereo sample lands on its ORBIT: dry + reverb/delay sends
static inline void voice_out(const Voice *v, int i, float al, float ar) {
  Orbit *o = &orbits[v->orbit];
  o->dry[i * OUT_CH] += al;
  o->dry[i * OUT_CH + 1] += ar;
  if (v->room_send > 0) o->vin[i] += (al + ar) * 0.5f * v->room_send;
  if (v->delay_send > 0) {
    o->din[i * OUT_CH] += al * v->delay_send;
    o->din[i * OUT_CH + 1] += ar * v->delay_send;
  }
}

// polyBLEP residual — band-limits saw/square edges like OscillatorNode does.
static inline float polyblep(double t, double dt) {
  if (t < dt) {
    float x = (float)(t / dt);
    return x + x - x * x - 1.0f;
  }
  if (t > 1.0 - dt) {
    float x = (float)((t - 1.0) / dt);
    return x * x + x + x + 1.0f;
  }
  return 0.0f;
}

__attribute__((export_name("sd_dsp"))) void sd_dsp(void) {
  for (int i = 0; i < BLOCK * OUT_CH; i++) out_buf[i] = 0;

  // start due events
  for (int i = 0; i < MAX_EVENTS; i++) {
    if (events[i].used && events[i].at_frame < engine_frame + BLOCK) {
      start_voice(&events[i]);
      events[i].used = false;
    }
  }

  for (int vi = 0; vi < MAX_VOICES; vi++) {
    Voice *v = &voices[vi];
    if (!v->active) continue;
    for (int i = 0; i < BLOCK; i++) {
      double f = engine_frame + i;
      if (f < v->start_frame) continue;
      float tt = (float)((f - v->start_frame) / (double)sr_f);
      if (tt >= v->end) { v->active = false; break; }

      // CONTROL RATE (every 16 samples): vib detune + filter-envelope sweep —
      // WebAudio automates these a-rate; 16-sample control is inaudible for
      // exponential glides and keeps coefficients cheap
      if (((int)(f - v->start_frame) & 15) == 0) {
        if (v->vib_hz > 0) {
          float cents = sd_sinf(TWO_PI * v->vib_hz * tt) * v->vibmod * 100.0f;
          float mult = sd_exp2f(cents / 1200.0f);
          if (v->src == SRC_SAMPLE) v->rate = v->base_rate * (double)mult;
          else v->phase_inc = (double)(v->base_freq * mult) / (double)sr_f;
        }
        if (v->phaser_on) {
          // LFO = unipolar TRIANGLE (waveshapes.tri, shape index 0), phase
          // aligned to ABSOLUTE time: phase0 = frac(begin·freq) (LFOProcessor)
          float tabs = (float)((v->start_frame / (double)sr_f)) + tt;
          float ph = tabs * v->ph_rate;
          ph -= (float)(int)ph;
          float tri01 = ph < 0.5f ? ph * 2.0f : (1.0f - ph) * 2.0f;
          float cents = (tri01 - 0.5f) * 2.0f * v->ph_sweep;
          float fq = v->ph_center * sd_exp2f(cents / 1200.0f);
          biquad_notch(&v->ph_l, fq, v->ph_q, sr_f);
          biquad_notch(&v->ph_r, fq, v->ph_q, sr_f);
        }
        if (v->lp_env_active) {
          float fenv = adsr_exp_at(&v->lp_env, tt, v->dur, v->lp_fmin, v->lp_fmax);
          if (v->ladder_on) {
            v->lad_cut = ladder_cut(fenv, sr_f); // env rides the worklet's frequency param
          } else {
            biquad_lowpass_coeffs(&v->lpf, fenv, v->lp_qdb, sr_f);
            if (v->lp24) {
              biquad_lowpass_coeffs(&v->lpf2, fenv, v->lp_qdb, sr_f);
              if (v->src == SRC_SUPERSAW) biquad_lowpass_coeffs(&v->lpf2_r, fenv, v->lp_qdb, sr_f);
            }
            if (v->src == SRC_SUPERSAW) biquad_lowpass_coeffs(&v->lpf_r, fenv, v->lp_qdb, sr_f);
          }
        }
      }
      if (v->src == SRC_SAMPLE) {
        float xs = 0, xs_r = 0;
        bool in_range = v->rate < 0
                          ? v->pos >= 0 && v->pos >= (double)0 && v->pos < (double)v->pcm_frames
                          : v->pos < v->end_frame && v->pos < (double)v->pcm_frames;
        if (in_range) {
          int i0 = (int)v->pos;
          float fr = (float)(v->pos - (double)i0);
          int i1 = i0 + 1 < v->pcm_frames ? i0 + 1 : i0;
          if (v->pcm_channels == 1) {
            xs = v->pcm[i0] + (v->pcm[i1] - v->pcm[i0]) * fr;
            xs_r = xs;
          } else {
            xs = v->pcm[i0 * 2] + (v->pcm[i1 * 2] - v->pcm[i0 * 2]) * fr;
            xs_r = v->pcm[i0 * 2 + 1] + (v->pcm[i1 * 2 + 1] - v->pcm[i0 * 2 + 1]) * fr;
          }
          v->pos += v->rate;
          if (v->smp_loop && v->pos >= v->loop_b) v->pos = v->loop_a + (v->pos - v->loop_b);
        } else if (tt >= v->end) {
          v->active = false;
          break;
        }
        float env = adsr_at(&v->env, tt, v->dur);
        float al = xs * v->amp * env;
        float ar = xs_r * v->amp * env;
        if (v->ladder_on) {
          al = ladder_run(&v->lad, al, v->lad_cut, v->lad_k, v->lad_drive) * v->lad_makeup;
          ar = v->pcm_channels == 1 ? al : ladder_run(&v->lad_r, ar, v->lad_cut, v->lad_k, v->lad_drive) * v->lad_makeup;
        } else if (v->lpf.active) {
          al = biquad_run(&v->lpf, al);
          ar = v->pcm_channels == 1 ? al : biquad_run(&v->lpf_r, ar);
          if (v->lp24) { al = biquad_run(&v->lpf2, al); ar = v->pcm_channels == 1 ? al : biquad_run(&v->lpf2_r, ar); }
        }
        if (v->hladder_on) {
          al = ladder_run(&v->hlad, al, v->hlad_cut, v->hlad_k, v->hlad_drive) * v->hlad_makeup;
          ar = v->pcm_channels == 1 ? al : ladder_run(&v->hlad_r, ar, v->hlad_cut, v->hlad_k, v->hlad_drive) * v->hlad_makeup;
        } else if (v->hpf.active) {
          al = biquad_run(&v->hpf, al);
          ar = v->pcm_channels == 1 ? al : biquad_run(&v->hpf_r, ar);
          if (v->hp24) { al = biquad_run(&v->hpf2, al); ar = v->pcm_channels == 1 ? al : biquad_run(&v->hpf2_r, ar); }
        }
        if (v->shape_on) {
          al = shape_drive(al, v->shape_k, v->shapevol);
          ar = shape_drive(ar, v->shape_k, v->shapevol);
        }
        al *= v->postgain;
        ar *= v->postgain;
        if (v->pan_set) {
          if (v->pcm_channels == 1) {
            // MONO source → StereoPanner MONO equal-power law (spec): the
            // stereo fold-law here read +3.64dB hot on every panned gm layer
            al *= v->pan_l;
            ar *= v->pan_r;
          } else {
            float px = v->pan_x;
            if (px > 0) {
              float ang = px * (PI_F * 0.5f);
              float nl = al * sd_cosf(ang);
              float nr = ar + al * sd_sinf(ang);
              al = nl; ar = nr;
            } else if (px < 0) {
              float ang = -px * (PI_F * 0.5f);
              float nr = ar * sd_cosf(ang);
              float nl = al + ar * sd_sinf(ang);
              al = nl; ar = nr;
            }
          }
        }
        voice_fx(v, engine_frame + i, &al, &ar);
        voice_out(v, i, al, ar);
        continue;
      }
      double t = v->phase;
      double dt = v->phase_inc;
      float s;
      if (v->src == SRC_SUPERSAW) {
        // stereo path: N polyBLEP saws (worklets.mjs sawblep), alternating
        // √panspread gains, then env/filters/pan on BOTH channels
        float suml = 0, sumr = 0;
        for (int k = 0; k < v->unison; k++) {
          double fq = (double)v->phase_inc * sd_exp2f(v->fan[k] / 12.0f);
          double ph = v->ss_phase[k];
          float x = (float)(2.0 * ph - 1.0) - polyblep(ph, fq);
          float glk = (k & 1) ? v->ss_gr : v->ss_gl;
          float grk = (k & 1) ? v->ss_gl : v->ss_gr;
          suml += x * glk;
          sumr += x * grk;
          ph += fq;
          if (ph >= 1.0) ph -= 1.0;
          v->ss_phase[k] = ph;
        }
        float env = adsr_at(&v->env, tt, v->dur);
        float al = suml * v->amp * env;
        float ar = sumr * v->amp * env;
        if (v->ladder_on) {
          al = ladder_run(&v->lad, al, v->lad_cut, v->lad_k, v->lad_drive) * v->lad_makeup;
          ar = ladder_run(&v->lad_r, ar, v->lad_cut, v->lad_k, v->lad_drive) * v->lad_makeup;
        } else if (v->lpf.active) {
          al = biquad_run(&v->lpf, al); ar = biquad_run(&v->lpf_r, ar);
          if (v->lp24) { al = biquad_run(&v->lpf2, al); ar = biquad_run(&v->lpf2_r, ar); }
        }
        if (v->hladder_on) {
          al = ladder_run(&v->hlad, al, v->hlad_cut, v->hlad_k, v->hlad_drive) * v->hlad_makeup;
          ar = ladder_run(&v->hlad_r, ar, v->hlad_cut, v->hlad_k, v->hlad_drive) * v->hlad_makeup;
        } else if (v->hpf.active) {
          al = biquad_run(&v->hpf, al); ar = biquad_run(&v->hpf_r, ar);
          if (v->hp24) { al = biquad_run(&v->hpf2, al); ar = biquad_run(&v->hpf2_r, ar); }
        }
        if (v->shape_on) {
          al = shape_drive(al, v->shape_k, v->shapevol);
          ar = shape_drive(ar, v->shape_k, v->shapevol);
        }
        al *= v->postgain;
        ar *= v->postgain;
        if (v->pan_set) {
          // StereoPanner STEREO law (spec): x>0 folds L into R, x<0 folds R into L
          float x = v->pan_x;
          if (x > 0) {
            float ang = x * (PI_F * 0.5f);
            float nl = al * sd_cosf(ang);
            float nr = ar + al * sd_sinf(ang);
            al = nl; ar = nr;
          } else if (x < 0) {
            float ang = -x * (PI_F * 0.5f);
            float nr = ar * sd_cosf(ang);
            float nl = al + ar * sd_sinf(ang);
            al = nl; ar = nr;
          }
        }
        voice_fx(v, engine_frame + i, &al, &ar);
        voice_out(v, i, al, ar);
        continue;
      }
      switch (v->src) {
        case SRC_SINE: s = sd_sinf((float)(t * (double)TWO_PI)); break;
        // noise.mjs getNoiseBuffer formulas, per-sample
        case SRC_WHITE: s = nz_rand(); break;
        case SRC_BROWN: {
          float w = nz_rand();
          s = (v->nz_last + 0.02f * w) / 1.02f;
          v->nz_last = s;
          break;
        }
        case SRC_PINK: {
          float w = nz_rand();
          float *b = v->nz_b;
          b[0] = 0.99886f * b[0] + w * 0.0555179f;
          b[1] = 0.99332f * b[1] + w * 0.0750759f;
          b[2] = 0.969f * b[2] + w * 0.153852f;
          b[3] = 0.8665f * b[3] + w * 0.3104856f;
          b[4] = 0.55f * b[4] + w * 0.5329522f;
          b[5] = -0.7616f * b[5] - w * 0.016898f;
          s = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362f) * 0.11f;
          b[6] = w * 0.115926f;
          break;
        }
        case SRC_CRACKLE: {
          float u = nz_rand() * 0.5f + 0.5f;
          s = u < v->nz_density * 0.01f ? nz_rand() : 0.0f;
          break;
        }
        case SRC_SAW: {
          float pos = (float)(t * (double)WT_LEN);
          int i0 = (int)pos;
          float fr = pos - (float)i0;
          const float *tab = wt_saw[v->wt_lvl];
          s = tab[i0] + (tab[i0 + 1] - tab[i0]) * fr;
          break;
        }
        case SRC_SQUARE: {
          s = t < 0.5 ? 1.0f : -1.0f;
          s += polyblep(t, dt);
          double t2 = t + 0.5; if (t2 >= 1.0) t2 -= 1.0;
          s -= polyblep(t2, dt);
          break;
        }
        default: { // triangle — SPEC phase: starts 0 rising, peak +1 at t=0.25
          // (the sin-series limit; the old −1-start was 90° late and shifted
          // steady-state sums with octave-related saw partials)
          float ph = (float)t;
          s = ph < 0.25f ? (4.0f * ph) : ph < 0.75f ? (2.0f - 4.0f * ph) : (4.0f * ph - 4.0f);
        }
      }
      v->phase += dt;
      if (v->phase >= 1.0) v->phase -= 1.0;

      float env = adsr_at(&v->env, tt, v->dur);
      float x = s * v->amp * env;
      if (v->ladder_on) x = ladder_run(&v->lad, x, v->lad_cut, v->lad_k, v->lad_drive) * v->lad_makeup;
      else if (v->lpf.active) {
        x = biquad_run(&v->lpf, x);
        if (v->lp24) x = biquad_run(&v->lpf2, x);
      }
      if (v->hladder_on) x = ladder_run(&v->hlad, x, v->hlad_cut, v->hlad_k, v->hlad_drive) * v->hlad_makeup;
      else if (v->hpf.active) {
        x = biquad_run(&v->hpf, x);
        if (v->hp24) x = biquad_run(&v->hpf2, x);
      }
      if (v->shape_on) x = shape_drive(x, v->shape_k, v->shapevol);
      x *= v->postgain;
      float al = x * v->pan_l, ar = x * v->pan_r;
      voice_fx(v, engine_frame + i, &al, &ar);
      voice_out(v, i, al, ar);
    }
  }
  // ---- BUS PASS: per used orbit — delay ring, FDN reverb, duck, mix ----
  for (int oi = 0; oi < MAX_ORBITS; oi++) {
    Orbit *o = &orbits[oi];
    if (!o->used) continue;
    if (o->verb_on) {
      // reverb retunes GLIDE (~50ms settle at 375 blocks/s) — the crackle
      // war's law extended to the FDN: buses ramp, never step (and NEVER
      // clear a ringing line)
      for (int gi = 0; gi < FDN_LINES; gi++)
        o->fdn_g[gi] += (o->fdn_g_tgt[gi] - o->fdn_g[gi]) * 0.05f;
      o->damp_a += (o->damp_a_tgt - o->damp_a) * 0.05f;
      o->wet_scale += (o->wet_tgt - o->wet_scale) * 0.05f;
    }
    for (int i = 0; i < BLOCK; i++) {
      float dl = 0, dr = 0;
      if (o->delay_on) {
        // clickless moves (the crackle war's law: buses RAMP, never step)
        o->dt_cur += (o->dt_tgt - o->dt_cur) * 0.0005f;
        o->fb_cur += (o->fb_tgt - o->fb_cur) * 0.001f;
        int dsamp = (int)(o->dt_cur * sr_f);
        if (dsamp >= o->dl_len) dsamp = o->dl_len - 1;
        int rd = o->dl_pos - dsamp;
        if (rd < 0) rd += o->dl_len;
        if (o->dl_fill > dsamp) { // gate: only samples we actually wrote
          dl = o->dl_l[rd];
          dr = o->dl_r[rd];
        }
        if (o->dl_fill < o->dl_len) o->dl_fill++;
        o->dl_l[o->dl_pos] = undenorm(o->din[i * OUT_CH] + dl * o->fb_cur);
        o->dl_r[o->dl_pos] = undenorm(o->din[i * OUT_CH + 1] + dr * o->fb_cur);
        o->dl_pos++;
        if (o->dl_pos >= o->dl_len) o->dl_pos = 0;
      }
      float wl = 0, wr = 0;
      if (o->verb_on) {
        float x = o->vin[i];
        // INPUT DIFFUSION: two series allpasses (g .62) turn each discrete
        // hit into a dense burst before the FDN — echo density up, flutter
        // and modal ping down. Prime lengths 259/611 samples.
        {
          static const int APL[2] = { 259, 611 };
          for (int a = 0; a < 2; a++) {
            float *buf = o->ap[a];
            int pos = o->ap_pos[a];
            float d = buf[pos];
            float in = x + d * 0.62f;
            buf[pos] = undenorm(in);
            x = d - in * 0.62f;
            pos++;
            if (pos >= APL[a]) pos = 0;
            o->ap_pos[a] = pos;
          }
        }
        float ys[FDN_LINES];
        float sum = 0;
        for (int k = 0; k < FDN_LINES; k++) {
          if (k < 2) {
            // MODULATED READ on TWO lines only (the Dattorro recipe): ±4
            // samples at ~0.5/0.7Hz, linear interp. Two moving lines smear
            // the whole network's modes (everything is cross-coupled), while
            // the interp's tiny per-pass loss touches only 2 of 8 paths —
            // stable, and the tail keeps its energy. (All-8 modulation ate
            // 4dB; allpass interp in a time-varying feedback loop BLEW UP.)
            o->mod_ph[k] += (0.5f + 0.2f * (float)k) * (TWO_PI / sr_f);
            if (o->mod_ph[k] > TWO_PI) o->mod_ph[k] -= TWO_PI;
            float md = (sd_sinf(o->mod_ph[k]) + 1.0f) * 4.0f; // 0..8 samples
            int L = o->fdn_len[k];
            float rf = (float)o->fdn_pos[k] - md;
            if (rf < 0) rf += (float)L;
            int r0 = (int)rf;
            float fr = rf - (float)r0;
            int r1 = r0 + 1;
            if (r1 >= L) r1 = 0;
            ys[k] = o->fdn_fill >= L
                      ? o->fdn[k][r0] + (o->fdn[k][r1] - o->fdn[k][r0]) * fr
                      : 0.0f;
          } else {
            ys[k] = o->fdn_fill >= o->fdn_len[k] ? o->fdn[k][o->fdn_pos[k]] : 0.0f;
          }
          sum += ys[k];
        }
        float h = sum * 0.25f; // Householder: y − (2/N)Σy, N=8
        for (int k = 0; k < FDN_LINES; k++) {
          float inj = x + ys[k] - h;
          o->damp[k] = undenorm(o->damp[k] + o->damp_a * (inj - o->damp[k])); // in-loop damping
          o->fdn[k][o->fdn_pos[k]] = undenorm(o->damp[k] * o->fdn_g[k]);
          o->fdn_pos[k]++;
          if (o->fdn_pos[k] >= o->fdn_len[k]) o->fdn_pos[k] = 0;
        }
        if (o->fdn_fill < FDN_MAX) o->fdn_fill++; // per-sample; caps at max line
        // ± signs decorrelate L/R; wet_scale = level calibration + the
        // convolver's energy normalization (orbit_config_verb)
        wl = (ys[0] - ys[1] + ys[2] - ys[3] + ys[4] - ys[5] + ys[6] - ys[7]) * o->wet_scale;
        wr = (ys[0] + ys[1] - ys[2] - ys[3] + ys[4] + ys[5] - ys[6] - ys[7]) * o->wet_scale;
      }
      float g = orbit_duck_gain(o, engine_frame + i);
      o->out_g += (o->out_g_tgt - o->out_g) * 0.002f; // kill ramp (~10ms tau)
      g *= o->out_g;
      out_buf[i * OUT_CH] += (o->dry[i * OUT_CH] + dl + wl) * g;
      out_buf[i * OUT_CH + 1] += (o->dry[i * OUT_CH + 1] + dr + wr) * g;
      o->dry[i * OUT_CH] = 0;
      o->dry[i * OUT_CH + 1] = 0;
      o->vin[i] = 0;
      o->din[i * OUT_CH] = 0;
      o->din[i * OUT_CH + 1] = 0;
    }
  }
  engine_frame += BLOCK;
}
