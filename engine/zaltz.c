// ZALTZ — Klappn's own synthesis engine. C99, freestanding, wasm32.
//
// Copyright (C) 2026 Eliyahu Moshe Leinkram
// Licensed under the GNU Affero General Public License, version 3 or later.
// See LICENSE and NOTICE.md.
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
// PHYSICAL slots. The MUSICAL cap is POLY_CAP below — superdough's
// maxPolyphony semantics need headroom above it, because a stolen voice keeps
// sounding (fading) for 0.25s after it is stolen.
#define MAX_VOICES 320
// superdough DEFAULT_MAX_POLYPHONY (superdough.mjs:36) — the same 128 the
// client re-asserts on every play (strudel-client maxVoices).
#define POLY_CAP 128
// superdough steals by ramping the victim to 0 over 0.25s (superdough.mjs:527
// `const endTime = t + 0.25`).
#define STEAL_FADE 0.25f
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
  // Taylor deg-11 on [−π/2, π/2]: max err ~6e-8 — float precision. (It was
  // deg-7, ~1e-4 at the quadrant edge, which is exactly where sd_cosf lands.)
  float r = y * (1.0f + y2 * (-0.16666666667f + y2 * (0.0083333333333f + y2 * (-0.00019841269841f +
                y2 * (2.7557319224e-6f + y2 * -2.5052108385e-8f)))));
  // odd k flips the sign
  return ((int)k & 1) ? -r : r;
}
static float sd_cosf(float x) { return sd_sinf(x + 1.57079632679f); }

// DOUBLE sin/cos for FILTER COEFFICIENTS (2026-09-23). A biquad's low end
// lives in 1 − cos(w0), which at 40Hz/48k is 1.4e-5 — smaller than the float
// sine's old error. Every filter under ~400Hz was mistuned: hpf(100) passed
// its own cutoff at −6dB (Web Audio: 0dB), lpf(150) rang +2.9dB. Browsers
// compute biquads in double (Chromium's Biquad: double coefficients + state);
// so do we. Taylor deg-17 on [−π/2, π/2] after exact-ish reduction: ~1e-14.
static double sd_sin_d(double x) {
  const double PI_D = 3.14159265358979323846;
  double k = (double)(long long)(x * (1.0 / PI_D) + (x >= 0 ? 0.5 : -0.5));
  double y = x - k * PI_D;
  double y2 = y * y;
  double r = y * (1.0 + y2 * (-1.0 / 6 + y2 * (1.0 / 120 + y2 * (-1.0 / 5040 + y2 * (1.0 / 362880 +
             y2 * (-1.0 / 39916800 + y2 * (1.0 / 6227020800.0 + y2 * (-1.0 / 1307674368000.0 +
             y2 * (1.0 / 355687428096000.0)))))))));
  return ((long long)k & 1) ? -r : r;
}
static double sd_cos_d(double x) { return sd_sin_d(x + 1.57079632679489661923); }

// exp2: split int/frac, 5th-order poly on [0,1) (coefficient-time only).
static float sd_exp2f(float x) {
  if (x < -126.0f) return 0.0f;
  int ip = (int)x;
  if (x < 0 && x != (float)ip) ip -= 1;
  float fr = x - (float)ip;
  // Taylor to fr^7 (was fr^5: 0.15 cents sharp-to-flat across every octave
  // — inaudible, but a systematic tuning error has no place in the engine)
  float p = 1.0f + fr * (0.69314718f + fr * (0.24022651f + fr * (0.05550411f + fr * (0.00961813f +
            fr * (0.00133336f + fr * (1.5403530e-4f + fr * 1.5252734e-5f))))));
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
  // ln m = 2·atanh(s), s = (m−1)/(m+1) ∈ [0, 1/3]: odd series to s^9,
  // err ~1e-6 (was a quartic, ~2e-4 — it shaped every exponential ramp)
  float sv = (m - 1.0f) / (m + 1.0f), s2 = sv * sv;
  float ln = 2.0f * sv * (1.0f + s2 * (1.0f / 3 + s2 * (1.0f / 5 + s2 * (1.0f / 7 + s2 * (1.0f / 9)))));
  return (float)e + ln * 1.44269504089f;
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
static inline float sd_clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
// e^x − 1 / ln(1+x) / true tanh — the distortion family (helpers.mjs:496-567)
// uses Math.expm1/log1p/tanh; the ladder's fast_tanh is an approximation
// calibrated for THAT filter, not for waveshaping character.
static inline float sd_expm1f(float x) { return sd_exp2f(x * 1.442695041f) - 1.0f; }
static inline float sd_log1pf(float x) { return sd_log2f(1.0f + x) * 0.69314718056f; }
static inline float sd_tanh_true(float x) {
  if (x > 9.0f) return 1.0f;
  if (x < -9.0f) return -1.0f;
  float e = sd_exp2f(2.0f * x * 1.442695041f); // e^(2x)
  return (e - 1.0f) / (e + 1.0f);
}
static inline float sd_floorf(float x) { float t = (float)(int)x; return (x < 0 && t != x) ? t - 1.0f : t; }

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
  double b0, b1, b2, a1, a2; // DOUBLE, like Chromium's Biquad — poles near 1
  double x1, x2, y1, y2;     // at low cutoffs need the precision
  bool active;
} Biquad;

static void biquad_lowpass(Biquad *f, float freq, float qdb, float sr) {
  double w0 = 6.283185307179586 * (double)sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / (double)sr;
  double q = (double)sd_fmaxf(sd_pow10f(qdb / 20.0f), 0.0001f);
  double alpha = sd_sin_d(w0) / (2.0 * q);
  double cosw = sd_cos_d(w0);
  double a0 = 1.0 + alpha;
  f->b0 = ((1.0 - cosw) / 2.0) / a0;
  f->b1 = (1.0 - cosw) / a0;
  f->b2 = f->b0;
  f->a1 = (-2.0 * cosw) / a0;
  f->a2 = (1.0 - alpha) / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0;
  f->active = true;
}
// notch — WebAudio Q is LINEAR here (only lp/hp speak dB); RBJ cookbook
static void biquad_notch(Biquad *f, float freq, float q, float sr) {
  double w0 = 6.283185307179586 * (double)sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / (double)sr;
  if (q < 0.0001f) q = 0.0001f;
  double alpha = sd_sin_d(w0) / (2.0 * (double)q);
  double cosw = sd_cos_d(w0);
  double a0 = 1.0 + alpha;
  double x1 = f->x1, x2 = f->x2, y1 = f->y1, y2 = f->y2; // retune keeps state
  f->b0 = 1.0 / a0;
  f->b1 = (-2.0 * cosw) / a0;
  f->b2 = 1.0 / a0;
  f->a1 = (-2.0 * cosw) / a0;
  f->a2 = (1.0 - alpha) / a0;
  f->x1 = x1; f->x2 = x2; f->y1 = y1; f->y2 = y2;
  f->active = true;
}

static void biquad_highpass(Biquad *f, float freq, float qdb, float sr) {
  double w0 = 6.283185307179586 * (double)sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / (double)sr;
  double q = (double)sd_fmaxf(sd_pow10f(qdb / 20.0f), 0.0001f);
  double alpha = sd_sin_d(w0) / (2.0 * q);
  double cosw = sd_cos_d(w0);
  double a0 = 1.0 + alpha;
  f->b0 = ((1.0 + cosw) / 2.0) / a0;
  f->b1 = (-(1.0 + cosw)) / a0;
  f->b2 = f->b0;
  f->a1 = (-2.0 * cosw) / a0;
  f->a2 = (1.0 - alpha) / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0;
  f->active = true;
}
static inline float biquad_run(Biquad *f, float xin) {
  double x = (double)xin;
  double y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
  if (y < 1e-30 && y > -1e-30) y = 0; // denormal flush (the crackle-killer law)
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;
  return (float)y;
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

// getParamADSR 'exponential' (helpers.mjs:40-99), evaluated analytically. The
// SCHEDULE is superdough's, not an ideal curve: the ramp TARGETS come from a
// LINEAR interpolation (envValAtTime), and each segment glides to its target
// exponentially (v1·(v2/v1)^u). That difference is audible exactly where it
// matters — a note shorter than attack+decay (a pluck, a hat with a filter
// env): superdough ramps to the linearly-interpolated value at the note's
// end, an ideal exponential curve would sit far lower. min/max of exactly 0
// become 0.001 (the curve can't touch zero).
static inline float exp_ramp(float v1, float v2, float u) {
  if (u <= 0) return v1;
  if (u >= 1) return v2;
  if (v1 <= 0 || v2 <= 0) return v1; // spec: no exponential ramp through 0 — hold
  return v1 * sd_exp2f(u * sd_log2f(v2 / v1));
}
static float adsr_exp_at(const Adsr *e, float tt, float dur, float vmin, float vmax) {
  if (vmin == 0) vmin = 0.001f;
  if (vmax == 0) vmax = 0.001f;
  const float a = e->attack, d = e->decay;
  const float susv = vmin + e->sustain * (vmax - vmin);
  // envValAtTime: the linear shape, used only as ramp TARGETS
  #define ENV_LIN(T) ({ float _t = (T); float _v = (a > _t) ? _t * ((vmax - vmin) / a) + vmin \
                                                        : (_t - a) * ((susv - vmax) / d) + vmax; \
                        _v == 0 ? 0.001f : _v; })
  if (tt < 0) return vmin;
  float held; // the value the schedule reaches at `dur`
  if (a > dur) { // attack only, cut at the end
    held = ENV_LIN(dur);
    if (tt < dur) return exp_ramp(vmin, held, tt / dur);
  } else if (a + d > dur) { // attack, then a decay cut at the end
    float top = ENV_LIN(a);
    held = ENV_LIN(dur);
    if (tt < a) return exp_ramp(vmin, top, tt / a);
    if (tt < dur) return exp_ramp(top, held, (tt - a) / (dur - a));
  } else { // attack, decay, hold at sustain
    float top = ENV_LIN(a);
    float sv = ENV_LIN(a + d);
    held = susv;
    if (tt < a) return exp_ramp(vmin, top, tt / a);
    if (tt < a + d) return exp_ramp(top, sv, (tt - a) / d);
    if (tt < dur) return susv;
  }
  #undef ENV_LIN
  return exp_ramp(held, vmin, (tt - dur) / e->release); // release → min
}

// ---------- THE FILTER FAMILY (superdough createFilter, helpers.mjs:219) -----
// lpf, hpf and bpf are ONE mechanism upstream: a biquad (Q in dB for lp/hp,
// LINEAR for bp — Web Audio), or under ftype "ladder" the LadderProcessor —
// which is a LOWPASS topology whatever the type (the corpus was authored with
// that quirk); ftype 24db cascades a second identical stage; each has the
// same exponential envelope with the shared fanchor. Order: lp → hp → bp.
enum { FT_LP = 0, FT_HP, FT_BP };
typedef struct {
  bool on;
  int type;
  bool ladder, x24;
  float q;
  Biquad f[2][2]; // [stage][channel]
  Ladder lad[2];  // [channel]
  float lad_cut, lad_k, lad_drive, lad_makeup;
  bool env_on;
  Adsr env;
  float fmin, fmax;
} Filt;

static void biquad_bandpass(Biquad *f, float freq, float q, float sr) {
  // Web Audio bandpass (constant 0dB peak): alpha = sin(w0)/(2Q), Q LINEAR
  double w0 = 6.283185307179586 * (double)sd_fminf(sd_fmaxf(freq, 10.0f), sr * 0.49f) / (double)sr;
  if (q < 0.0001f) q = 0.0001f;
  double alpha = sd_sin_d(w0) / (2.0 * (double)q);
  double cosw = sd_cos_d(w0);
  double a0 = 1.0 + alpha;
  f->b0 = alpha / a0;
  f->b1 = 0;
  f->b2 = -alpha / a0;
  f->a1 = (-2.0 * cosw) / a0;
  f->a2 = (1.0 - alpha) / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0;
  f->active = true;
}
// set a biquad by type; keep = retune under a sounding signal (state kept)
static void biquad_set(Biquad *f, int type, float freq, float q, float sr, bool keep) {
  double x1 = f->x1, x2 = f->x2, y1 = f->y1, y2 = f->y2;
  if (type == FT_HP) biquad_highpass(f, freq, q, sr);
  else if (type == FT_BP) biquad_bandpass(f, freq, q, sr);
  else biquad_lowpass(f, freq, q, sr);
  if (keep) { f->x1 = x1; f->x2 = x2; f->y1 = y1; f->y2 = y2; }
}
static void filt_setup(Filt *F, int type, float freq, float q, int ftype, float drive,
                       float a, float d, float s, float r, float envv, float anchor, float sr) {
  F->on = !is_nan(freq) && freq > 0;
  if (!F->on) return;
  F->type = type;
  F->q = q;
  F->ladder = ftype == 1;
  F->x24 = ftype == 2;
  if (F->ladder) {
    F->lad[0] = (Ladder){0}; F->lad[1] = (Ladder){0};
    F->lad_cut = ladder_cut(freq, sr);
    F->lad_k = sd_fminf(8.0f, q * 0.13f); // worklets.mjs:404
    float dr = sd_exp2f(drive * 1.44269504089f); // exp(drive), clamped .1–2000
    F->lad_drive = dr < 0.1f ? 0.1f : dr > 2000.0f ? 2000.0f : dr;
    F->lad_makeup = (1.0f / F->lad_drive) * sd_fminf(1.75f, 1.0f + F->lad_k);
  } else {
    biquad_set(&F->f[0][0], type, freq, q, sr, false);
    F->f[0][1] = F->f[0][0];
    F->f[1][0] = F->f[0][0];
    F->f[1][1] = F->f[0][0];
  }
  // the envelope is live when ANY of its five params was given
  F->env_on = !is_nan(a) || !is_nan(d) || !is_nan(s) || !is_nan(r) || !is_nan(envv);
  if (F->env_on) {
    F->env = adsr_values(a, d, s, r, 0.005f, 0.14f, 0.0f, 0.1f);
    float e = is_nan(envv) ? 1.0f : envv;      // nanFallback(env, 1)
    float an = is_nan(anchor) ? 0.0f : anchor; // nanFallback(anchor, 0)
    float ea = sd_fabsf(e), off = ea * an;
    float mn = sd_clampf(sd_exp2f(-off) * freq, 0.0f, 20000.0f);
    float mx = sd_clampf(sd_exp2f(ea - off) * freq, 0.0f, 20000.0f);
    if (e < 0) { float t = mn; mn = mx; mx = t; }
    F->fmin = mn;
    F->fmax = mx;
  }
}
static void filt_env(Filt *F, float tt, float dur, float sr) {
  if (!F->on || !F->env_on) return;
  float fq = adsr_exp_at(&F->env, tt, dur, F->fmin, F->fmax);
  if (F->ladder) { F->lad_cut = ladder_cut(fq, sr); return; }
  // ONE coefficient solve (the double trig is the cost), copied to every
  // stage/channel — their running state stays their own
  Biquad c;
  biquad_set(&c, F->type, fq, F->q, sr, false);
  for (int st = 0; st < (F->x24 ? 2 : 1); st++)
    for (int ch = 0; ch < 2; ch++) {
      Biquad *b = &F->f[st][ch];
      b->b0 = c.b0; b->b1 = c.b1; b->b2 = c.b2; b->a1 = c.a1; b->a2 = c.a2;
    }
}
static inline float filt_run(Filt *F, float x, int ch) {
  if (F->ladder) return ladder_run(&F->lad[ch], x, F->lad_cut, F->lad_k, F->lad_drive) * F->lad_makeup;
  x = biquad_run(&F->f[0][ch], x);
  return F->x24 ? biquad_run(&F->f[1][ch], x) : x;
}

// VOWEL (superdough vowel.mjs VowelNode, from webdirt): five PARALLEL
// band-passes (Q linear), each through its formant gain, summed ×8 (makeup).
// Rows in the bridge's index order; the unicode aliases resolve in the bridge.
#define N_VOWELS 15
static const float VOWEL_F[N_VOWELS][5] = {
  {660, 1120, 2750, 3000, 3350}, {440, 1800, 2700, 3000, 3300}, {270, 1850, 2900, 3350, 3590},
  {430, 820, 2700, 3000, 3300},  {370, 630, 2750, 3000, 3400},  {650, 1515, 2400, 3000, 3350},
  {560, 900, 2570, 3000, 3300},  {500, 1430, 2300, 3000, 3300}, {250, 1750, 2150, 3200, 3300},
  {400, 1460, 2400, 3000, 3300}, {600, 1250, 2100, 3100, 3500}, {500, 1240, 2280, 3000, 3500},
  {600, 1480, 2450, 3200, 3300}, {700, 1050, 2500, 3000, 3300}, {500, 1080, 2350, 3000, 3300}};
static const float VOWEL_G[N_VOWELS][5] = {
  {1, 0.5012f, 0.0708f, 0.0631f, 0.0126f}, {1, 0.1995f, 0.1259f, 0.1f, 0.1f},
  {1, 0.0631f, 0.0631f, 0.0158f, 0.0158f}, {1, 0.3162f, 0.0501f, 0.0794f, 0.01995f},
  {1, 0.1f, 0.0708f, 0.0316f, 0.01995f},   {1, 0.5f, 0.1008f, 0.0631f, 0.0126f},
  {1, 0.5f, 0.0708f, 0.0631f, 0.0126f},    {1, 0.2f, 0.0708f, 0.0316f, 0.01995f},
  {1, 0.1f, 0.0708f, 0.0316f, 0.01995f},   {1, 0.2f, 0.0708f, 0.0316f, 0.02995f},
  {1, 0.3f, 0.0608f, 0.0316f, 0.01995f},   {1, 0.1f, 0.1708f, 0.0216f, 0.02995f},
  {1, 0.15f, 0.0708f, 0.0316f, 0.02995f},  {1, 0.1f, 0.0708f, 0.0316f, 0.02995f},
  {1, 0.1f, 0.0708f, 0.0316f, 0.02995f}};
static const float VOWEL_Q[N_VOWELS][5] = {
  {80, 90, 120, 130, 140}, {70, 80, 100, 120, 120}, {40, 90, 100, 120, 120}, {40, 80, 100, 120, 120},
  {40, 60, 100, 120, 120}, {80, 90, 120, 130, 140}, {80, 90, 120, 130, 140}, {40, 60, 100, 120, 120},
  {40, 60, 100, 120, 120}, {40, 60, 100, 120, 120}, {40, 70, 100, 120, 130}, {40, 60, 100, 120, 120},
  {40, 60, 100, 120, 120}, {40, 60, 100, 120, 120}, {40, 60, 100, 120, 120}};
typedef struct {
  bool on;
  float g[5];
  Biquad f[5][2]; // [formant][channel]
} Vowel;
static void vowel_setup(Vowel *w, int idx, float sr) {
  w->on = idx >= 0 && idx < N_VOWELS;
  if (!w->on) return;
  for (int i = 0; i < 5; i++) {
    w->g[i] = VOWEL_G[idx][i];
    for (int ch = 0; ch < 2; ch++) biquad_bandpass(&w->f[i][ch], VOWEL_F[idx][i], VOWEL_Q[idx][i], sr);
  }
}
static inline float vowel_run(Vowel *w, float x, int ch) {
  float y = 0;
  for (int i = 0; i < 5; i++) y += w->g[i] * biquad_run(&w->f[i][ch], x);
  return y * 8.0f; // makeupGain
}

// ---------- voices ------------------------------------------------------------
enum Source { SRC_SINE = 0, SRC_SAW, SRC_SQUARE, SRC_TRIANGLE, SRC_SUPERSAW, SRC_SAMPLE,
              SRC_WHITE, SRC_PINK, SRC_BROWN, SRC_CRACKLE, SRC_PULSE };
// Chrome PEAK-NORMALIZES its built-in sawtooth and square (PeriodicWave: scale
// = 1/max of the full-band table — the Gibbs overshoot). MEASURED in Chromium
// (OfflineAudioContext, 2026-09-23): saw rms .4894 vs the series' .5774, square
// .8476 vs 1.0 — both ×0.8477. zaltz played them +1.43dB hot until then.
// Sine and triangle have no overshoot: unscaled, identical to Chrome.
#define WA_NORM 0.8477f
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

// PHASE VOCODER (stretch) — forward decls; implementation lives with the
// other worklet ports below (phaze OLA + spectral peak shift, per voice)
typedef struct Pv Pv;
static Pv *pv_alloc(void);
static void pv_release(Pv *p);
static void pv_tables_init(void);

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
  // DISTORTION FAMILY (DistortProcessor port): y = pg·algo(x, expm1(distort))
  bool dist_on;
  float dist_k, dist_pg;
  int dist_alg;
  // TREMOLO (superdough.mjs:796-827 + LFOProcessor): amp gain =
  // max(1−depth,0) + clamp(pow(tri(phase,skew)·depth, 1.5), 0, 1)
  bool trem_on;
  float trem_rate, trem_depth, trem_skew, trem_base;
  double trem_phase;
  // PITCH ENVELOPE (helpers.mjs getPitchEnvelope): cents ADSR on the source
  // frequency — min = −cents·anchor, max = cents − cents·anchor (linear curve)
  bool penv_on;
  Adsr penv_env;
  float penv_min, penv_max;
  bool crush_on; // CrushProcessor runs whenever crush is SET (crush(0) quantizes too)
  float crush; // bit reduce: round(x·2^(crush−1))/2^(crush−1), crush = max(1, crush)
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
  Filt flt[3]; // lp → hp → bp (the filter family above)
  Vowel vow;   // after the filters (superdough.mjs:751)
  // RETIRE (the crossfade takeover): the OLD music fades out under the new
  // loop instead of being hushed — set on live voices by sd_retire and
  // inherited by voices spawned from pre-retire events
  double retire_start; // frame the fade began (-1 = not retiring)
  // VOICE STEALING (superdough parity): when the musical cap is reached the
  // OLDEST voice is ramped to 0 over 0.25s and the new note always starts.
  double steal_at; // frame the steal ramp began (-1 = not stolen)
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
  float pitch_mult; // 2^(detune cents/1200) — vib + penv, control rate
  // FM operator 1
  bool fm_on, fm_env_on, fm_env_lin;
  int fm_wave;
  float fm_dev, fm_modfreq, fm_envval;
  double fm_phase;
  Adsr fm_env;
  float fm_nb[7], fm_nlast; // noise-modulator state
  // PULSE — half-Tomisawa (worklets.mjs PulseOscillatorProcessor), doubles like JS
  double pl_phi, pl_y0, pl_y1, pl_dphif, pl_envf, pl_env;
  float pl_pw, pw_depth, pw_rate;
  bool pw_lfo, pl_live;
  double pw_phase;
  // noise mix (getNoiseMix → drywet)
  bool nmix_on;
  int trem_shape;
  double nudge_frames; // sampler nudge: buffer start offset from the envelope
  float nmix_dry, nmix_wet;
  const float *ptab; // partials table (waveformN), NULL = the stock wave
  // SAMPLE voice
  const float *pcm;
  int pcm_frames, pcm_channels;
  double pos;      // frames into the buffer
  double rate;     // playbackRate (buffer frames per output frame)
  double base_rate;
  bool rev; // speed < 0: the WHOLE buffer reads reversed (getSampleBufferSource)
  bool smp_loop;
  double loop_a, loop_b;
  // PHASE VOCODER (stretch): superdough spawns a fresh phase-vocoder worklet
  // per hap; here each stretch voice owns a fresh Pv (arena, freelist-reused).
  // pv == 0 with stretch set means the PV pool was exhausted → voice plays dry.
  Pv *pv;
  float pv_stretch; // RAW stretch value; the worklet's transform runs per block
  bool pv_dead;     // source ended — OLA tail still draining
} Voice;

typedef struct {
  float time, freq, duration;
  float attack, decay, sustain, release;
  float gain, velocity, postgain, pan;
  float lpf, lpq, hpf, hpq;
  float hpattack, hpdecay, hpsustain, hprelease, hpenv;
  float bandf, bandq, bpattack, bpdecay, bpsustain, bprelease, bpenv;
  float fanchor;
  int vowel; // formant row, −1 = none
  // FM operator 1 (helpers.mjs applyFM): fmi Hz-per-unit·modfreq, ratio fmh
  float fmi, fmh, fmattack, fmdecay, fmsustain, fmrelease;
  int fmwave;     // 0 sine 1 square 2 sawtooth 3 triangle 4 white 5 pink 6 brown 7 crackle
  bool fmenv_lin; // fmenv "lin" (default "exp")
  // PULSE (synth.mjs 'pulse'): width + the resolved pw LFO (bridge applies the defaults)
  float pw, pwrate, pwsweep;
  float noise;    // getOscillator noise mix (pink, drywet)
  float djf;      // orbit DJ filter value (NaN = not set)
  int tremoloshape; // −1 unset · 0 tri 1 sine 2 ramp 3 saw 4 square
  float nudge;    // sampler: the buffer starts `nudge` s after the envelope
  int partials;   // getOscillator: partials ?? n → waveformN (0 = stock wave)
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
  float room, roomsize, roomlp, roomdim, delay, delaytime, delayfeedback;
  float shape, shapevol;
  float distort, distortvol;
  int distorttype;
  float tremolo, tremolodepth, tremoloskew, tremolophase, tremtime;
  float penv, pattack, pdecay, psustain, prelease, panchor;
  float stretch; // phase-vocoder pitch factor (NaN = off)
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

// waveformN (synth.mjs:457) — `partials ?? n` harmonics of the wave's own
// series (saw imag −1/n · square imag 1/n odd · triangle real 1/n² odd),
// peak-normalized from the FULL series (PeriodicWave, disableNormalization
// false), then band-limited to the note (partials past Nyquist dropped, the
// scale kept) like Chrome's range tables. Cached per (wave, count, band).
static float sr_f; // tentative — defined with the orbit code below
static double inv_sr = 1.0 / 48000.0; // set in sd_init
#define PT_SLOTS 12
typedef struct { int src, n, ne; float tab[WT_LEN + 1]; } PTab;
static PTab pt_cache[PT_SLOTS];
static int pt_used = 0, pt_next = 0;
static float pt_term_sin(int src, int k) {
  if (src == SRC_SAW) return -1.0f / (float)k;
  if (src == SRC_SQUARE) return (k & 1) ? 1.0f / (float)k : 0.0f;
  return 0.0f;
}
static float pt_term_cos(int src, int k) {
  return (src == SRC_TRIANGLE && (k & 1)) ? 1.0f / ((float)k * (float)k) : 0.0f;
}
static float pt_sum(int src, int n, float t) {
  float acc = 0;
  for (int k = 1; k <= n; k++) {
    float a = pt_term_sin(src, k), b = pt_term_cos(src, k);
    if (a != 0) acc += a * sd_sinf(TWO_PI * (float)k * t);
    if (b != 0) acc += b * sd_cosf(TWO_PI * (float)k * t);
  }
  return acc;
}
static const float *partial_table(int src, int n, float freq) {
  if (n > 1024) n = 1024;
  int nyq = (int)((sr_f * 0.5f) / (freq > 1 ? freq : 1));
  int ne = n < nyq ? n : nyq;
  if (ne < 1) ne = 1;
  for (int i = 0; i < pt_used; i++)
    if (pt_cache[i].src == src && pt_cache[i].n == n && pt_cache[i].ne == ne) return pt_cache[i].tab;
  PTab *p = &pt_cache[pt_next];
  pt_next = (pt_next + 1) % PT_SLOTS;
  if (pt_used < PT_SLOTS) pt_used++;
  float mx = 0;
  for (int i = 0; i < WT_LEN; i++) { // the normalization reads the FULL series
    float x = pt_sum(src, n, (float)i / (float)WT_LEN);
    if (x < 0) x = -x;
    if (x > mx) mx = x;
  }
  float scale = mx > 0 ? 1.0f / mx : 1.0f;
  for (int i = 0; i < WT_LEN; i++) p->tab[i] = pt_sum(src, ne, (float)i / (float)WT_LEN) * scale;
  p->tab[WT_LEN] = p->tab[0];
  p->src = src; p->n = n; p->ne = ne;
  return p->tab;
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
  float gen_size, gen_lp, gen_dim; // convolver.duration/lp/dim as last generated (defaults applied)
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
  // DJ FILTER (superdoughoutput getDjf + worklets.mjs DJFProcessor): once an
  // orbit has one it keeps it; each hap's djf is setValueAtTime(t) — read
  // k-rate (value[0]), so a change lands on the first quantum starting ≥ t
  bool djf_on;
  float djf_val, djf_next;
  double djf_at;
  double djf_s0[2], djf_s1[2];
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

static void orbit_config_verb(Orbit *o, float t60, float lp, float dim) {
  float t = t60 > 0.05f ? t60 : 2.0f;
  float l = lp > 0 ? lp : 15000.0f;
  // ROOMDIM (superdough reverbGen): the convolver IR's lowpass RAMPS linearly
  // lp → dim (Hz) across the decay, so high frequencies die early — content at
  // f survives only until the ramp crosses f, i.e. T60(f)/T60 ≈ (lp−f)/(lp−dim).
  // The FDN's per-pass one-pole is the same mechanism in loop form (older
  // energy = more passes = darker); a swept cutoff can't be expressed in a
  // recirculating network, so match the HIGH-BAND DECAY TIME instead: pick the
  // per-pass cutoff whose extra attenuation at f_ref makes T60(f_ref) hit the
  // convolver's ratio. Derivation: per pass amp = g·a, want ln(g·a)/ln g = 1/r
  // → ln a = ln g·(1/r − 1); one-pole |H(f_ref)| = a → fc = f_ref/√(a⁻²−1).
  // dim defaults 1000 (reverb.mjs generate) — that maps to fc ≈ the plain
  // roomlp already in use, so unset stays the calibrated sound.
  if (!is_nan(dim) && dim > 0 && dim < l) {
    float fref = l > 12000.0f ? 5000.0f : l * (1.0f / 3.0f);
    float r = (l - fref) / (l - dim);
    if (r < 0.05f) r = 0.05f;
    if (r < 1.0f) {
      float lng = -3.0f * 2.302585f * (0.042f / t); // mean FDN line ≈ 42ms
      // NETWORK DILUTION 0.32, MEASURED (Schroeder band-T60 harness,
      // verify-rdim2): the Householder mixing spreads energy across the 8
      // lines, so the in-loop one-pole bites ~1/3 as hard as a naive
      // per-pass cascade predicts — consistent at fc 5k (ratio .58) and
      // 12.7k (ratio .89). The wanted per-pass loss divides by it.
      float a = sd_exp2f(lng * (1.0f / r - 1.0f) * (1.0f / 0.32f) * 1.442695f);
      if (a < 0.9995f) {
        float inv = 1.0f / (a * a) - 1.0f;
        float fc = fref / sd_sqrtf(inv);
        if (fc < l) l = fc;
      }
    }
  }
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
  // DECAY TRIM (2026-07-29) — the nominal law below is superdough's exactly
  // (reverbGen: amplitude decayBase^n, −60dB at decayTime), but a FEEDBACK
  // NETWORK is not an IR: the input diffusion allpasses, the in-loop damping
  // one-pole and the linear interpolation on the two modulated lines each take
  // a bite per pass, so the REALISED decay ran short — measured 1.60s for a
  // nominal 2s, and worse the longer the room (5.02s for 8s), because more
  // passes means more bites. The room died early and the product read drier and
  // shorter than strudel.cc ("it just feels like it sustains a little more").
  // Asking for a longer decay than we want cancels it. The curve is FITTED to
  // engine/golden/roomsize.mjs, which measures the realised T60 by Schroeder
  // integration and fails if any size drifts more than 12%.
  float trim = 1.19f + 0.052f * t;
  if (trim < 1.1f) trim = 1.1f;
  if (trim > 1.75f) trim = 1.75f;
  const float t_set = t * trim;
  for (int i = 0; i < FDN_LINES; i++) {
    int len = (int)((float)FDN_PRIMES[i] * (sr_f / 48000.0f));
    if (len >= FDN_MAX) len = FDN_MAX - 1;
    o->fdn_len[i] = len;
    // g = 10^(−3·lineSeconds/T60): −60dB after T60 (reverbGen decayBase)
    o->fdn_g_tgt[i] = sd_pow10f(-3.0f * ((float)len / sr_f) / t_set);
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

/** getReverb (superdoughoutput.mjs:69-92): the orbit's room regenerates only
 *  when a value the hap SET differs from what was last generated; the
 *  regeneration then takes generate()'s defaults (2s, lp 15000, dim 1000) for
 *  everything the hap left unset. THE DEFAULT ROOM DARKENS (2026-09-23): every
 *  superdough IR runs applyGradualLowpass lp → dim, and dim defaults to 1000 —
 *  so an unset room's highs die early (T60 5k/500 ≈ .71). zaltz used to skip
 *  the dim for unset rooms (.90): every default tail rang brighter than
 *  strudel.cc. engine/golden/rdim.mjs now asserts unset == dim 1000. */
static void orbit_room(Orbit *o, float size, float lp, float dim) {
  bool changed = !o->verb_on ||
                 (!is_nan(size) && size != o->gen_size) ||
                 (!is_nan(lp) && lp != o->gen_lp) ||
                 (!is_nan(dim) && dim != o->gen_dim);
  if (!changed) return;
  o->gen_size = is_nan(size) ? 2.0f : size;
  o->gen_lp = is_nan(lp) ? 15000.0f : lp;
  o->gen_dim = is_nan(dim) ? 1000.0f : dim;
  orbit_config_verb(o, o->gen_size, o->gen_lp, o->gen_dim);
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
  // FeedbackDelayNode is `super(ac)` → DelayNode maxDelayTime 1s, so longer
  // times clamp to 1s; a DelayNode inside a cycle is at least one render
  // quantum (Web Audio §1.21)
  o->dt_tgt = sd_fminf(sd_fmaxf(dt, (float)BLOCK / sr_f), 1.0f);
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
  // THE RECOVERY USED TO NEVER RUN (2026-07-29, measured). `duckonset`
  // defaults to 0, so t1 == t0 — and the old guard read
  // `f < t1 || t1 <= t0`, whose right half is then TRUE for every f. The
  // function returned duck_low for the WHOLE window and snapped to 1 at t2:
  // a 200ms gate at 1% instead of superdough's instant dip and exponential
  // climb back. Everything sharing a ducked bus got chopped rather than
  // pumped — the user's fiddle. Onset dip and attack recovery are now
  // separate stretches, and a zero-length onset simply skips to the climb.
  if (f < o->duck_t1) { // the dip itself (only when duckonset > 0)
    float u = (float)((f - o->duck_t0) / (o->duck_t1 - o->duck_t0));
    return o->duck_from * sd_exp2f(u * sd_log2f(o->duck_low / o->duck_from));
  }
  if (o->duck_t2 <= o->duck_t1) return o->duck_low; // degenerate attack
  // the climb back: duck_low → 1 across `duckattack`, exponential like
  // superdough's exponentialRampToValueAtTime
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

// ---------- STEM TAP (the take's track separation) ---------------------------
// Armed by sd_stems: the bus pass mirrors each used orbit's post-FX,
// post-duck, post-kill block — the EXACT per-sample value it adds into
// out_buf — into that orbit's stem_buf slot. Summing every stem reconstructs
// the pre-limiter master by construction; the host ships used slots to the
// recorder. Off = one predictable branch per sample, no writes.
static int stems_on = 0;
static float stem_buf[MAX_ORBITS * BLOCK * OUT_CH];
static unsigned int stem_mask_lo, stem_mask_hi; // orbits 0-31 / 32-39, per block

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
__attribute__((export_name("sd_stems"))) void sd_stems(int on) { stems_on = on; }
__attribute__((export_name("sd_stem_ptr"))) float *sd_stem_ptr(void) { return stem_buf; }
// which orbits wrote their slot THIS block (a slot without its bit is stale)
__attribute__((export_name("sd_stem_mask_lo"))) unsigned int sd_stem_mask_lo(void) { return stem_mask_lo; }
__attribute__((export_name("sd_stem_mask_hi"))) unsigned int sd_stem_mask_hi(void) { return stem_mask_hi; }
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
  for (int i = 0; i < MAX_VOICES; i++) {
    voices[i].active = false;
    voices[i].steal_at = -1;
    if (voices[i].pv) { pv_release(voices[i].pv); voices[i].pv = 0; } // capped pool must never leak
  }
  for (int i = 0; i < MAX_EVENTS; i++) events[i].used = false;
  for (int i = 0; i < MAX_CUT_GROUPS; i++) cut_last[i] = -1;
  for (int i = 0; i < MAX_ORBITS; i++) {
    orbits[i].used = false;
    orbits[i].delay_on = false;
    orbits[i].verb_on = false;
    orbits[i].duck_active = false;
    orbits[i].djf_on = false;
    orbits[i].duck_g = 1.0f;
    orbits[i].out_g = 1.0f;
    orbits[i].out_g_tgt = 1.0f;
  }
}

__attribute__((export_name("sd_init"))) void sd_init(float sample_rate) {
  sr_f = sample_rate;
  inv_sr = 1.0 / (double)sample_rate;
  engine_frame = 0;
  build_wavetables(sample_rate); // ONCE per boot — never on the hush path
  pv_tables_init();              // FFT twiddles + Hann, once
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
  ev.hpattack = NAN_F; ev.hpdecay = NAN_F; ev.hpsustain = NAN_F; ev.hprelease = NAN_F; ev.hpenv = NAN_F;
  ev.bandf = 0; ev.bandq = 1; // createFilter q default 1 (LINEAR for the bandpass)
  ev.bpattack = NAN_F; ev.bpdecay = NAN_F; ev.bpsustain = NAN_F; ev.bprelease = NAN_F; ev.bpenv = NAN_F;
  ev.vowel = -1;
  ev.fmi = 0; ev.fmh = 1; ev.fmwave = 0; ev.fmenv_lin = false;
  ev.fmattack = NAN_F; ev.fmdecay = NAN_F; ev.fmsustain = NAN_F; ev.fmrelease = NAN_F;
  ev.pw = 0.5f; ev.pwrate = 1; ev.pwsweep = 0; ev.noise = 0; ev.partials = 0;
  ev.djf = NAN_F; ev.tremoloshape = -1; ev.nudge = 0;
  ev.fanchor = NAN_F; // createFilter anchor: nanFallback(anchor, 0) — no default consulted
  ev.unison = 5; ev.spread = 0.6f; ev.detune = NAN_F; // supersaw defaults (synth.mjs:157-158)
  ev.lpattack = NAN_F; ev.lpdecay = NAN_F; ev.lpsustain = NAN_F; ev.lprelease = NAN_F; ev.lpenv = NAN_F;
  ev.ftype = 0; ev.drive = 0.69f; ev.density = 0.02f;
  ev.phaserrate = 0; ev.phaserdepth = 0.75f; ev.phasercenter = 1000.0f; ev.phasersweep = 2000.0f;
  ev.retire = false;
  ev.vib = 0; ev.vibmod = 0.5f; // helpers.mjs:347
  ev.sample_id = -1; ev.speed = 1; ev.begin = 0; ev.endf = 1; ev.loopv = 0; ev.loop_begin = 0; ev.loop_end = 1;
  ev.orbit = 1;
  // room params stay UNSET (NaN) unless the hap names them: superdough only
  // regenerates an orbit's room for a value a hap actually SET (hasChanged,
  // superdoughoutput.mjs:14) — a layer without roomsize must not drag a
  // shared orbit's size back to 2
  ev.room = 0; ev.roomsize = NAN_F; ev.roomlp = NAN_F; ev.roomdim = NAN_F; ev.delay = 0;
  // superdough: delaytime ?? delaysync(3/16 cycle) / cps — 0.375s at its default
  // cps 0.5. Klappn's bridge always sends the tempo-true value; this is the
  // fallback for hosts that speak to the engine directly (the npm package).
  ev.delaytime = 0.375f; ev.delayfeedback = 0.5f;
  ev.shape = NAN_F; ev.shapevol = 1;
  ev.distort = NAN_F; ev.distortvol = 1; ev.distorttype = 0; // superdough DEFAULT_VALUES
  ev.tremolo = NAN_F; ev.tremolodepth = 1; ev.tremoloskew = NAN_F; // default: 1, or .5 once a shape is named (superdough.mjs:818)
  ev.tremolophase = 0; ev.tremtime = 0;
  ev.penv = NAN_F; ev.pattack = NAN_F; ev.pdecay = NAN_F; ev.psustain = NAN_F;
  ev.prelease = NAN_F; ev.panchor = NAN_F;
  ev.stretch = NAN_F;
  ev.duck_n = 0; ev.duckonset = 0; ev.duckattack = 0.1f; ev.duckdepth = 1;
  ev.crush = NAN_F; ev.coarse = 0; ev.cut = -1;
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
    else if (str_eq(key, "hpattack") || str_eq(key, "hpa")) ev.hpattack = parse_f(val);
    else if (str_eq(key, "hpdecay") || str_eq(key, "hpd")) ev.hpdecay = parse_f(val);
    else if (str_eq(key, "hpsustain") || str_eq(key, "hps")) ev.hpsustain = parse_f(val);
    else if (str_eq(key, "hprelease") || str_eq(key, "hpr")) ev.hprelease = parse_f(val);
    else if (str_eq(key, "hpenv") || str_eq(key, "hpe")) ev.hpenv = parse_f(val);
    else if (str_eq(key, "bandf") || str_eq(key, "bpf")) ev.bandf = parse_f(val);
    else if (str_eq(key, "bandq") || str_eq(key, "bpq")) ev.bandq = parse_f(val);
    else if (str_eq(key, "bpattack") || str_eq(key, "bpa")) ev.bpattack = parse_f(val);
    else if (str_eq(key, "bpdecay") || str_eq(key, "bpd")) ev.bpdecay = parse_f(val);
    else if (str_eq(key, "bpsustain") || str_eq(key, "bps")) ev.bpsustain = parse_f(val);
    else if (str_eq(key, "bprelease") || str_eq(key, "bpr")) ev.bprelease = parse_f(val);
    else if (str_eq(key, "bpenv") || str_eq(key, "bpe")) ev.bpenv = parse_f(val);
    else if (str_eq(key, "fanchor")) ev.fanchor = parse_f(val);
    else if (str_eq(key, "vowel")) ev.vowel = (int)parse_f(val); // the bridge sends the row index
    else if (str_eq(key, "fmi") || str_eq(key, "fm")) ev.fmi = parse_f(val);
    else if (str_eq(key, "fmh")) ev.fmh = parse_f(val);
    else if (str_eq(key, "fmwave")) ev.fmwave = (int)parse_f(val);
    else if (str_eq(key, "fmenv")) ev.fmenv_lin = parse_f(val) > 0;
    else if (str_eq(key, "fmattack")) ev.fmattack = parse_f(val);
    else if (str_eq(key, "fmdecay")) ev.fmdecay = parse_f(val);
    else if (str_eq(key, "fmsustain")) ev.fmsustain = parse_f(val);
    else if (str_eq(key, "fmrelease")) ev.fmrelease = parse_f(val);
    else if (str_eq(key, "pw")) ev.pw = parse_f(val);
    else if (str_eq(key, "pwrate")) ev.pwrate = parse_f(val);
    else if (str_eq(key, "pwsweep")) ev.pwsweep = parse_f(val);
    else if (str_eq(key, "noise")) ev.noise = parse_f(val);
    else if (str_eq(key, "partials")) ev.partials = (int)parse_f(val);
    else if (str_eq(key, "djf")) ev.djf = parse_f(val);
    else if (str_eq(key, "tremoloshape")) ev.tremoloshape = (int)parse_f(val);
    else if (str_eq(key, "nudge")) ev.nudge = parse_f(val);
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
    else if (str_eq(key, "roomdim")) ev.roomdim = parse_f(val);
    else if (str_eq(key, "delay")) ev.delay = parse_f(val);
    else if (str_eq(key, "delaytime")) ev.delaytime = parse_f(val);
    else if (str_eq(key, "delayfeedback")) ev.delayfeedback = parse_f(val);
    else if (str_eq(key, "shape")) ev.shape = parse_f(val);
    else if (str_eq(key, "shapevol")) ev.shapevol = parse_f(val);
    else if (str_eq(key, "distort")) ev.distort = parse_f(val);
    else if (str_eq(key, "distortvol")) ev.distortvol = parse_f(val);
    else if (str_eq(key, "distorttype")) {
      // name or index (getDistortionAlgorithm: names wrap by index too)
      if (str_eq(val, "scurve")) ev.distorttype = 0;
      else if (str_eq(val, "soft")) ev.distorttype = 1;
      else if (str_eq(val, "hard")) ev.distorttype = 2;
      else if (str_eq(val, "cubic")) ev.distorttype = 3;
      else if (str_eq(val, "diode")) ev.distorttype = 4;
      else if (str_eq(val, "asym")) ev.distorttype = 5;
      else if (str_eq(val, "fold")) ev.distorttype = 6;
      else if (str_eq(val, "sinefold")) ev.distorttype = 7;
      else if (str_eq(val, "chebyshev")) ev.distorttype = 8;
      else ev.distorttype = ((int)parse_f(val)) % 9;
    }
    else if (str_eq(key, "stretch")) ev.stretch = parse_f(val);
    else if (str_eq(key, "tremolo")) ev.tremolo = parse_f(val);
    else if (str_eq(key, "tremolodepth")) ev.tremolodepth = parse_f(val);
    else if (str_eq(key, "tremoloskew")) ev.tremoloskew = parse_f(val);
    else if (str_eq(key, "tremolophase")) ev.tremolophase = parse_f(val);
    else if (str_eq(key, "tremtime")) ev.tremtime = parse_f(val);
    else if (str_eq(key, "penv")) ev.penv = parse_f(val);
    else if (str_eq(key, "pattack")) ev.pattack = parse_f(val);
    else if (str_eq(key, "pdecay")) ev.pdecay = parse_f(val);
    else if (str_eq(key, "psustain")) ev.psustain = parse_f(val);
    else if (str_eq(key, "prelease")) ev.prelease = parse_f(val);
    else if (str_eq(key, "panchor")) ev.panchor = parse_f(val);
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
      if (str_eq(val, "sine") || str_eq(val, "sin")) ev.src = SRC_SINE;
      else if (str_eq(val, "sawtooth") || str_eq(val, "saw")) ev.src = SRC_SAW;
      else if (str_eq(val, "square") || str_eq(val, "sqr")) ev.src = SRC_SQUARE;
      else if (str_eq(val, "pulse")) ev.src = SRC_PULSE;
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

/** superdough's polyphony law (superdough.mjs:524-531), which zaltz used to
 *  invert: when the cap is exceeded superdough ramps the OLDEST sound to 0
 *  over 0.25s and ALWAYS plays the new one. zaltz used to scan for a free
 *  slot and, finding none, silently DROP the new note — so in a dense patch
 *  long ringing tails starved every fresh hit (the user's "the hi-hat is not
 *  even playing"). Voices that are already stolen or retiring don't count
 *  against the cap (they are on their way out) and are never stolen twice. */
static void enforce_polyphony(double at_frame) {
  for (;;) {
    int live = 0, oldest = -1;
    unsigned int oldest_vid = 0;
    for (int i = 0; i < MAX_VOICES; i++) {
      const Voice *v = &voices[i];
      if (!v->active || v->steal_at >= 0 || v->retire_start >= 0) continue;
      live++;
      if (oldest < 0 || v->vid < oldest_vid) {
        oldest = i;
        oldest_vid = v->vid;
      }
    }
    if (live < POLY_CAP || oldest < 0) return;
    voices[oldest].steal_at = at_frame; // 0.25s ramp, then the slot frees
  }
}

static void start_voice(const Event *ev) {
  enforce_polyphony(ev->at_frame);
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active) continue;
    Voice *v = &voices[i];
    v->active = true;
    v->steal_at = -1;
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
      // the PULSE env peaks at 1, not 0.3 (synth.mjs 'pulse': getParamADSR(…, 0, 1))
      // — its 0.15 level lives inside the worklet
      if (ev->src == SRC_PULSE) v->amp = ev->gain * ev->velocity;
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
    filt_setup(&v->flt[FT_LP], FT_LP, ev->lpf, ev->lpq, ev->ftype, ev->drive, ev->lpattack,
               ev->lpdecay, ev->lpsustain, ev->lprelease, ev->lpenv, ev->fanchor, sr_f);
    filt_setup(&v->flt[FT_HP], FT_HP, ev->hpf, ev->hpq, ev->ftype, ev->drive, ev->hpattack,
               ev->hpdecay, ev->hpsustain, ev->hprelease, ev->hpenv, ev->fanchor, sr_f);
    filt_setup(&v->flt[FT_BP], FT_BP, ev->bandf, ev->bandq, ev->ftype, ev->drive, ev->bpattack,
               ev->bpdecay, ev->bpsustain, ev->bprelease, ev->bpenv, ev->fanchor, sr_f);
    vowel_setup(&v->vow, ev->vowel, sr_f);
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
    v->pitch_mult = 1.0f;
    // FM (applyFM, operator 1): oscillators, supersaw and pulse only — the
    // sampler and the noises never call it. modfreq = the carrier param's
    // VALUE × fmh (vibrato/penv ride detune, not this); `if (!amt) continue`.
    bool osc_src = ev->src == SRC_SINE || ev->src == SRC_SAW || ev->src == SRC_SQUARE ||
                   ev->src == SRC_TRIANGLE || ev->src == SRC_SUPERSAW || ev->src == SRC_PULSE;
    v->fm_on = osc_src && ev->fmi != 0 && !is_nan(ev->fmi);
    if (v->fm_on) {
      v->fm_modfreq = ev->freq * ev->fmh;
      v->fm_dev = ev->fmi * v->fm_modfreq; // gain(amt) → gain(modfreq) → frequency (Hz)
      v->fm_wave = ev->fmwave;
      v->fm_phase = 0; // mod() starts it at scheduling time upstream — its phase is arbitrary there
      v->fm_nlast = 0;
      for (int k = 0; k < 7; k++) v->fm_nb[k] = 0;
      v->fm_env_on = !is_nan(ev->fmattack) || !is_nan(ev->fmdecay) || !is_nan(ev->fmsustain) ||
                     !is_nan(ev->fmrelease);
      if (v->fm_env_on) {
        v->fm_env = adsr_values(ev->fmattack, ev->fmdecay, ev->fmsustain, ev->fmrelease,
                                0.001f, 0.001f, 1.0f, 0.01f); // bare getADSRValues
        v->fm_env_lin = ev->fmenv_lin;
      }
      v->fm_envval = v->fm_env_on ? 0.001f : 1.0f;
    }
    // noise mix (getOscillator → getNoiseMix → drywet): stock oscillators only
    v->nmix_on = ev->noise != 0 && (ev->src == SRC_SINE || ev->src == SRC_SAW ||
                                    ev->src == SRC_SQUARE || ev->src == SRC_TRIANGLE);
    if (v->nmix_on) {
      float d = ev->noise, w = 1.0f - ev->noise; // wetfade: 1 until .5, then down to 0 at 1
      v->nmix_dry = d < 0.5f ? 1.0f : 1.0f - (d - 0.5f) / 0.5f;
      v->nmix_wet = w < 0.5f ? 1.0f : 1.0f - (w - 0.5f) / 0.5f;
    }
    // partials (getOscillator: `partials ?? n`, never for sine)
    v->ptab = 0;
    if (ev->partials > 0 && (ev->src == SRC_SAW || ev->src == SRC_SQUARE || ev->src == SRC_TRIANGLE))
      v->ptab = partial_table(ev->src, ev->partials, ev->freq);
    if (ev->src == SRC_PULSE) {
      v->pl_phi = -3.14159265358979323846; v->pl_y0 = 0; v->pl_y1 = 0;
      v->pl_dphif = 0; v->pl_envf = 0; v->pl_env = 1; v->pl_live = false;
      v->pl_pw = ev->pw;
      v->pw_lfo = ev->pwsweep != 0;
      v->pw_depth = ev->pwsweep;
      v->pw_rate = ev->pwrate;
      // LFOProcessor: phase = ffrac(time·frequency), time = begin (absolute)
      double seed = (ev->at_frame / (double)sr_f) * (double)ev->pwrate;
      v->pw_phase = seed - (double)(long long)seed;
      if (v->pw_phase < 0) v->pw_phase += 1.0;
    }
    // ORBIT + SENDS (M4)
    int ob = ev->orbit;
    if (ob < 0) ob = 0;
    if (ob >= MAX_ORBITS) ob = MAX_ORBITS - 1;
    v->orbit = ob;
    orbits[ob].used = true;
    // sends are plain GainNodes (effectSend) — no ceiling, like superdough
    v->room_send = sd_fmaxf(ev->room, 0.0f);
    v->delay_send = sd_fmaxf(ev->delay, 0.0f);
    if (v->room_send > 0) orbit_room(&orbits[ob], ev->roomsize, ev->roomlp, ev->roomdim);
    if (!is_nan(ev->djf)) { // getDjf(value, t): the orbit gains its filter for good
      Orbit *oo = &orbits[ob];
      if (!oo->djf_on) {
        oo->djf_on = true;
        oo->djf_val = 0.5f; // DJFProcessor default until the first set lands
        oo->djf_s0[0] = oo->djf_s0[1] = oo->djf_s1[0] = oo->djf_s1[1] = 0;
      }
      oo->djf_next = ev->djf;
      oo->djf_at = ev->at_frame;
    }
    // superdough.mjs:955 — the orbit delay exists only when ALL three are > 0
    if (v->delay_send > 0 && ev->delaytime > 0 && ev->delayfeedback > 0)
      orbit_config_delay(&orbits[ob], ev->delaytime, ev->delayfeedback);
    else
      v->delay_send = 0;
    v->shape_on = !is_nan(ev->shape); // ShapeProcessor runs whenever shape is set (shapevol applies at 0)
    if (v->shape_on) {
      // ShapeProcessor clamp — superdough's 1-4e-10 rounds to exactly 1.0f in
      // float32 (k would be inf → NaN into the orbit rings); stop a hair lower.
      float sh = ev->shape >= 0.999999f ? 0.999999f : ev->shape;
      v->shape_k = (2.0f * sh) / (1.0f - sh);
      float pg = ev->shapevol;
      v->shapevol = sd_fminf(sd_fmaxf(pg, 0.001f), 1.0f);
    }
    v->dist_on = !is_nan(ev->distort); // distort(0) still applies distortvol (superdough.mjs:797)
    if (v->dist_on) {
      v->dist_k = sd_expm1f(ev->distort);            // DistortProcessor: expm1(distort)
      v->dist_pg = sd_clampf(ev->distortvol, 0.001f, 1.0f);
      v->dist_alg = ev->distorttype;
      if (v->dist_alg < 0 || v->dist_alg > 8) v->dist_alg = 0;
    }
    v->trem_on = !is_nan(ev->tremolo) && ev->tremolo > 0;
    if (v->trem_on) {
      v->trem_rate = ev->tremolo;
      v->trem_depth = is_nan(ev->tremolodepth) ? 1.0f : ev->tremolodepth;
      v->trem_shape = ev->tremoloshape < 0 ? 0 : ev->tremoloshape % 5;
      v->trem_skew = !is_nan(ev->tremoloskew) ? sd_clampf(ev->tremoloskew, 0.0f, 1.0f)
                                                : (ev->tremoloshape >= 0 ? 0.5f : 1.0f);
      v->trem_base = sd_fmaxf(1.0f - v->trem_depth, 0.0f); // amGain base (superdough.mjs:814)
      // LFOProcessor phase seed: ffrac(time·frequency + phaseoffset), where
      // time = the hap's cycle position in seconds (cycle/cps)
      float seed = ev->tremtime * v->trem_rate + ev->tremolophase;
      v->trem_phase = (double)(seed - sd_floorf(seed));
    }
    // getPitchEnvelope: active when ANY of the p-family was given; defaults
    // [0.2, 0.001, 1, 0.001], penv default 1 semitone, anchor default sustain.
    v->penv_on = !is_nan(ev->penv) || !is_nan(ev->pattack) || !is_nan(ev->pdecay) ||
                 !is_nan(ev->psustain) || !is_nan(ev->prelease);
    if (v->penv_on) {
      float pen = is_nan(ev->penv) ? 1.0f : ev->penv;
      float pa = is_nan(ev->pattack) ? 0.2f : ev->pattack;
      float pd = is_nan(ev->pdecay) ? 0.001f : ev->pdecay;
      float ps = is_nan(ev->psustain) ? 1.0f : ev->psustain;
      float pr = is_nan(ev->prelease) ? 0.001f : ev->prelease;
      float anchor = is_nan(ev->panchor) ? ps : ev->panchor;
      float cents = pen * 100.0f;
      v->penv_min = 0.0f - cents * anchor;
      v->penv_max = cents - cents * anchor;
      v->penv_env = adsr_values(pa, pd, ps, pr, 0, 0, 0, 0);
    }
    // PHASE VOCODER: fresh state per hap, like superdough's per-trigger
    // worklet node. Pool exhausted → dry (documented cap, never unbounded).
    v->pv = 0;
    v->pv_dead = false;
    v->pv_stretch = ev->stretch;
    if (!is_nan(ev->stretch)) v->pv = pv_alloc();
    v->crush_on = !is_nan(ev->crush);
    v->crush = v->crush_on ? sd_fmaxf(1.0f, ev->crush) : 0; // CrushProcessor: max(1, crush)
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
        if (v->pv) { pv_release(v->pv); v->pv = 0; }
        v->active = false;
        return;
      }
      const Sample *sm = &samples[ev->sample_id];
      v->pcm = arena_base + sm->offset;
      v->pcm_frames = sm->frames;
      v->pcm_channels = sm->channels;
      // AudioBufferSourceNode on superdough's buffer (sampler.mjs:36-80):
      // playbackRate = |speed| (the bridge folds the pitch-map transpose in);
      // speed < 0 REVERSES THE WHOLE BUFFER, then offset = begin·duration on
      // that reversed buffer — so .begin(.25).speed(-1) starts at the ORIGINAL
      // 75% point, reading down. `end` never stops the source: it only sets
      // the hold (sliceDuration, computed by the bridge) — the voice plays to
      // its release like any other and the buffer runs to its own end.
      double rate = ev->speed < 0 ? -ev->speed : ev->speed;
      v->rev = ev->speed < 0;
      v->nudge_frames = ev->nudge > 0 ? (double)ev->nudge * (double)sr_f : 0; // a negative nudge plays as 0 here
      v->rate = rate;
      v->base_rate = rate;
      v->pos = (double)ev->begin * (double)sm->frames;
      if (v->pos < 0) v->pos = 0; // negative begin must never index the arena
      v->smp_loop = ev->loopv > 0;
      v->loop_a = (double)ev->loop_begin * (double)sm->frames;
      if (v->loop_a < 0) v->loop_a = 0;
      v->loop_b = (double)ev->loop_end * (double)sm->frames;
    }
    if (ev->src == SRC_SUPERSAW) {
      // synth.mjs:156-170 + worklets.mjs SuperSawOscillatorProcessor
      // voices = clamp(unison, 1, 100) (synth.mjs:168), a k-rate param the
      // worklet loops `n < voices` over — a fractional count runs ⌈voices⌉
      // saws while the detuner and the 1/√voices trim use the raw value
      float uf = sd_clampf(ev->unison, 1.0f, 100.0f);
      int u = (int)uf;
      if ((float)u < uf) u++;
      if (u > MAX_UNISON) u = MAX_UNISON;
      v->unison = u;
      float fs = is_nan(ev->detune) ? 0.18f : ev->detune; // freqspread, semitones
      if (fs < 0) fs = 0; // freqspread param min 0 (worklets.mjs:511)
      // getDetuner (worklets.mjs:38): idx·(fs/(voices−1)) − fs/2; 0 when voices < 2
      for (int k = 0; k < u; k++) {
        v->fan[k] = uf < 2 ? 0.0f : (float)k * (fs / (uf - 1.0f)) - fs * 0.5f;
        v->ss_phase[k] = (double)frandf();
      }
      // panspread → alternating √ gains (worklets.mjs:544-548):
      // ps = spread·0.5+0.5; gainL = √(1−ps), gainR = √ps, swapped per voice
      float ps = uf > 1 ? sd_fminf(sd_fmaxf(ev->spread, 0.0f), 1.0f) : 0.0f;
      ps = ps * 0.5f + 0.5f;
      v->ss_gl = sd_sqrtf(1.0f - ps);
      v->ss_gr = sd_sqrtf(ps);
      // env max is 0.3·(1/√voices) (synth.mjs:186,195): fold 1/√u into amp
      v->amp /= sd_sqrtf(uf);
    }
    return;
  }
  // EVERY physical slot busy — only reachable if ~192 stolen voices are still
  // inside their 0.25s fades at once (≈770 steals/s). superdough's contract is
  // that the NEW sound always plays, so take the oldest slot outright rather
  // than dropping the note; whatever is there is already fading toward zero.
  {
    int oldest = -1;
    unsigned int oldest_vid = 0;
    for (int i = 0; i < MAX_VOICES; i++)
      if (oldest < 0 || voices[i].vid < oldest_vid) {
        oldest = i;
        oldest_vid = voices[i].vid;
      }
    if (oldest >= 0) {
      voices[oldest].active = false;
      if (voices[oldest].pv) { pv_release(voices[oldest].pv); voices[oldest].pv = 0; }
      start_voice(ev); // the slot is free now; recursion depth is 1 by construction
    }
  }
}

// ShapeProcessor port (worklets.mjs:259-295): y = (1+k)x / (1+k|x|), ×shapevol
static inline float shape_drive(float x, float k, float vol) {
  return ((1.0f + k) * x) / (1.0f + k * (x < 0 ? -x : x)) * vol;
}

// ── THE DISTORTION FAMILY (superdough helpers.mjs:496-567, DistortProcessor
// worklets.mjs) — memoryless waveshapers, ported term-for-term. k arrives
// PRE-SHAPED: DistortProcessor computes shape = expm1(distort) per block and
// postgain = clamp(pg, .001, 1); both are computed ONCE at voice start here
// (zaltz events are static per voice — no param ramps to follow).
// Algorithm order = Object.keys(distortionAlgorithms):
//   0 scurve · 1 soft · 2 hard · 3 cubic · 4 diode · 5 asym · 6 fold ·
//   7 sinefold · 8 chebyshev
static inline float dist_squash(float x) { return x / (1.0f + x); } // [0,inf)→[0,1)
static inline float dist_mod4(float y) { return y - 4.0f * sd_floorf(y * 0.25f); } // _mod(y,4)
static inline float dist_scurve(float x, float k) {
  return ((1.0f + k) * x) / (1.0f + k * sd_fabsf(x));
}
static inline float dist_soft(float x, float k) { return sd_tanh_true(x * (1.0f + k)); }
static inline float dist_hard(float x, float k) { return sd_clampf((1.0f + k) * x, -1.0f, 1.0f); }
static inline float dist_fold(float x, float k) {
  float y = (1.0f + 0.5f * k) * x;
  float w = dist_mod4(y + 1.0f);
  return 1.0f - sd_fabsf(w - 2.0f);
}
static inline float dist_sinefold(float x, float k) {
  return sd_sinf((PI_F * 0.5f) * dist_fold(x, k));
}
static inline float dist_cubic(float x, float k) {
  float t = dist_squash(sd_log1pf(k));
  float cubic = (x - (t / 3.0f) * x * x * x) / (1.0f - t / 3.0f);
  return dist_soft(cubic, k);
}
static float dist_diode(float x, float k, bool asym) {
  float g = 1.0f + 2.0f * k;
  float t = dist_squash(sd_log1pf(k));
  float bias = 0.07f * t;
  float pos = dist_soft(x + bias, 2.0f * k);
  float neg = dist_soft(asym ? bias : -x + bias, 2.0f * k);
  float y = pos - neg;
  // divide by the derivative at 0 so small values pass undistorted
  float e = sd_exp2f(g * bias * 1.442695041f); // e^(g·bias)
  float sech = 2.0f / (e + 1.0f / e);          // 1/cosh
  float sech2 = sech * sech;
  float denom = sd_fmaxf(1e-8f, (asym ? 1.0f : 2.0f) * g * sech2);
  return dist_soft(y / denom, k);
}
static float dist_chebyshev(float x, float k) {
  float kl = 10.0f * sd_log1pf(k);
  float tnm1 = 1.0f, tnm2 = x, tn;
  float y = 0;
  for (int i = 1; i < 64; i++) {
    if (i < 2) { y += (i == 0) ? tnm1 : tnm2; continue; }
    tn = 2.0f * x * tnm1 - tnm2;
    tnm2 = tnm1;
    tnm1 = tn;
    if ((i & 1) == 0) y += sd_fminf((1.3f * kl) / (float)i, 2.0f) * tn;
  }
  return dist_soft(y, kl / 20.0f);
}
static inline float dist_run(int alg, float x, float k) {
  switch (alg) {
    case 1: return dist_soft(x, k);
    case 2: return dist_hard(x, k);
    case 3: return dist_cubic(x, k);
    case 4: return dist_diode(x, k, false);
    case 5: return dist_diode(x, k, true); // asym
    case 6: return dist_fold(x, k);
    case 7: return dist_sinefold(x, k);
    case 8: return dist_chebyshev(x, k);
    default: return dist_scurve(x, k);
  }
}

// LFO tri waveshape (worklets.mjs waveshapes.tri) — the tremolo's default
// (and only, here) modulator shape. The skew edges are handled EXPLICITLY:
// the double phase accumulator cast to float can round 0.99999… up to exactly
// 1.0f, and at skew 1 the general branch then divides by zero (one NaN per
// LFO wrap, seen in the offline harness at precisely the 0.25s wrap of 4Hz).
static inline float lfo_tri(float phase, float skew) {
  if (skew >= 0.999999f) return phase >= 1.0f ? 0.0f : phase;      // pure ramp
  if (skew <= 0.000001f) return phase >= 1.0f ? 1.0f : 1.0f - phase; // pure saw
  if (phase >= skew) {
    float x = 1.0f - skew;
    float y = 1.0f / x - phase / x;
    return y < 0 ? 0.0f : y;
  }
  return phase / skew;
}


// ---------- PHASE VOCODER — phaze port (worklets.mjs PhaseVocoderProcessor +
// ola-processor.js + fft.js), term for term ----------------------------------
// superdough: `.stretch(x)` inserts a phase-vocoder worklet as the FIRST fx of
// the sound chain, pitchFactor = x (transformed per block: x<0 → x·0.25, then
// max(0, x+1)), and shifts the hap onset 40ms early for the OLA latency
// (superdough.mjs:446-451 — the bridge clones that shift host-side).
// OLA: block 2048, hop 128 (= one WebAudio quantum), 16 overlaps, Hann ×1.62
// applied on analysis AND synthesis.
// PLACEMENT NOTE (documented deviation): here the PV runs on the voice's
// FINISHED stereo contribution (post filters/fx, pre orbit) instead of first
// in the chain — restructuring three fused render paths risks regressions the
// mandate forbids. For linear stages this commutes; a patch that stacks
// distortion ON TOP of stretch will color slightly differently.
#define PV_N 2048
#define PV_HOP 128 /* == BLOCK */
#define PV_OVER (PV_N / PV_HOP)
#define PV_HALF (PV_N / 2)
#define PV_MAX 64 /* beyond this many live stretch voices → new ones play dry */

struct Pv {
  float in_l[PV_N], in_r[PV_N];   // sliding analysis windows (newest at tail)
  float out_l[PV_N], out_r[PV_N]; // OLA accumulators
  double time_cursor;             // samples — phase-correction clock
  int drain;                      // silence blocks left after the source dies
};

static float pv_hann[PV_N];
// SPECTRAL PATH IN DOUBLE — fft.js runs float64, and the peak finder reads
// the numerical floor: a float32 FFT grows a forest of spurious micro-peaks
// there (98 vs the reference's 5 on pure sines, measured) whose regions of
// influence carve up the real peaks' phase corrections → 20% RMS divergence
// at down-shifts. Precision is part of the algorithm here.
static double pv_tw_cos[PV_HALF], pv_tw_sin[PV_HALF]; // e^{-i2πk/N} (forward)
static unsigned short pv_bitrev[PV_N];
static bool pv_tables_ready = false;

// shared per-block scratch — voices render sequentially on the audio thread
static double pv_re[PV_N], pv_im[PV_N];   // analysis spectrum
static double pv_re2[PV_N], pv_im2[PV_N]; // shifted spectrum
static float pv_mag[PV_HALF + 1]; // Float32Array in phaze — compared as f32
static int pv_peaks[PV_HALF + 1];
static int pv_dbg_npeaks = -1; // harness observability (last channel processed)
static bool pv_dbg_capture = false;
static double pv_dbg_sre[PV_N], pv_dbg_sim[PV_N]; // shifted-spectrum snapshot

// double sin/cos for the twiddles + phase factors: Taylor on [0, π/4] (8
// terms ≈ 4e-17) + exact-grid quadrant reduction. sd_sinf's float pipeline
// would put ~1e-7 noise straight into the peak floor (see above).
static double pv_sin_poly(double x) {
  double x2 = x * x;
  return x * (1.0 + x2 * (-1.0 / 6 + x2 * (1.0 / 120 + x2 * (-1.0 / 5040 + x2 * (1.0 / 362880 + x2 * (-1.0 / 39916800.0 + x2 * (1.0 / 6227020800.0 + x2 * (-1.0 / 1307674368000.0))))))));
}
static double pv_cos_poly(double x) {
  double x2 = x * x;
  return 1.0 + x2 * (-0.5 + x2 * (1.0 / 24 + x2 * (-1.0 / 720 + x2 * (1.0 / 40320 + x2 * (-1.0 / 3628800.0 + x2 * (1.0 / 479001600.0 + x2 * (-1.0 / 87178291200.0)))))));
}
#define PV_PI 3.14159265358979323846
static void pv_sincos64(double a, double *c, double *s) {
  // a reduced to [0, 2π) by the caller; split into quadrant + [0, π/4] wing
  int q = (int)(a / (PV_PI / 2)); // 0..3
  if (q > 3) q = 3;
  double b = a - (double)q * (PV_PI / 2);
  double cb, sb;
  if (b > PV_PI / 4) {
    double w = PV_PI / 2 - b;
    cb = pv_sin_poly(w);
    sb = pv_cos_poly(w);
  } else {
    cb = pv_cos_poly(b);
    sb = pv_sin_poly(b);
  }
  switch (q) {
    case 0: *c = cb;  *s = sb;  break;
    case 1: *c = -sb; *s = cb;  break;
    case 2: *c = -cb; *s = -sb; break;
    default: *c = sb; *s = -cb; break;
  }
}
static float pv_stage_l[BLOCK], pv_stage_r[BLOCK]; // voice staging pre-PV

static Pv *pv_freelist[PV_MAX];
static int pv_nfree = 0;
static int pv_total = 0;

// fft.js clone — the ANALYSIS transform must be indutny's _realTransform4
// EXACTLY: it computes only the lower half-spectrum properly and leaves
// deterministic radix-4 INTERMEDIATES in the upper bins, which phaze's
// shiftPeaks then READS for the last peak's region of influence. On
// down-shifts that junk folds into the audible band — it is part of the
// stretch sound on strudel.cc (measured: ref bins ~789 carried mag ~92 of
// it while a mathematically-correct FFT left zeros → 22% RMS divergence).
#define PV_CSIZE (2 * PV_N)
#define PV_WIDTH 11 // power of 2048; odd → initial len=4 radix-2 pass
static double pv_fft_table[PV_CSIZE]; // [cos(πi/N), −sin(πi/N)] pairs
static int pv_bitrev4[1 << PV_WIDTH];
static double pv_spec[PV_CSIZE]; // interleaved complex spectrum (JS `out`)
static double pv_win[PV_N];      // windowed real input (JS `data`)

// JS `x << s` semantics: shift count masked to 5 bits (the table builder hits
// revShift = −1 on the last digit pair; operand is 0 for reachable indices,
// but the construction is cloned without UB)
static inline int js_shl(int x, int s) { return (int)((unsigned)x << ((unsigned)s & 31u)); }

static void pv_tables_init(void) {
  if (pv_tables_ready) return;
  pv_tables_ready = true;
  // radix-2 twiddles + bitrev for the INVERSE (mathematically identical to
  // fft.js _transform4 on real spectra — rounding-level only, verified)
  for (int i = 0; i < PV_N; i++) {
    unsigned int r = 0, x = (unsigned int)i;
    for (int b = 0; b < 11; b++) { r = (r << 1) | (x & 1u); x >>= 1; } // 2^11 = 2048
    pv_bitrev[i] = (unsigned short)r;
  }
  for (int k = 0; k < PV_HALF; k++) {
    double a = TWO_PI * (double)k / (double)PV_N;
    double c, s;
    pv_sincos64(a, &c, &s);
    pv_tw_cos[k] = c;
    pv_tw_sin[k] = -s; // forward convention e^{-iωk}
  }
  // fft.js constructor: table[i] = cos(πi/size), table[i+1] = −sin(πi/size)
  for (int i = 0; i < PV_CSIZE; i += 2) {
    double a = PV_PI * (double)i / (double)PV_N; // < 2π
    double c, s;
    pv_sincos64(a, &c, &s);
    pv_fft_table[i] = c;
    pv_fft_table[i + 1] = -s;
  }
  // fft.js base-4 digit reversal, width 11
  for (int j = 0; j < (1 << PV_WIDTH); j++) {
    int r = 0;
    for (int shift = 0; shift < PV_WIDTH; shift += 2) {
      int rev = PV_WIDTH - shift - 2;
      r |= js_shl((j >> shift) & 3, rev);
    }
    pv_bitrev4[j] = r;
  }
  for (int i = 0; i < PV_N; i++) {
    // genHannWindow: 0.5·(1 − cos(2πi/N)) — periodic Hann, length N, f32 store
    double a = TWO_PI * (double)i / (double)PV_N;
    double c, s;
    pv_sincos64(a, &c, &s);
    pv_hann[i] = (float)(0.5 * (1.0 - c));
  }
}

// fft.js _singleRealTransform2 (initial pass, len=4, odd width)
static inline void pv_srt2(double *out, const double *data, int outOff, int off, int step) {
  double evenR = data[off];
  double oddR = data[off + step];
  out[outOff] = evenR + oddR;
  out[outOff + 1] = 0;
  out[outOff + 2] = evenR - oddR;
  out[outOff + 3] = 0;
}

// fft.js _realTransform4, forward only (inv = 1), verbatim
static void pv_real_transform4(double *out, const double *data) {
  const int size = PV_CSIZE;
  int step = 1 << PV_WIDTH;
  int len = (size / step) << 1; // 4 for width 11
  int outOff, t;
  for (outOff = 0, t = 0; outOff < size; outOff += len, t++) {
    int off = pv_bitrev4[t];
    pv_srt2(out, data, outOff, off >> 1, step >> 1);
  }
  const double inv = 1.0;
  for (step >>= 2; step >= 2; step >>= 2) {
    len = (size / step) << 1;
    int halfLen = len >> 1;
    int quarterLen = halfLen >> 1;
    int hquarterLen = quarterLen >> 1;
    for (outOff = 0; outOff < size; outOff += len) {
      for (int i = 0, k = 0; i <= hquarterLen; i += 2, k += step) {
        int A = outOff + i, B = A + quarterLen, C = B + quarterLen, D = C + quarterLen;
        double Ar = out[A], Ai = out[A + 1];
        double Br = out[B], Bi = out[B + 1];
        double Cr = out[C], Ci = out[C + 1];
        double Dr = out[D], Di = out[D + 1];
        double MAr = Ar, MAi = Ai;
        double tBr = pv_fft_table[k], tBi = inv * pv_fft_table[k + 1];
        double MBr = Br * tBr - Bi * tBi, MBi = Br * tBi + Bi * tBr;
        double tCr = pv_fft_table[2 * k], tCi = inv * pv_fft_table[2 * k + 1];
        double MCr = Cr * tCr - Ci * tCi, MCi = Cr * tCi + Ci * tCr;
        double tDr = pv_fft_table[3 * k], tDi = inv * pv_fft_table[3 * k + 1];
        double MDr = Dr * tDr - Di * tDi, MDi = Dr * tDi + Di * tDr;
        double T0r = MAr + MCr, T0i = MAi + MCi;
        double T1r = MAr - MCr, T1i = MAi - MCi;
        double T2r = MBr + MDr, T2i = MBi + MDi;
        double T3r = inv * (MBr - MDr), T3i = inv * (MBi - MDi);
        double FAr = T0r + T2r, FAi = T0i + T2i;
        double FBr = T1r + T3i, FBi = T1i - T3r;
        out[A] = FAr; out[A + 1] = FAi;
        out[B] = FBr; out[B + 1] = FBi;
        if (i == 0) {
          out[C] = T0r - T2r;
          out[C + 1] = T0i - T2i;
          continue;
        }
        if (i == hquarterLen) continue; // do not overwrite ourselves
        double ST0r = T1r, ST0i = -T1i;
        double ST1r = T0r, ST1i = -T0i;
        double ST2r = -inv * T3i, ST2i = -inv * T3r;
        double ST3r = -inv * T2i, ST3i = -inv * T2r;
        double SFAr = ST0r + ST2r, SFAi = ST0i + ST2i;
        double SFBr = ST1r + ST3i, SFBi = ST1i - ST3r;
        int SA = outOff + quarterLen - i, SB = outOff + halfLen - i;
        out[SA] = SFAr; out[SA + 1] = SFAi;
        out[SB] = SFBr; out[SB + 1] = SFBi;
      }
    }
  }
}

// iterative radix-2 complex FFT, in place, DOUBLE; inv flips the twiddle sign
// and scales by 1/N (fft.js inverseTransform normalizes the same way)
static void pv_fft(double *re, double *im, bool inv) {
  for (int i = 0; i < PV_N; i++) {
    int j = pv_bitrev[i];
    if (j > i) {
      double tr = re[i]; re[i] = re[j]; re[j] = tr;
      double ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }
  for (int len = 2; len <= PV_N; len <<= 1) {
    int half = len >> 1, step = PV_N / len;
    for (int base = 0; base < PV_N; base += len) {
      for (int k = 0; k < half; k++) {
        double wr = pv_tw_cos[k * step];
        double wi = pv_tw_sin[k * step];
        if (inv) wi = -wi;
        int a = base + k, b = a + half;
        double xr = re[b] * wr - im[b] * wi;
        double xi = re[b] * wi + im[b] * wr;
        re[b] = re[a] - xr; im[b] = im[a] - xi;
        re[a] += xr;        im[a] += xi;
      }
    }
  }
  if (inv) {
    double s = 1.0 / (double)PV_N;
    for (int i = 0; i < PV_N; i++) { re[i] *= s; im[i] *= s; }
  }
}

static Pv *pv_alloc(void) {
  Pv *p;
  if (pv_nfree > 0) p = pv_freelist[--pv_nfree];
  else {
    if (pv_total >= PV_MAX) return 0;
    p = (Pv *)arena_take((long)((sizeof(Pv) + 3) / 4));
    if (!p) return 0;
    pv_total++;
  }
  for (int i = 0; i < PV_N; i++) {
    p->in_l[i] = 0; p->in_r[i] = 0; p->out_l[i] = 0; p->out_r[i] = 0;
  }
  p->time_cursor = 0;
  p->drain = -1;
  return p;
}

static void pv_release(Pv *p) {
  if (p && pv_nfree < PV_MAX) pv_freelist[pv_nfree++] = p;
}

// phaze helpers cloned EXACTLY: fround = floor(x+0.5), fceil = floor(x+1)
// (worklets.mjs:28-29 — note fceil(2) = 3; Math.ceil would give 2)
static inline float pv_jround(float x) { return sd_floorf(x + 0.5f); }
static inline float pv_jceil(float x) { return sd_floorf(x + 1.0f); }

// cos/sin of omegaDelta·timeCursor — the argument grows unbounded, so reduce
// mod 2π in double, then the double Taylor pipeline (float trig would seed
// the shifted spectrum with 1e-7 phase noise)
static inline void pv_phasor(double a, double *c, double *s) {
  double t = a / TWO_PI;
  t -= __builtin_floor(t);
  pv_sincos64(t * TWO_PI, c, s);
}

// one channel: analysis window → FFT → peak shift → IFFT → synthesis window
static void pv_channel(float *ring, float pf, double time_cursor, float *out_frame) {
  for (int i = 0; i < PV_N; i++) {
    // applyHannWindow runs on a Float32Array: value = f32(x·(hann·1.62)) —
    // clone the rounding, THEN promote to double for the transform
    pv_win[i] = (double)(float)((double)ring[i] * ((double)pv_hann[i] * 1.62));
  }
  // indutny realTransform — upper-half bins carry its radix-4 intermediates,
  // which the peak regions below deliberately read (see pv_real_transform4)
  pv_real_transform4(pv_spec, pv_win);
  for (int k = 0; k < PV_N; k++) {
    pv_re[k] = pv_spec[2 * k];
    pv_im[k] = pv_spec[2 * k + 1];
  }
  // magnitudes land in a Float32Array in phaze — the peak comparisons happen
  // at f32, cloned exactly
  for (int k = 0; k <= PV_HALF; k++) pv_mag[k] = (float)(pv_re[k] * pv_re[k] + pv_im[k] * pv_im[k]);
  // findPeaks: strictly greater than the 2 neighbours each side, i starts 2
  int npeaks = 0;
  pv_dbg_npeaks = -1; /* set below; harness-only observability */
  {
    int i = 2;
    const int end = PV_HALF + 1 - 2; // magnitudes.length - 2
    while (i < end) {
      float m = pv_mag[i];
      if (pv_mag[i - 1] >= m || pv_mag[i - 2] >= m) { i++; continue; }
      if (pv_mag[i + 1] >= m || pv_mag[i + 2] >= m) { i++; continue; }
      pv_peaks[npeaks++] = i;
      i += 2;
    }
  }
  pv_dbg_npeaks = npeaks;
  // shiftPeaks
  for (int i = 0; i < PV_N; i++) { pv_re2[i] = 0; pv_im2[i] = 0; }
  for (int pi = 0; pi < npeaks; pi++) {
    int peak = pv_peaks[pi];
    int shifted = (int)pv_jround((float)peak * pf);
    if (shifted > PV_HALF + 1) break; // `> this.magnitudes.length` verbatim
    int start_i = 0, end_i = PV_N;
    if (pi > 0) start_i = peak - (int)pv_jround((float)(peak - pv_peaks[pi - 1]) / 2.0f);
    if (pi < npeaks - 1) end_i = peak + (int)pv_jceil((float)(pv_peaks[pi + 1] - peak) / 2.0f);
    int start_off = start_i - peak, end_off = end_i - peak;
    double omega_delta = TWO_PI * (1.0 / (double)PV_N) * (double)(shifted - peak);
    double ps_r, ps_i;
    pv_phasor(omega_delta * time_cursor, &ps_r, &ps_i);
    for (int j = start_off; j < end_off; j++) {
      int bin = peak + j;
      int bin_s = shifted + j;
      if (bin_s >= PV_HALF + 1) break;
      if (bin < 0 || bin_s < 0) continue; // JS negative indices vanish into Array properties; skip
      double vr = pv_re[bin], vi = pv_im[bin];
      pv_re2[bin_s] += vr * ps_r - vi * ps_i;
      pv_im2[bin_s] += vr * ps_i + vi * ps_r;
    }
  }
  // completeSpectrum: conjugate-mirror bins 1..N/2−1 (Nyquist untouched)
  for (int k = 1; k < PV_HALF; k++) {
    pv_re2[PV_N - k] = pv_re2[k];
    pv_im2[PV_N - k] = -pv_im2[k];
  }
  if (pv_dbg_capture) // harness-only: the inverse below destroys the spectrum
    for (int i = 0; i < PV_N; i++) { pv_dbg_sre[i] = pv_re2[i]; pv_dbg_sim[i] = pv_im2[i]; }
  pv_fft(pv_re2, pv_im2, true);
  // fromComplexArray lands doubles in a Float32Array, then applyHannWindow
  // multiplies that f32 — clone both roundings
  for (int i = 0; i < PV_N; i++)
    out_frame[i] = (float)((double)(float)pv_re2[i] * ((double)pv_hann[i] * 1.62));
}

// one OLA hop: feed 128 staged samples, emit 128 output. A mono voice
// (stereo=false) runs the L pipeline only and mirrors it to R.
static void pv_process(Pv *p, const float *inl, const float *inr, float *outl, float *outr, float stretch, bool stereo) {
  // pitchFactor transform (processOLA): x<0 → x·0.25, then max(0, x+1)
  float pf = stretch;
  if (pf < 0) pf *= 0.25f;
  pf += 1.0f;
  if (pf < 0) pf = 0;
  float *rings[2]; const float *ins[2]; float *outs[2]; float *acc[2];
  rings[0] = p->in_l; rings[1] = p->in_r;
  ins[0] = inl; ins[1] = inr;
  outs[0] = outl; outs[1] = outr;
  acc[0] = p->out_l; acc[1] = p->out_r;
  static float frame[PV_N];
  const int nch = stereo ? 2 : 1;
  for (int ch = 0; ch < nch; ch++) {
    float *ring = rings[ch];
    // slide the analysis window forward one hop (readInputs + shiftInputBuffers)
    for (int i = 0; i < PV_N - PV_HOP; i++) ring[i] = ring[i + PV_HOP];
    for (int i = 0; i < PV_HOP; i++) ring[PV_N - PV_HOP + i] = ins[ch][i];
    pv_channel(ring, pf, p->time_cursor, frame);
    // handleOutputBuffersToRetrieve: accumulate ÷ overlaps, emit head, shift
    float *a = acc[ch];
    for (int i = 0; i < PV_N; i++) a[i] += frame[i] * (1.0f / (float)PV_OVER);
    for (int i = 0; i < PV_HOP; i++) {
      float y = a[i];
      outs[ch][i] = y == y ? y : 0; // NaN can never reach the graph (law)
    }
    for (int i = 0; i < PV_N - PV_HOP; i++) a[i] = a[i + PV_HOP];
    for (int i = PV_N - PV_HOP; i < PV_N; i++) a[i] = 0;
  }
  if (!stereo)
    for (int i = 0; i < PV_HOP; i++) outr[i] = outl[i]; // mono mirror
  p->time_cursor += (double)PV_HOP;
}

// THE VOICE CHAIN, IN SUPERDOUGH'S ORDER (superdough.mjs:585-930, read top to
// bottom): source → gain(gain·velocity) → lpf → hpf → bpf → vowel → coarse →
// crush → shape → distort → tremolo → compressor → PAN → phaser → postgain →
// orbit sends. Order is not cosmetic here: every one of coarse/crush/shape/
// distort is NONLINEAR, so moving a gain across one changes the sound, not
// just the level. zaltz used to run shape → postgain → pan → phaser → distort
// → tremolo → crush → coarse, which fed `postgain` INTO the distortion —
// `.soft(.6).postgain(1.2)` saturated harder and came out ~1.2dB quieter than
// strudel.cc (the user's kick with "no umph"). Split in two around the pan
// stage, because the pan law differs per source path.
//
// voice_pre_pan: everything from coarse through tremolo. All stages are
// channel-symmetric; a mono voice passes the same pointer twice-safe values
// via voice_pre_pan_mono, which keeps the per-sample state (coarse counter,
// tremolo phase) advancing exactly once.
static inline void voice_pre_pan(Voice *v, float *al, float *ar) {
  if (v->coarse >= 2) { // superdough: coarse BEFORE crush, both before shape
    if (v->coarse_ctr == 0) {
      v->coarse_hold_l = *al;
      v->coarse_hold_r = *ar;
    }
    *al = v->coarse_hold_l;
    *ar = v->coarse_hold_r;
    v->coarse_ctr++;
    if (v->coarse_ctr >= v->coarse) v->coarse_ctr = 0;
  }
  if (v->crush_on) {
    float x = sd_exp2f(v->crush - 1.0f); // CrushProcessor: Math.round(x·2^(crush−1))/2^(crush−1)
    *al = sd_floorf(*al * x + 0.5f) / x; // Math.round: halves go UP (−2.5 → −2)
    *ar = sd_floorf(*ar * x + 0.5f) / x;
  }
  if (v->shape_on) {
    *al = shape_drive(*al, v->shape_k, v->shapevol);
    *ar = shape_drive(*ar, v->shape_k, v->shapevol);
  }
  // DISTORTION FAMILY (DistortProcessor): y = postgain·algo(x, k)
  if (v->dist_on) {
    *al = v->dist_pg * dist_run(v->dist_alg, *al, v->dist_k);
    *ar = v->dist_pg * dist_run(v->dist_alg, *ar, v->dist_k);
  }
  // TREMOLO: base amGain max(1−depth,0) + LFO tri(phase, skew), curved 1.5
  // (LFOProcessor: modval = pow(tri·depth, 1.5), clamped [0,1])
  if (v->trem_on) {
    float ph = (float)v->trem_phase, shp; // worklets.mjs waveshapes
    switch (v->trem_shape) {
      case 1: shp = sd_sinf(TWO_PI * ph) * 0.5f + 0.5f; break;  // sine
      case 2: shp = ph; break;                                   // ramp
      case 3: shp = 1.0f - ph; break;                            // saw
      case 4: shp = ph >= v->trem_skew ? 0.0f : 1.0f; break;     // square
      default: shp = lfo_tri(ph, v->trem_skew);                  // tri
    }
    float w = shp * v->trem_depth;
    if (w < 0) w = 0;
    w = w * sd_sqrtf(w); // pow(x, 1.5) for x ≥ 0
    if (w > 1) w = 1;
    float g = v->trem_base + w;
    *al *= g;
    *ar *= g;
    v->trem_phase += (double)(v->trem_rate / sr_f);
    if (v->trem_phase >= 1.0) v->trem_phase -= 1.0;
  }
}

/** Mono form: one channel in/out, per-sample state advanced exactly once. */
static inline void voice_pre_pan_mono(Voice *v, float *x) {
  float dummy = *x;
  voice_pre_pan(v, x, &dummy);
}

// voice_post_pan: phaser → postgain → the klappn-only fades. The fades sit at
// the very end so a crossfade or a steal is TRANSPARENT — a gain ride ahead of
// a nonlinearity would change the timbre as it moves.
static inline void voice_post_pan(Voice *v, double f, float *al, float *ar) {
  if (v->phaser_on) {
    *al = biquad_run(&v->ph_l, *al);
    *ar = biquad_run(&v->ph_r, *ar);
  }
  *al *= v->postgain; // superdough: `post = GainNode(postgain)`, last before sends
  *ar *= v->postgain;
  if (v->retire_start >= 0) {
    float k = 1.0f - (float)(f - v->retire_start) * retire_inv;
    if (k <= 0) { v->active = false; *al = 0; *ar = 0; return; }
    *al *= k;
    *ar *= k;
  }
  if (v->steal_at >= 0) { // superdough voice steal: 0.25s linear ramp to 0
    float k = 1.0f - (float)((f - v->steal_at) / ((double)STEAL_FADE * (double)sr_f));
    if (k <= 0) {
      v->active = false;
      if (v->pv) { pv_release(v->pv); v->pv = 0; }
      *al = 0; *ar = 0;
      return;
    }
    *al *= k;
    *ar *= k;
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

// A voice's channels can only differ when something in its FIXED path splits
// them: a panner, a stereo sample, or the supersaw's alternating gains. Every
// per-sample fx stage is channel-symmetric, so a mono voice's L==R forever —
// the vocoder then runs ONE spectral pipeline instead of two (the hats and
// claps this exists for are exactly these voices; halves the audio-thread
// cost per stretch voice).
static inline bool pv_voice_stereo(const Voice *v) {
  return v->pan_set || v->src == SRC_SUPERSAW ||
         (v->src == SRC_SAMPLE && v->pcm_channels == 2);
}

// the render loops emit through here: stretch voices STAGE their block for
// the phase vocoder (pv_flush below), everything else lands on the orbit now
static inline void voice_emit(const Voice *v, int i, float al, float ar) {
  if (v->pv) {
    pv_stage_l[i] = al;
    pv_stage_r[i] = ar;
    return;
  }
  voice_out(v, i, al, ar);
}

// after a stretch voice's sample loop: vocode the staged block onto the orbit.
// When the source dies the OLA tail (16 hops of buffered audio) still drains —
// superdough's per-hap worklet node rings out the same way before GC.
// THE HUSK BUG (2026-07-29, "plays nicely for a bit and then it just stops"):
// the drain used to tick only when v->active was false — but the drain loop
// itself re-marks the voice active, and the pv_dead break leaves it that way,
// so the counter NEVER decremented. Every finished stretch voice squatted on
// a voice slot forever; at ~4 stretch haps/s the 128 slots choked in minutes
// and the whole room fell silent. The drain now ticks on pv_dead, always.
static void pv_flush(Voice *v) {
  Pv *p = v->pv;
  float ol[BLOCK], orr[BLOCK];
  pv_process(p, pv_stage_l, pv_stage_r, ol, orr, v->pv_stretch, pv_voice_stereo(v));
  for (int i = 0; i < BLOCK; i++) voice_out(v, i, ol[i], orr[i]);
  if (v->pv_dead) {
    if (--p->drain <= 0) {
      pv_release(p);
      v->pv = 0;
      v->active = false; // truly done — the slot is free again
    }
    return;
  }
  if (!v->active) {
    v->pv_dead = true; // the i-loop now breaks instantly → staged silence
    p->drain = PV_OVER;
    v->active = true; // live until the tail drains
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
  stem_mask_lo = 0;
  stem_mask_hi = 0;

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
    const bool pv_on = v->pv != 0;
    if (pv_on) // pre-zero: a mid-block start/stop leaves true silence staged
      for (int i = 0; i < BLOCK; i++) { pv_stage_l[i] = 0; pv_stage_r[i] = 0; }
    for (int i = 0; i < BLOCK; i++) {
      if (v->pv_dead) break; // OLA drain: source silent, vocoder still ringing
      double f = engine_frame + i;
      if (f < v->start_frame) continue;
      float tt = (float)((f - v->start_frame) / (double)sr_f);
      if (tt >= v->end) { v->active = false; break; }

      // CONTROL RATE (every 16 samples): vib detune + filter-envelope sweep —
      // WebAudio automates these a-rate; 16-sample control is inaudible for
      // exponential glides and keeps coefficients cheap
      if (((int)(f - v->start_frame) & 15) == 0) {
        if (v->vib_hz > 0 || v->penv_on) {
          // both modulators land on ONE detune param (cents), summed
          float cents_sum = 0.0f;
          if (v->vib_hz > 0) cents_sum += sd_sinf(TWO_PI * v->vib_hz * tt) * v->vibmod * 100.0f;
          if (v->penv_on) {
            // getPitchEnvelope: detune cents ride the param ADSR between
            // min=−cents·anchor and max=cents−cents·anchor (linear curve;
            // pcurve exponential falls back to linear here — negative-cents
            // ranges can't ride an exponential ramp anyway)
            float e01 = adsr_at(&v->penv_env, tt, v->dur);
            cents_sum += v->penv_min + (v->penv_max - v->penv_min) * e01;
          }
          // the SUPERSAW worklet's detune AudioParam has min 0 (worklets.mjs:
          // 514) and Web Audio clamps to the nominal range — vibrato only bends
          // UP there, and a default penv (anchor = sustain = 1 → range ≤ 0)
          // does nothing at all. Oscillator/buffer detune params are unbounded.
          if (v->src == SRC_SUPERSAW && cents_sum < 0) cents_sum = 0;
          float mult = sd_exp2f(cents_sum / 1200.0f);
          if (v->src == SRC_SAMPLE) v->rate = v->base_rate * (double)mult;
          else v->pitch_mult = mult;
        }
        if (v->fm_on && v->fm_env_on)
          v->fm_envval = v->fm_env_lin ? adsr_at(&v->fm_env, tt, v->dur)
                                       : adsr_exp_at(&v->fm_env, tt, v->dur, 0.0f, 1.0f);
        // without FM the frequency only moves with detune → control rate
        if (!v->fm_on && v->src != SRC_SAMPLE) {
          double f0 = v->base_freq;
          if ((v->src == SRC_SUPERSAW || v->src == SRC_PULSE) && f0 < 1e-9) f0 = 1e-9;
          f0 *= (double)v->pitch_mult;
          const double nq = 0.5 * (double)sr_f;
          v->phase_inc = (f0 > nq ? nq : f0 < -nq ? -nq : f0) * inv_sr;
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
        for (int fk = 0; fk < 3; fk++) filt_env(&v->flt[fk], tt, v->dur, sr_f);
      }
      if (v->src == SRC_SAMPLE) {
        float xs = 0, xs_r = 0;
        bool nudged = (f - v->start_frame) < v->nudge_frames; // the buffer hasn't started yet
        bool in_range = !nudged && v->pos >= 0 && v->pos < (double)v->pcm_frames;
        if (in_range) {
          int i0 = (int)v->pos;
          float fr = (float)(v->pos - (double)i0);
          int i1 = i0 + 1 < v->pcm_frames ? i0 + 1 : i0;
          if (v->rev) { i0 = v->pcm_frames - 1 - i0; i1 = v->pcm_frames - 1 - i1; } // reversed buffer
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
        for (int fk = 0; fk < 3; fk++) {
          if (!v->flt[fk].on) continue;
          al = filt_run(&v->flt[fk], al, 0);
          ar = v->pcm_channels == 1 ? al : filt_run(&v->flt[fk], ar, 1);
        }
        if (v->vow.on) {
          al = vowel_run(&v->vow, al, 0);
          ar = v->pcm_channels == 1 ? al : vowel_run(&v->vow, ar, 1);
        }
        // coarse → crush → shape → distort → tremolo (superdough's order)
        if (v->pcm_channels == 1) {
          voice_pre_pan_mono(v, &al);
          ar = al;
        } else {
          voice_pre_pan(v, &al, &ar);
        }
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
        voice_post_pan(v, engine_frame + i, &al, &ar);
        voice_emit(v, i, al, ar);
        continue;
      }
      // INSTANTANEOUS FREQUENCY = (param value + FM) · 2^(detune/1200). The
      // oscillator clamps its computed frequency to ±Nyquist; the supersaw and
      // pulse worklets floor their frequency param at EPSILON first.
      double fi;
      if (!v->fm_on) fi = v->phase_inc * (double)sr_f; // control-rate value (above)
      else {
      fi = v->base_freq;
      {
        double mp = v->fm_phase;
        float m;
        switch (v->fm_wave) {
          case 1: m = (mp < 0.5 ? 1.0f : -1.0f) * WA_NORM; break;                     // square
          case 2: m = (float)(2.0 * (mp + 0.5 - (double)(int)(mp + 0.5)) - 1.0) * WA_NORM; break; // saw (0, rising)
          case 3: m = mp < 0.25 ? (float)(4.0 * mp) : mp < 0.75 ? (float)(2.0 - 4.0 * mp) : (float)(4.0 * mp - 4.0); break;
          case 4: m = nz_rand(); break;
          case 5: {
            float w = nz_rand(); float *b = v->fm_nb;
            b[0] = 0.99886f * b[0] + w * 0.0555179f; b[1] = 0.99332f * b[1] + w * 0.0750759f;
            b[2] = 0.969f * b[2] + w * 0.153852f;    b[3] = 0.8665f * b[3] + w * 0.3104856f;
            b[4] = 0.55f * b[4] + w * 0.5329522f;    b[5] = -0.7616f * b[5] - w * 0.016898f;
            m = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362f) * 0.11f;
            b[6] = w * 0.115926f;
            break;
          }
          case 6: { float w = nz_rand(); m = (v->fm_nlast + 0.02f * w) / 1.02f; v->fm_nlast = m; break; }
          case 7: { float u = nz_rand() * 0.5f + 0.5f; m = u < 0.02f ? nz_rand() : 0.0f; break; } // density 2
          default: m = sd_sinf(TWO_PI * (float)mp);
        }
        fi += (double)(v->fm_dev * m * v->fm_envval);
        v->fm_phase += (double)v->fm_modfreq / (double)sr_f;
        v->fm_phase -= (double)(long long)v->fm_phase;
        if (v->fm_phase < 0) v->fm_phase += 1.0;
      }
      if (v->src == SRC_SUPERSAW || v->src == SRC_PULSE) { if (fi < 1e-9) fi = 1e-9; }
      fi *= (double)v->pitch_mult;
      const double nyq = 0.5 * (double)sr_f;
      if (fi > nyq) fi = nyq;
      if (fi < -nyq) fi = -nyq;
      v->phase_inc = fi * inv_sr;
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
        for (int fk = 0; fk < 3; fk++) {
          if (!v->flt[fk].on) continue;
          al = filt_run(&v->flt[fk], al, 0);
          ar = filt_run(&v->flt[fk], ar, 1);
        }
        if (v->vow.on) { al = vowel_run(&v->vow, al, 0); ar = vowel_run(&v->vow, ar, 1); }
        voice_pre_pan(v, &al, &ar); // coarse → crush → shape → distort → tremolo
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
        voice_post_pan(v, engine_frame + i, &al, &ar);
        voice_emit(v, i, al, ar);
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
        case SRC_PULSE: {
          // PulseOscillatorProcessor, sample for sample. The worklet only runs
          // for render quanta that START after `begin` (currentTime <= begin →
          // silent) and its `env` restarts at 1 every quantum.
          if (!v->pl_live) {
            if (engine_frame > v->start_frame) v->pl_live = true;
            else { s = 0; break; }
          }
          if (i == 0) v->pl_env = 1.0;
          double pwv = v->pl_pw;
          if (v->pw_lfo) { // getLfo: (tri(φ,.5) − .5)·depth, clamped to ±depth/2
            double ph = v->pw_phase;
            double tri = ph >= 0.5 ? 2.0 - 2.0 * ph : 2.0 * ph;
            pwv += (tri - 0.5) * (double)v->pw_depth;
            v->pw_phase += (double)v->pw_rate / (double)sr_f;
            if (v->pw_phase > 1.0) v->pw_phase -= 1.0;
          }
          if (pwv < 0) pwv = 0; // the pulsewidth param's min
          double pwc = pwv > 0.99 ? 0.99 : pwv;
          const double PI_D = 3.14159265358979323846;
          double pwr = (1.0 - pwc) * PI_D;
          double freq = fi; // frequency param (+FM, floored) through the detune
          double dphi = freq * 2.0 * PI_D / (double)sr_f;
          v->pl_dphif += 0.1 * (dphi - v->pl_dphif);
          v->pl_env *= 0.9998;
          v->pl_envf += 0.1 * (v->pl_env - v->pl_envf);
          double B = 2.3 * (1.0 - 0.0001 * freq);
          if (B < 0) B = 0;
          v->pl_phi += v->pl_dphif;
          if (v->pl_phi >= PI_D) v->pl_phi -= 2.0 * PI_D;
          double out0 = sd_cos_d(v->pl_phi + B * v->pl_y0);
          v->pl_y0 = 0.5 * (out0 + v->pl_y0);
          double out1 = sd_cos_d(v->pl_phi + B * v->pl_y1 + pwr);
          v->pl_y1 = 0.5 * (out1 + v->pl_y1);
          s = (float)(0.15 * (out0 - out1) * v->pl_envf);
          break;
        }
        case SRC_SAW: {
          float pos = (float)(t * (double)WT_LEN);
          int i0 = (int)pos;
          if (i0 >= WT_LEN) i0 = WT_LEN - 1;
          float fr = pos - (float)i0;
          if (v->ptab) { s = v->ptab[i0] + (v->ptab[i0 + 1] - v->ptab[i0]) * fr; break; }
          const float *tab = wt_saw[v->wt_lvl];
          s = (tab[i0] + (tab[i0 + 1] - tab[i0]) * fr) * WA_NORM;
          break;
        }
        case SRC_SQUARE: {
          if (v->ptab) {
            float pos = (float)(t * (double)WT_LEN);
            int i0 = (int)pos;
            if (i0 >= WT_LEN) i0 = WT_LEN - 1;
            float fr = pos - (float)i0;
            s = v->ptab[i0] + (v->ptab[i0 + 1] - v->ptab[i0]) * fr;
            break;
          }
          double adt = dt < 0 ? -dt : dt;
          s = t < 0.5 ? 1.0f : -1.0f;
          s += polyblep(t, adt);
          double t2 = t + 0.5; if (t2 >= 1.0) t2 -= 1.0;
          s -= polyblep(t2, adt);
          s *= WA_NORM;
          break;
        }
        default: { // triangle — SPEC phase: starts 0 rising, peak +1 at t=0.25
          if (v->ptab) {
            float pos = (float)(t * (double)WT_LEN);
            int i0 = (int)pos;
            if (i0 >= WT_LEN) i0 = WT_LEN - 1;
            float fr = pos - (float)i0;
            s = v->ptab[i0] + (v->ptab[i0 + 1] - v->ptab[i0]) * fr;
            break;
          }
          // (the sin-series limit; the old −1-start was 90° late and shifted
          // steady-state sums with octave-related saw partials)
          float ph = (float)t;
          s = ph < 0.25f ? (4.0f * ph) : ph < 0.75f ? (2.0f - 4.0f * ph) : (4.0f * ph - 4.0f);
        }
      }
      v->phase += dt;
      if (v->phase >= 1.0) v->phase -= 1.0;
      if (v->phase < 0.0) v->phase += 1.0; // deep FM runs the carrier backwards
      if (v->nmix_on) { // drywet(osc, pink, noise) — the oscillator's own mix, pre-envelope
        float w = nz_rand(); float *b = v->nz_b;
        b[0] = 0.99886f * b[0] + w * 0.0555179f; b[1] = 0.99332f * b[1] + w * 0.0750759f;
        b[2] = 0.969f * b[2] + w * 0.153852f;    b[3] = 0.8665f * b[3] + w * 0.3104856f;
        b[4] = 0.55f * b[4] + w * 0.5329522f;    b[5] = -0.7616f * b[5] - w * 0.016898f;
        float pk = (b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362f) * 0.11f;
        b[6] = w * 0.115926f;
        s = s * v->nmix_dry + pk * v->nmix_wet;
      }

      float env = adsr_at(&v->env, tt, v->dur);
      float x = s * v->amp * env;
      for (int fk = 0; fk < 3; fk++)
        if (v->flt[fk].on) x = filt_run(&v->flt[fk], x, 0);
      if (v->vow.on) x = vowel_run(&v->vow, x, 0);
      voice_pre_pan_mono(v, &x); // coarse → crush → shape → distort → tremolo
      float al = x * v->pan_l, ar = x * v->pan_r;
      voice_post_pan(v, engine_frame + i, &al, &ar);
      voice_emit(v, i, al, ar);
    }
    if (pv_on) pv_flush(v); // stretch voices: vocode the staged block now
  }
  // ---- BUS PASS: per used orbit — delay ring, FDN reverb, duck, mix ----
  for (int oi = 0; oi < MAX_ORBITS; oi++) {
    Orbit *o = &orbits[oi];
    if (!o->used) continue;
    float *sb = stem_buf + oi * BLOCK * OUT_CH;
    if (stems_on) {
      if (oi < 32) stem_mask_lo |= 1u << oi;
      else stem_mask_hi |= 1u << (oi - 32);
    }
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
      float sl = o->dry[i * OUT_CH] + dl + wl;
      float sr2 = o->dry[i * OUT_CH + 1] + dr + wr;
      if (o->djf_on) {
        if (i == 0 && engine_frame >= o->djf_at) o->djf_val = o->djf_next; // k-rate value[0]
        float dv = sd_clampf(o->djf_val, 0.0f, 1.0f);
        int ft = dv > 0.51f ? 2 : dv < 0.49f ? 1 : 0; // hipass · lopass · none
        if (ft) {
          double vv = ft == 2 ? (dv - 0.5f) * 2.0f : dv * 2.0f;
          double cut = vv * 11.0; cut = cut * cut * cut * cut; // (v·11)^4
          double nyq1 = (double)sr_f / 2.0 - 1.0;
          if (cut > nyq1) cut = nyq1;
          double c = 2.0 * sd_sin_d(cut * 3.14159265358979323846 / (double)sr_f);
          if (c < 0) c = 0; if (c > 1.14) c = 1.14;
          const double r = 0.28717458874925877; // 0.5^(8·0.1 + 1), resonance 0.1
          double mrc = 1.0 - r * c;
          float in[2] = { sl, sr2 };
          for (int ch = 0; ch < 2; ch++) { // TwoPoleFilter.update
            o->djf_s0[ch] = mrc * o->djf_s0[ch] - c * o->djf_s1[ch] + c * (double)in[ch];
            o->djf_s1[ch] = mrc * o->djf_s1[ch] + c * o->djf_s0[ch];
            in[ch] = ft == 1 ? (float)o->djf_s1[ch] : in[ch] - (float)o->djf_s1[ch];
          }
          sl = in[0]; sr2 = in[1];
        }
      }
      sl *= g;
      sr2 *= g;
      out_buf[i * OUT_CH] += sl;
      out_buf[i * OUT_CH + 1] += sr2;
      if (stems_on) {
        sb[i * OUT_CH] = sl;
        sb[i * OUT_CH + 1] = sr2;
      }
      o->dry[i * OUT_CH] = 0;
      o->dry[i * OUT_CH + 1] = 0;
      o->vin[i] = 0;
      o->din[i * OUT_CH] = 0;
      o->din[i * OUT_CH + 1] = 0;
    }
  }
  engine_frame += BLOCK;
}

// ---------- OFFLINE-HARNESS HOOKS (pv) — drive the vocoder directly so the
// golden scripts can diff it against the JS phaze reference, block by block --
static Pv *pv_test_state = 0;
static float pv_test_io[BLOCK * 2]; // L at [0..128), R at [128..256); in place

__attribute__((export_name("pv_test_io"))) float *pv_test_io_ptr(void) { return pv_test_io; }

__attribute__((export_name("pv_test_reset"))) void pv_test_reset(void) {
  pv_tables_init();
  if (pv_test_state) pv_release(pv_test_state);
  pv_test_state = pv_alloc();
}

__attribute__((export_name("pv_test_block"))) void pv_test_block(float stretch) {
  if (!pv_test_state) return;
  pv_dbg_capture = true;
  pv_process(pv_test_state, pv_test_io, pv_test_io + BLOCK, pv_test_io, pv_test_io + BLOCK, stretch, true);
}

__attribute__((export_name("pv_test_npeaks"))) int pv_test_npeaks(void) { return pv_dbg_npeaks; }
__attribute__((export_name("pv_test_peaks"))) int *pv_test_peaks(void) { return pv_peaks; }

__attribute__((export_name("pv_test_spec_re"))) double *pv_test_spec_re(void) { return pv_re; }
__attribute__((export_name("pv_test_spec_im"))) double *pv_test_spec_im(void) { return pv_im; }
__attribute__((export_name("pv_test_shift_re"))) double *pv_test_shift_re(void) { return pv_dbg_sre; }
__attribute__((export_name("pv_test_shift_im"))) double *pv_test_shift_im(void) { return pv_dbg_sim; }

__attribute__((export_name("sd_active_voices"))) int sd_active_voices(void) {
  int n = 0;
  for (int i = 0; i < MAX_VOICES; i++)
    if (voices[i].active) n++;
  return n;
}
