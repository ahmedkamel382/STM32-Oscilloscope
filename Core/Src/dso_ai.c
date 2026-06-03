/**
 * ============================================================================
 *  dso_ai.c  --  Implementation of the on-device Signal Analyzer.
 *  Pure logic: reads the frozen buffer + meas, writes a dso_ai_result_t.
 *  No HAL calls, no globals touched -> safe to run from the main loop.
 * ============================================================================
 */
#include "dso_ai.h"
#include "st7789.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

const char *DSO_AI_ClassName(dso_sig_class_t c) {
    switch (c) {
        case SIG_NONE:     return "NO SIGNAL";
        case SIG_DC:       return "DC LEVEL";
        case SIG_SINE:     return "SINE WAVE";
        case SIG_SQUARE:   return "SQUARE/PULSE";
        case SIG_TRIANGLE: return "TRIANGLE/SAW";
        case SIG_NOISE:    return "NOISE";
        case SIG_CLIPPED:  return "CLIPPING!";
        case SIG_ALIASED:  return "ALIASED!";
        default:           return "UNKNOWN";
    }
}

uint16_t DSO_AI_ClassColor(dso_sig_class_t c) {
    switch (c) {
        case SIG_CLIPPED:
        case SIG_ALIASED:  return COLOR_RED;
        case SIG_NONE:     return COLOR_GRAY;
        case SIG_NOISE:    return COLOR_ORANGE;
        case SIG_DC:       return COLOR_CYAN;
        default:           return COLOR_GREEN;
    }
}

/* small helper: append a printf-style advice line (with truncation safety). */
static void add_advice(dso_ai_result_t *r, const char *fmt, ...) {
    if (r->advice_n >= AI_MAX_LINES) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r->advice[r->advice_n], AI_LINE_LEN + 1, fmt, ap);
    va_end(ap);
    r->advice_n++;
}

/* ========================================================================
 *  DENOISING FRONT-END
 *  --------------------------------------------------------------------
 *  Goal: when the input has jitter / spikes (e.g. a 4.20V line that the
 *  ADC reads as 4.18, 4.31, 4.22, 4.40, ...), the raw min/max is dragged
 *  apart by single bad samples.  That inflates Vpp and corrupts the crest
 *  factor that drives classification.  We clean a PRIVATE COPY of the
 *  buffer so the measurements the analyzer uses are robust, without ever
 *  modifying the real DMA buffer, the live display, or the DSO engine.
 *
 *  Two cheap, edge-preserving stages (run once, only on HOLD):
 *    1) Median-of-5 : removes impulsive spikes up to 2 samples wide.
 *                     Median keeps the flat tops & sharp edges of a square
 *                     wave intact (an averaging filter would round them).
 *    2) 1-2-1 smooth: light low-pass for the leftover small Gaussian hiss.
 * ====================================================================== */
static uint16_t s_clean[SAMPLE_COUNT];   /* private scratch (~640 B, .bss) */

/* median of five 12-bit samples via a tiny sorting network */
static uint16_t med5(uint16_t a, uint16_t b, uint16_t c,
                     uint16_t d, uint16_t e) {
#define SWAP_(x,y) do{ if((x)>(y)){ uint16_t t=(x); (x)=(y); (y)=t; } }while(0)
    SWAP_(a,b); SWAP_(d,e); SWAP_(a,c); SWAP_(b,c);
    SWAP_(a,d); SWAP_(c,d); SWAP_(b,e); SWAP_(b,c); SWAP_(d,e);
#undef SWAP_
    return c;   /* the median ends up in the middle slot */
}

/* clamp-indexed read so edges don't need special cases */
static uint16_t at(const uint16_t *b, int i) {
    if (i < 0) i = 0;
    if (i >= SAMPLE_COUNT) i = SAMPLE_COUNT - 1;
    return b[i];
}

/* Fill s_clean[] with a denoised copy of raw[].  Returns mean-abs noise
 * (in ADC counts) that the cleaning removed -- a handy "how dirty" metric. */
static uint32_t denoise(const uint16_t *raw) {
    /* stage 1: median-of-5 despike into s_clean */
    for (int i = 0; i < SAMPLE_COUNT; ++i)
        s_clean[i] = med5(at(raw, i - 2), at(raw, i - 1), raw[i],
                          at(raw, i + 1), at(raw, i + 2));

    /* stage 2: in-place 1-2-1 smooth using 3 rolling scalars (no 2nd buffer) */
    uint16_t prev = s_clean[0];
    for (int i = 1; i < SAMPLE_COUNT - 1; ++i) {
        uint16_t cur  = s_clean[i];
        uint16_t next = s_clean[i + 1];
        s_clean[i] = (uint16_t)(((uint32_t)prev + 2u * cur + next) >> 2);
        prev = cur;                 /* feed the ORIGINAL cur, not the smoothed */
    }

    /* noise metric: mean |raw - clean| in ADC counts */
    uint32_t acc = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        int32_t diff = (int32_t)raw[i] - (int32_t)s_clean[i];
        acc += (uint32_t)(diff < 0 ? -diff : diff);
    }
    return acc / SAMPLE_COUNT;
}

void DSO_AI_Analyze(const dso_t *d, const uint16_t *buf, dso_ai_result_t *out) {
    memset(out, 0, sizeof(*out));
    out->duty_pct = -1;

    /* ---- 0. Clean a PRIVATE copy first (raw buffer is left untouched) -- */
    uint32_t noise_cnt = denoise(buf);     /* fills s_clean[], returns counts */
    const uint16_t *cln = s_clean;
    out->denoised = 1;

    /* ---- 1. Feature extraction --------------------------------------- *
     * Amplitude / RMS / crest come from the CLEANED copy so a stray spike
     * cannot inflate them.  Rail-hit counting stays on the RAW buffer --
     * clipping is genuine saturated signal, not noise we should smooth.  */
    uint16_t mn = 0xFFFF, mx = 0;
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint16_t s = cln[i];
        if (s > mx) mx = s;
        if (s < mn) mn = s;
        sum += s;
    }
    uint32_t mean = sum / SAMPLE_COUNT;
    int32_t  amp  = (int32_t)mx - (int32_t)mn;   /* peak-to-peak in counts */

    int32_t band = amp / 5;                      /* +/-20% centre band     */
    if (band < 3) band = 3;

    uint64_t ssq = 0;
    uint32_t rail_hi = 0, rail_lo = 0, center = 0, above = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        int32_t dx = (int32_t)cln[i] - (int32_t)mean;
        ssq += (uint64_t)(dx * dx);
        if (buf[i] >= (uint16_t)(ADC_MAX - 2)) rail_hi++;   /* RAW for rails */
        if (buf[i] <= 2)                       rail_lo++;   /* RAW for rails */
        if (dx > -band && dx < band)           center++;
        if (dx > 0)                            above++;
    }
    float rms_ac     = sqrtf((float)(ssq / SAMPLE_COUNT));
    float crest      = (rms_ac > 1.0f) ? ((float)amp * 0.5f) / rms_ac : 0.0f;
    float center_fr  = (float)center / SAMPLE_COUNT;
    float rail_fr    = (float)(rail_hi + rail_lo) / SAMPLE_COUNT;
    out->crest_x100  = (int32_t)(crest * 100.0f + 0.5f);

    /* cleaned peak-to-peak + noise level, in probe millivolts */
    {
        int32_t hi_mv = DSO_AdcToProbeMv(mx);
        int32_t lo_mv = DSO_AdcToProbeMv(mn);
        out->vpp_clean_mv = hi_mv - lo_mv;
        /* convert the count-domain noise into a mV span (gain only, offset
         * cancels): map [0..noise_cnt] through the probe scale.            */
        int32_t z  = DSO_AdcToProbeMv(0);
        int32_t nz = DSO_AdcToProbeMv((uint16_t)(noise_cnt > ADC_MAX ? ADC_MAX
                                                                     : noise_cnt));
        out->noise_mv = nz - z;
        if (out->noise_mv < 0) out->noise_mv = -out->noise_mv;
    }

    /* ---- 2. Probe-domain / screen-fit context ------------------------- */
    int32_t vpp   = d->meas.vpp_mv;                          /* probe mV   */
    int32_t mvdiv = DSO_Voltbases[d->vb_idx].mv_per_div;
    int32_t divs_used_x10 = (mvdiv > 0) ? (vpp * 10 / mvdiv) : 0;

    /* cycles visible = capture_time / period (all in us, integer-safe)    */
    const dso_timebase_t *tb = &DSO_Timebases[d->tb_idx];
    /* sample period (us) = (PSC+1)*(ARR+1)/84  ; capture = *SAMPLE_COUNT  */
    uint64_t cap_us = (uint64_t)(tb->timer_psc + 1) * (tb->timer_arr + 1)
                      * SAMPLE_COUNT / 84ULL;
    if (d->meas.period_us > 0)
        out->cycles_x10 = (int32_t)(cap_us * 10ULL / d->meas.period_us);
    else
        out->cycles_x10 = 0;

    /* ---- 3. Classification (priority order) --------------------------- */
    dso_sig_class_t cls;

    if (vpp < 40 && amp < 25) {
        /* essentially a flat line */
        if (d->meas.vavg_mv > 200 || d->meas.vavg_mv < -200) {
            cls = SIG_DC;
            out->confidence = 90;
            snprintf(out->why, AI_LINE_LEN + 1,
                     "DC level %ldmV, no AC motion.", (long)d->meas.vavg_mv);
            add_advice(out, "Steady DC. For a wave, the input");
            add_advice(out, "must actually change over time.");
        } else {
            cls = SIG_NONE;
            out->confidence = 95;
            snprintf(out->why, AI_LINE_LEN + 1, "Trace sits flat on the baseline.");
            add_advice(out, "Connect the probe to a live node.");
            add_advice(out, "Check probe + 1.65V bias (GND row).");
        }
    }
    else if (rail_fr > 0.04f) {
        cls = SIG_CLIPPED;
        out->confidence = 95;
        snprintf(out->why, AI_LINE_LEN + 1,
                 "Peaks pinned at the ADC rail (%d%%).", (int)(rail_fr * 100));
        add_advice(out, "Input exceeds the +6/-5V window");
        add_advice(out, "or the 1.65V bias has drifted.");
        add_advice(out, "Reduce amplitude; raise mV/div PB14.");
    }
    else if (d->meas.aliased) {
        cls = SIG_ALIASED;
        out->confidence = 90;
        snprintf(out->why, AI_LINE_LEN + 1, "Under 4 samples per cycle.");
        add_advice(out, "Signal too fast for this time base.");
        add_advice(out, "Speed up: press PB12 (less us/div).");
        add_advice(out, "Freq/Vpp may be wrong until fixed.");
    }
    else {
        /* A genuine periodic wave always yields a detected frequency from the
         * engine's zero-cross counter.  If there is none, it is noise -- this
         * is what separates noise from a triangle (both have crest ~1.73). */
        int has_freq = (d->meas.freq_hz > 0);

        if (!has_freq) {
            cls = SIG_NOISE;  out->confidence = 75;
            snprintf(out->why, AI_LINE_LEN + 1, "No stable period detected.");
            add_advice(out, "Looks like noise / no clean signal.");
            add_advice(out, "Check grounding & probe contact.");
        }
        else if (crest < 1.20f && center_fr < 0.18f) {
            cls = SIG_SQUARE; out->confidence = 88;
            out->duty_pct = (int32_t)(100 * above / SAMPLE_COUNT);
            snprintf(out->why, AI_LINE_LEN + 1,
                     "Flat hi/lo levels, duty ~%ld%%.", (long)out->duty_pct);
            add_advice(out, "Square/PWM. Sharp edges may ring;");
            add_advice(out, "that is normal at fast time bases.");
        }
        else if (crest >= 1.20f && crest < 1.55f) {
            cls = SIG_SINE;   out->confidence = 85;
            snprintf(out->why, AI_LINE_LEN + 1,
                     "Smooth rounded peaks; crest ~1.41.");
        }
        else if (crest >= 1.55f && crest < 2.10f) {
            cls = SIG_TRIANGLE; out->confidence = 80;
            snprintf(out->why, AI_LINE_LEN + 1,
                     "Straight ramps; crest ~1.73.");
        }
        else if (crest >= 2.10f) {
            cls = SIG_NOISE;  out->confidence = 70;
            snprintf(out->why, AI_LINE_LEN + 1, "Very spiky, crest %ld.%02ld.",
                     (long)(out->crest_x100 / 100), (long)(out->crest_x100 % 100));
            add_advice(out, "Spiky/noisy. Check source & ground.");
        }
        else {
            cls = SIG_UNKNOWN; out->confidence = 40;
            snprintf(out->why, AI_LINE_LEN + 1, "Mixed shape, hard to classify.");
            add_advice(out, "Adjust time/V-div for a cleaner view.");
        }

        /* ---- generic, real, actionable tuning advice ------------------ */
        if (divs_used_x10 > 0 && divs_used_x10 < 10)
            add_advice(out, "Trace small (%ld.%ld div). Lower mV/div.",
                       (long)(divs_used_x10 / 10), (long)(divs_used_x10 % 10));
        else if (divs_used_x10 > 70)
            add_advice(out, "Trace very tall. Raise mV/div PB14.");

        if (out->cycles_x10 > 0 && out->cycles_x10 < 12)
            add_advice(out, "Under 1 cycle shown. Slow time PB13.");
        else if (out->cycles_x10 > 300)
            add_advice(out, "%ld cycles cramped. Speed time PB12.",
                       (long)(out->cycles_x10 / 10));

        /* noise feedback: vpp_clean_mv is already despiked, so compare the
         * leftover noise against it to judge signal quality.              */
        if (out->vpp_clean_mv > 0 &&
            out->noise_mv * 100 > out->vpp_clean_mv * 8)   /* noise > 8% of Vpp */
            add_advice(out, "Noisy (~%ldmV). Check ground/leads.",
                       (long)out->noise_mv);
    }

    out->cls  = cls;
    out->name = DSO_AI_ClassName(cls);
}
