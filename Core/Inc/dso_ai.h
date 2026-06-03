/**
 * ============================================================================
 *  dso_ai.h  --  On-device "AI" Signal Analyzer for the AAST DSO.
 *
 *  WHAT THIS IS (read me!)
 *  -----------------------
 *  An STM32F401 cannot host a real neural network / LLM, and it does not need
 *  to.  This module is a *deterministic expert system*: it looks at the frozen
 *  sample buffer + the measurements the DSO engine already computed, extracts a
 *  few cheap DSP features (crest factor, bimodality, rail-hits, samples/cycle),
 *  and from those it:
 *      1) classifies the waveform   (sine / square / triangle / noise / ...)
 *      2) explains WHY it looks like that
 *      3) tells the user WHAT TO DO (referencing the real controls & front-end)
 *
 *  It reads only.  It NEVER changes how the DSO samples or measures anything.
 *
 *  (If you later want a genuine LLM opinion: the firmware already streams every
 *   frame over USB-CDC in main.c -- a PC-side Python script can forward that to
 *   any model and print the answer.  This module is the offline, on-screen one.)
 * ============================================================================
 */
#ifndef DSO_AI_H
#define DSO_AI_H

#include "dso.h"

/* Waveform families the analyzer can recognise. */
typedef enum {
    SIG_NONE = 0,   /* flat line / nothing connected            */
    SIG_DC,         /* steady DC level, no AC component          */
    SIG_SINE,       /* sinusoid                                  */
    SIG_SQUARE,     /* square / rectangular / PWM pulse          */
    SIG_TRIANGLE,   /* triangle / sawtooth                       */
    SIG_NOISE,      /* random / no stable period                 */
    SIG_CLIPPED,    /* hitting the ADC rail (front-end saturated)*/
    SIG_ALIASED,    /* undersampled -- reading is unreliable     */
    SIG_UNKNOWN
} dso_sig_class_t;

#define AI_MAX_LINES   4    /* max advice lines                  */
#define AI_LINE_LEN   40    /* chars per line (fits 320px @ x1)  */

typedef struct {
    dso_sig_class_t cls;
    const char     *name;             /* short label e.g. "SINE WAVE"   */
    uint8_t         confidence;       /* 0..100                          */
    int32_t         crest_x100;       /* crest factor * 100 (no float printf) */
    int32_t         duty_pct;         /* square duty %, else -1          */
    int32_t         cycles_x10;       /* cycles visible on screen * 10   */
    /* --- denoising front-end (the analyzer cleans a PRIVATE copy of the
     *     buffer before measuring, so jitter like 4.3/4.4V spikes don't
     *     inflate Vpp or fool the crest-factor classifier). Read-only on
     *     the real buffer; the live display + DSO measurements are untouched. */
    int32_t         vpp_clean_mv;     /* denoised peak-to-peak, probe mV */
    int32_t         noise_mv;         /* est. noise removed (mean-abs), mV */
    uint8_t         denoised;         /* 1 = cleaning was applied        */
    char            why[AI_LINE_LEN + 1];                 /* one-line cause */
    char            advice[AI_MAX_LINES][AI_LINE_LEN + 1];/* what to do     */
    uint8_t         advice_n;
} dso_ai_result_t;

/* Analyse the frozen ADC buffer (raw 12-bit counts) + the engine's meas.   */
void        DSO_AI_Analyze(const dso_t *d, const uint16_t *buf, dso_ai_result_t *out);
const char *DSO_AI_ClassName(dso_sig_class_t c);
/* A colour (RGB565) suggestion for drawing the class name on screen.       */
uint16_t    DSO_AI_ClassColor(dso_sig_class_t c);

#endif /* DSO_AI_H */
