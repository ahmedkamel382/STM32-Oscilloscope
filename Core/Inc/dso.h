/**
 * ============================================================================
 *  dso.h  --  DSO engine: ping-pong DMA buffers, measurements, renderer & UI.
 *
 *  Sample rate / time base
 *  -----------------------
 *  TIM2 is clocked from APB1 timer clock (84 MHz on F401 @ 84 MHz core).
 *  TIM2->ARR is reprogrammed when the user changes the time base.
 *
 *  Buffers
 *  -------
 *  Two SAMPLE_COUNT-element uint16_t buffers.  DMA runs in circular mode and
 *  on half/full-complete the engine snapshots into buf_b for the renderer.
 *
 *  Front-end calibration (matches the AS-BUILT hardware on your breadboard)
 *  ----------------------------------------------------------------------
 *  Your front-end on the breadboard is:
 *
 *      Probe (Row 20) --[33k]-- Row 25 --[10k]-- Row 32 (VGND = 1.65 V)
 *                                  |
 *                                  +--[47R]-- Row 15 --[4.7nF]-- GND
 *                                                 |
 *                                                 +-- PA0 (ADC)
 *
 *  So Row 25 (and Row 15 / PA0 in steady state) is a divider:
 *
 *      V_25 = (V_probe * 10k + V_GND * 33k) / (10k + 33k)
 *           = (V_probe * 10  + 1.65 * 33 ) / 43
 *           = 0.2326 * V_probe + 1.266     (in volts)
 *
 *  Inverting it for the firmware:
 *
 *      V_probe = (V_adc - 1266 mV) * 43 / 10        (in millivolts)
 *
 *  Useful probe range:  about -5 V to +6 V at the BNC tip.
 *  At V_probe =  0 V  ->  V_adc =  1.266 V   (ADC count ~1571)
 *  At V_probe = +3 V  ->  V_adc =  1.964 V   (ADC count ~2437)
 *  At V_probe = -3 V  ->  V_adc =  0.568 V   (ADC count ~ 705)
 *
 *  These three constants are the heart of the volt-meter.  If your real
 *  resistors are off (resistor tolerance is +/-5%) you can calibrate the
 *  offset empirically:
 *      1) short the BNC tip to BNC shell.
 *      2) read Vavg on screen (set FE_OFFSET_MV = 0, FE_GAIN_NUM/DEN = 1/1
 *         first, then re-flash).  That Vavg in mV is your real FE_OFFSET_MV.
 *      3) apply a known +3.300 V to the probe; tune FE_GAIN_NUM up/down by 1
 *         until the screen reading matches +3300 mV.
 * ============================================================================
 */
#ifndef DSO_H
#define DSO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Sampling --------------------------------------------------------- */
#define SAMPLE_COUNT        320     /* one full screen worth */
#define ADC_MAX             4095u   /* 12-bit */
#define VREF_MV             3300    /* 3.3 V reference */

/* ---- Front-end calibration constants --------------------------------- */
/*   V_probe_mv = (V_adc_mv - FE_OFFSET_MV) * FE_GAIN_NUM / FE_GAIN_DEN     */
#define FE_HW_CONFIG 'C'

#if FE_HW_CONFIG == 'A'
    #define FE_OFFSET_MV    1266
    #define FE_GAIN_NUM     43
    #define FE_GAIN_DEN     10
#elif FE_HW_CONFIG == 'B'
    #define FE_OFFSET_MV    1241
    #define FE_GAIN_NUM     133
    #define FE_GAIN_DEN     33
#elif FE_HW_CONFIG == 'C'
    #define FE_OFFSET_MV    825
    #define FE_GAIN_NUM     20
    #define FE_GAIN_DEN     10
#endif

/* ---- Time-base presets (microseconds per division, 10 div across) ----
 * Constraint at 84-cycle sample time: max ADC rate = 42MHz/(84+12) =
 * ~437 ksps. With 32 samples per division we need at LEAST 75 us/div,
 * so the table starts at 100 us/div (safe with margin).
 * Sample rate Fs = 84e6 / ((PSC+1) * (ARR+1))
 * us_per_div   = (SAMPLE_COUNT/10) / Fs * 1e6
 * --------------------------------------------------------------------- */
typedef struct {
    const char *label;
    uint32_t    timer_arr;   /* TIM2->ARR value */
    uint32_t    timer_psc;   /* TIM2->PSC value */
    uint32_t    us_per_div;
} dso_timebase_t;

/* ---- Volt-base presets (millivolts per division) --------------------- */
typedef struct {
    const char *label;
    int32_t     mv_per_div;
} dso_voltbase_t;

/* ---- Trigger modes --------------------------------------------------- */
typedef enum { TRIG_AUTO = 0, TRIG_NORMAL, TRIG_SINGLE } dso_trig_mode_t;
typedef enum { EDGE_RISING = 0, EDGE_FALLING }           dso_trig_edge_t;

/* ---- Measurement results --------------------------------------------- */
typedef struct {
    int32_t  vmax_mv;
    int32_t  vmin_mv;
    int32_t  vpp_mv;
    int32_t  vavg_mv;
    int32_t  vrms_full_mv;
    int32_t  vrms_mv;
    uint32_t freq_hz;        /* 0 if cannot detect */
    uint32_t period_us;
    float    duty_pct;
    bool     aliased;        /* set when signal exceeds Nyquist/4 */
} dso_meas_t;

/* ---- Engine state ---------------------------------------------------- */
typedef struct {
    /* hardware handles */
    ADC_HandleTypeDef  *hadc;
    TIM_HandleTypeDef  *htim;
    SPI_HandleTypeDef  *hspi;

    /* ping-pong */
    volatile uint16_t   buf_a[SAMPLE_COUNT];
    volatile uint16_t   buf_b[SAMPLE_COUNT];
    volatile uint8_t    ready_buf;       /* which buffer is ready to draw */
    volatile bool       new_frame;
    volatile uint16_t  *cur_dma;         /* buffer DMA is currently filling */

    /* user-controlled state */
    uint8_t             tb_idx;          /* time-base preset index */
    uint8_t             vb_idx;          /* volt-base preset index */
    bool                hold;            /* freeze waveform */
    bool                run;             /* sampling active */
    dso_trig_mode_t     trig_mode;
    dso_trig_edge_t     trig_edge;
    int32_t             trig_level_mv;   /* trigger level in millivolts */

    /* derived */
    dso_meas_t          meas;
} dso_t;

/* ---- Public tables --------------------------------------------------- */
extern const dso_timebase_t  DSO_Timebases[];
extern const uint8_t         DSO_TimebaseCount;
extern const dso_voltbase_t  DSO_Voltbases[];
extern const uint8_t         DSO_VoltbaseCount;

/* ---- Public API ------------------------------------------------------ */
void DSO_Init(dso_t *dso, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim,
              SPI_HandleTypeDef *hspi);
void DSO_Start(dso_t *dso);
void DSO_Stop(dso_t *dso);

void DSO_OnDMA_HalfComplete(dso_t *dso);
void DSO_OnDMA_FullComplete(dso_t *dso);

void DSO_PollButtons(dso_t *dso);
void DSO_ProcessFrame(dso_t *dso);   /* call from the main loop */

void DSO_RedrawStaticUI(dso_t *dso);
void DSO_DrawWaveform(dso_t *dso, const uint16_t *buf);
void DSO_DrawMeasurements(dso_t *dso);

/* helpers */
int32_t  DSO_AdcToMv(uint16_t adc);
int32_t  DSO_AdcToProbeMv(uint16_t adc);
uint16_t DSO_ProbeMvToY(int32_t mv, int32_t mv_per_div);

#endif /* DSO_H */
