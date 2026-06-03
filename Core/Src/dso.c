/**
 * ============================================================================
 * dso.c  --  DSO engine implementation.
 * Includes DSP Parabolic Interpolation and Peak-Hold for high frequencies.
 * ============================================================================
 */
#include "dso.h"
#include "st7789.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/* ============================================================================
 * Time-base table.  Sample rate Fs = 84_000_000 / ((PSC+1) * (ARR+1))
 * us_per_div = (SAMPLE_COUNT/10) / Fs * 1e6 = 32 / Fs * 1e6
 * ============================================================================
 */
const dso_timebase_t DSO_Timebases[] = {
	{  "12.5us/d",  41.5,       0,       12.5 },  /* Msps  */
    {  "25us/d",       83,       0,       25 },  /* 1.27 Msps  */
    {  "50us/d",      130,       0,       50 },  /* 641 ksps   */
    { "100us/d",      261,       0,      100 },  /* 320 ksps   */
    { "200us/d",      524,       0,      200 },  /* 160 ksps   */
    { "500us/d",     1311,       0,      500 },  /* 64 ksps   */
    {   "1ms/d",     2624,       0,     1000 },  /* 32 ksps   */
    {   "2ms/d",     5249,       0,     2000 },  /* 16 ksps   */
    {   "5ms/d",    13124,       0,     5000 },  /* 6.4 ksps  */
    {  "10ms/d",    26249,       0,    10000 },  /* 3.2 ksps  */
    {  "20ms/d",    52499,       0,    20000 },  /* 1.6 ksps  */
    {  "50ms/d",    16405,       7,    50000 },  /* 640  sps  */
    { "100ms/d",    32811,       7,   100000 },  /* 320  sps  */
    { "200ms/d",    65624,       7,   200000 },  /* 160  sps  */
};
const uint8_t DSO_TimebaseCount = sizeof(DSO_Timebases) / sizeof(DSO_Timebases[0]);

const dso_voltbase_t DSO_Voltbases[] = {
    { "100mV/d",  100 },
    { "200mV/d",  200 },
    { "500mV/d",  500 },
    {   "1V/d", 1000 },
    {   "2V/d", 2000 },
    {   "5V/d", 5000 },
};
const uint8_t DSO_VoltbaseCount = sizeof(DSO_Voltbases) / sizeof(DSO_Voltbases[0]);

#define GRAPH_X         8
#define GRAPH_Y         20
#define GRAPH_W         300
#define GRAPH_H         120
#define DIVS_X          10
#define DIVS_Y          8
#define HEADER_H        18
#define FOOTER_Y        (GRAPH_Y + GRAPH_H + 2)

/* ============================================================================
 * DSP Peak-Hold State Variables
 * ============================================================================
 */
static uint16_t s_env_max = 0;
static uint16_t s_env_min = 4095;
static uint32_t s_alias_latch_time = 0;
static bool     s_is_aliased = false;

static void reset_peak_hold(void) {
    s_env_max = 0;
    s_env_min = 4095;
    s_is_aliased = false;
}

/* ============================================================================
 * Calibration helpers
 * ============================================================================
 */
int32_t DSO_AdcToMv(uint16_t adc) {
    return (int32_t)((uint32_t)adc * VREF_MV / ADC_MAX);
}

int32_t DSO_AdcToProbeMv(uint16_t adc) {
    int32_t vadc = DSO_AdcToMv(adc);
    return (vadc - FE_OFFSET_MV) * FE_GAIN_NUM / FE_GAIN_DEN;
}

uint16_t DSO_ProbeMvToY(int32_t mv, int32_t mv_per_div) {
    int32_t centre = GRAPH_Y + GRAPH_H / 2;
    int32_t pxdiv = GRAPH_H / DIVS_Y;
    int32_t y = centre - (mv * pxdiv) / mv_per_div;
    if (y < GRAPH_Y) y = GRAPH_Y;
    if (y > GRAPH_Y + GRAPH_H - 1) y = GRAPH_Y + GRAPH_H - 1;
    return (uint16_t)y;
}

static void apply_timebase(dso_t *d) {
    const dso_timebase_t *tb = &DSO_Timebases[d->tb_idx];
    HAL_TIM_Base_Stop(d->htim);
    __HAL_TIM_SET_PRESCALER(d->htim, tb->timer_psc);
    __HAL_TIM_SET_AUTORELOAD(d->htim, tb->timer_arr);
    __HAL_TIM_SET_COUNTER(d->htim, 0);
    HAL_TIM_Base_Start(d->htim);
}

void DSO_Init(dso_t *d, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim, SPI_HandleTypeDef *hspi) {
    memset(d, 0, sizeof(*d));
    d->hadc = hadc;
    d->htim = htim;
    d->hspi = hspi;
    d->tb_idx = 2;
    d->vb_idx = 3;
    d->run = true;
    d->trig_mode = TRIG_AUTO;
    d->trig_edge = EDGE_RISING;
    d->trig_level_mv = 0;
    d->cur_dma = d->buf_a;
    reset_peak_hold();
}

void DSO_Start(dso_t *d) {
    apply_timebase(d);
    HAL_ADC_Start_DMA(d->hadc, (uint32_t *)d->buf_a, SAMPLE_COUNT);
    d->run = true;
}

void DSO_Stop(dso_t *d) {
    HAL_ADC_Stop_DMA(d->hadc);
    d->run = false;
}

void DSO_OnDMA_HalfComplete(dso_t *d) {
    if (d->hold) return;
    memcpy((void *)d->buf_b, (const void *)d->buf_a, (SAMPLE_COUNT / 2) * sizeof(uint16_t));
}

void DSO_OnDMA_FullComplete(dso_t *d) {
    if (d->hold) return;
    memcpy((void *)&d->buf_b[SAMPLE_COUNT / 2], (const void *)&d->buf_a[SAMPLE_COUNT / 2], (SAMPLE_COUNT / 2) * sizeof(uint16_t));
    d->ready_buf = 1;
    d->new_frame = true;
}

/* ============================================================================
 * Buttons
 * ============================================================================
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       state;
    uint32_t      t;
} btn_t;

static btn_t s_btn[4] = {
    { BTN_TIMEUP_GPIO_Port, BTN_TIMEUP_Pin, 1, 0 },
    { BTN_TIMEDN_GPIO_Port, BTN_TIMEDN_Pin, 1, 0 },
    { BTN_VOLT_GPIO_Port,   BTN_VOLT_Pin,   1, 0 },
    { BTN_HOLD_GPIO_Port,   BTN_HOLD_Pin,   1, 0 },
};

static int btn_pressed(int i) {
    uint8_t now = HAL_GPIO_ReadPin(s_btn[i].port, s_btn[i].pin) ? 1 : 0;
    uint32_t t  = HAL_GetTick();
    int edge = 0;
    if (now != s_btn[i].state && (t - s_btn[i].t) > 30) {
        s_btn[i].state = now;
        s_btn[i].t = t;
        if (now == 0) edge = 1;
    }
    return edge;
}

void DSO_PollButtons(dso_t *d) {
    if (btn_pressed(0)) {
        if (d->tb_idx > 0) {
            d->tb_idx--;
            apply_timebase(d);
            reset_peak_hold();
            DSO_RedrawStaticUI(d);
        }
    }
    if (btn_pressed(1)) {
        if (d->tb_idx < DSO_TimebaseCount - 1) {
            d->tb_idx++;
            apply_timebase(d);
            reset_peak_hold();
            DSO_RedrawStaticUI(d);
        }
    }
    if (btn_pressed(2)) {
        d->vb_idx = (d->vb_idx + 1) % DSO_VoltbaseCount;
        reset_peak_hold();
        DSO_RedrawStaticUI(d);
    }
    if (btn_pressed(3)) {
        d->hold = !d->hold;
        reset_peak_hold();
        DSO_RedrawStaticUI(d);
    }
}

/* ============================================================================
 * DSP Measurements (Parabolic Interpolation + Peak-Hold + Smart Alias)
 * ============================================================================
 */
static void measure(dso_t *d, const uint16_t *buf) {
    uint16_t mn = 0xFFFF, mx = 0;
    int mn_idx = 0, mx_idx = 0;
    uint32_t sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint16_t s = buf[i];
        if (s > mx) { mx = s; mx_idx = i; }
        if (s < mn) { mn = s; mn_idx = i; }
        sum += s;
    }

    // --- 1. Parabolic Interpolation (Finds the true peak between dots) ---
    float interp_max = mx;
    float interp_min = mn;

    if (mx_idx > 0 && mx_idx < SAMPLE_COUNT - 1) {
        float y1 = buf[mx_idx - 1];
        float y2 = buf[mx_idx];
        float y3 = buf[mx_idx + 1];
        float denom = y1 - 2*y2 + y3;
        if (denom != 0) {
            float dx = 0.5f * (y1 - y3) / denom;
            interp_max = y2 - 0.25f * (y1 - y3) * dx;
        }
    }

    if (mn_idx > 0 && mn_idx < SAMPLE_COUNT - 1) {
        float y1 = buf[mn_idx - 1];
        float y2 = buf[mn_idx];
        float y3 = buf[mn_idx + 1];
        float denom = y1 - 2*y2 + y3;
        if (denom != 0) {
            float dx = 0.5f * (y1 - y3) / denom;
            interp_min = y2 - 0.25f * (y1 - y3) * dx;
        }
    }

    if (interp_max > ADC_MAX) interp_max = ADC_MAX;
    if (interp_min < 0) interp_min = 0;

    // --- 2. Outer Envelope Peak-Hold (Smooths high frequency jitter) ---
    if ((uint16_t)interp_max > s_env_max) {
        s_env_max = (uint16_t)interp_max; // Fast Attack
    } else {
        s_env_max = (uint16_t)((s_env_max * 31 + (uint32_t)interp_max) / 32); // Slow Decay
    }

    if ((uint16_t)interp_min < s_env_min) {
        s_env_min = (uint16_t)interp_min; // Fast Attack
    } else {
        s_env_min = (uint16_t)((s_env_min * 31 + (uint32_t)interp_min) / 32); // Slow Decay
    }

    // Map the held envelope to probe voltage
    d->meas.vmax_mv  = DSO_AdcToProbeMv(s_env_max);
    d->meas.vmin_mv  = DSO_AdcToProbeMv(s_env_min);
    d->meas.vpp_mv   = d->meas.vmax_mv - d->meas.vmin_mv;

    uint32_t mean = sum / SAMPLE_COUNT;
    d->meas.vavg_mv  = DSO_AdcToProbeMv((uint16_t)mean);

    // RMS AC Calculation
    uint64_t ssq_ac = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        int32_t dx_val = (int32_t)buf[i] - (int32_t)mean;
        ssq_ac += (uint64_t)(dx_val * dx_val);
    }
    float rms_ac = sqrtf((float)(ssq_ac / SAMPLE_COUNT));
    d->meas.vrms_mv = (int32_t)(rms_ac * VREF_MV / ADC_MAX) * FE_GAIN_NUM / FE_GAIN_DEN;

    // --- 3. Frequency & Duty Cycle ---
    uint32_t hyst = ((uint32_t)(mx - mn) * 2) / 100;
    if (hyst < 4) hyst = 4;
    int last_cross = -1;
    int crossings  = 0;
    int first_cross = -1;
    int high = (buf[0] > mean);

    for (int i = 1; i < SAMPLE_COUNT; ++i) {
        if (!high && buf[i] > (uint16_t)(mean + hyst)) {
            high = 1;
            if (first_cross < 0) first_cross = i;
            last_cross = i;
            crossings++;
        } else if (high && buf[i] < (uint16_t)(mean - hyst)) {
            high = 0;
        }
    }

    uint32_t samples_per_period = 0;
    if (crossings >= 2 && last_cross > first_cross) {
        samples_per_period = (uint32_t)(last_cross - first_cross) / (crossings - 1);
        const dso_timebase_t *tb = &DSO_Timebases[d->tb_idx];
        uint64_t sample_ns = (uint64_t)(tb->timer_psc + 1) * (tb->timer_arr + 1) * 1000ULL / 84ULL;
        uint64_t period_ns = (uint64_t)samples_per_period * sample_ns;

        if (period_ns > 0) {
            d->meas.period_us = (uint32_t)(period_ns / 1000ULL);
            d->meas.freq_hz   = (uint32_t)(1000000000ULL / period_ns);
        } else {
            d->meas.period_us = 0;
            d->meas.freq_hz   = 0;
        }
    } else {
        d->meas.freq_hz   = 0;
        d->meas.period_us = 0;
    }

    // --- 4. Smart Aliasing Sentinel ---
    bool aliased_now = false;
    if (crossings >= 2) {
        // Less than 4 samples per period is statistically unstable (Nyquist margin)
        if (samples_per_period < 4) aliased_now = true;
    } else {
        // High energy wave but no countable cycles means it folded entirely
        if ((s_env_max - s_env_min) > 400) aliased_now = true;
    }

    // 500ms latch so the warning doesn't flicker
    if (aliased_now) {
        s_is_aliased = true;
        s_alias_latch_time = HAL_GetTick();
    } else {
        if (HAL_GetTick() - s_alias_latch_time > 500) {
            s_is_aliased = false;
        }
    }
    d->meas.aliased = s_is_aliased;
}

/* ============================================================================
 * Triggering
 * ============================================================================
 */
static int find_trigger(const uint16_t *buf, int n, dso_t *d) {
    if (d->trig_mode == TRIG_AUTO && d->meas.vpp_mv < 50) return 0;
    int32_t trig_vadc_mv = (d->trig_level_mv * FE_GAIN_DEN) / FE_GAIN_NUM + FE_OFFSET_MV;
    int32_t trig_adc = trig_vadc_mv * (int32_t)ADC_MAX / VREF_MV;
    if (trig_adc < 0) trig_adc = 0;
    if (trig_adc > (int32_t)ADC_MAX) trig_adc = (int32_t)ADC_MAX;
    for (int i = 1; i < n - 1; ++i) {
        if (d->trig_edge == EDGE_RISING) {
            if (buf[i-1] < (uint16_t)trig_adc && buf[i] >= (uint16_t)trig_adc) return i;
        } else {
            if (buf[i-1] > (uint16_t)trig_adc && buf[i] <= (uint16_t)trig_adc) return i;
        }
    }
    return 0;
}

/* ============================================================================
 * Static UI & Render
 * ============================================================================
 */
void DSO_RedrawStaticUI(dso_t *d) {
    ST7789_FillScreen(COLOR_BLACK);
    ST7789_FillRect(0, 0, ST7789_WIDTH, HEADER_H, COLOR_DARKGRAY);
    ST7789_DrawString(4, 6, "AAST DSO  STM32F401", COLOR_YELLOW, COLOR_DARKGRAY, 1);
    ST7789_PrintF(140, 6, COLOR_CYAN, COLOR_DARKGRAY, 1,
                  "T:%-7s V:%-7s  ", DSO_Timebases[d->tb_idx].label, DSO_Voltbases[d->vb_idx].label);
    ST7789_DrawString(260, 6, d->hold ? "HOLD" : (d->run ? " RUN" : "STOP"),
                      d->hold ? COLOR_RED : COLOR_GREEN, COLOR_DARKGRAY, 1);

    ST7789_DrawRect(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_W + 2, GRAPH_H + 2, COLOR_LIGHTGRAY);

    int dx = GRAPH_W / DIVS_X;
    int dy = GRAPH_H / DIVS_Y;
    for (int i = 1; i < DIVS_X; ++i) {
        int x = GRAPH_X + i * dx;
        for (int y = GRAPH_Y; y < GRAPH_Y + GRAPH_H; y += 4) ST7789_DrawPixel(x, y, COLOR_GRID);
    }
    for (int j = 1; j < DIVS_Y; ++j) {
        int y = GRAPH_Y + j * dy;
        for (int x = GRAPH_X; x < GRAPH_X + GRAPH_W; x += 4) ST7789_DrawPixel(x, y, COLOR_GRID);
    }

    int cx = GRAPH_X + GRAPH_W / 2, cy = GRAPH_Y + GRAPH_H / 2;
    ST7789_DrawHLine(cx - 3, cy, 7, COLOR_GRAY);
    ST7789_DrawVLine(cx, cy - 3, 7, COLOR_GRAY);

    ST7789_DrawString(4, FOOTER_Y, "PB12-/PB13+ time  PB14:V/div  PB15:HOLD", COLOR_GRAY, COLOR_BLACK, 1);
}

static int16_t s_prev_min_y[GRAPH_W];
static int16_t s_prev_max_y[GRAPH_W];

void DSO_DrawWaveform(dso_t *d, const uint16_t *buf) {
    int trig = find_trigger(buf, SAMPLE_COUNT, d);
    int32_t mvdiv = DSO_Voltbases[d->vb_idx].mv_per_div;
    int16_t prev_y = -1;

    for (int x = 0; x < GRAPH_W; ++x) {
        int idx = trig + (x * SAMPLE_COUNT) / GRAPH_W;
        if (idx >= SAMPLE_COUNT) idx = SAMPLE_COUNT - 1;
        int32_t mv = DSO_AdcToProbeMv(buf[idx]);
        int16_t y  = (int16_t)DSO_ProbeMvToY(mv, mvdiv);

        int16_t curr_min = y;
        int16_t curr_max = y;
        if (prev_y >= 0) {
            if (prev_y < y) curr_min = prev_y;
            else            curr_max = prev_y;
        }

        if (s_prev_min_y[x] != 0 || s_prev_max_y[x] != 0) {
            for (int yy = s_prev_min_y[x]; yy <= s_prev_max_y[x]; ++yy) {
                if (yy >= GRAPH_Y && yy < GRAPH_Y + GRAPH_H) {
                    ST7789_DrawPixel(GRAPH_X + x, yy, COLOR_BLACK);
                    if (((x % (GRAPH_W / DIVS_X)) == 0) || (((yy - GRAPH_Y) % (GRAPH_H / DIVS_Y)) == 0)) {
                        ST7789_DrawPixel(GRAPH_X + x, yy, COLOR_GRID);
                    }
                }
            }
        }

        for (int yy = curr_min; yy <= curr_max; ++yy) {
            ST7789_DrawPixel(GRAPH_X + x, yy, COLOR_GREEN);
        }

        s_prev_min_y[x] = curr_min;
        s_prev_max_y[x] = curr_max;
        prev_y = y;
    }
}

void DSO_DrawMeasurements(dso_t *d) {
    char l1[64], l2[64];
    if (d->meas.aliased) {
        // UI Fix: Instructing the user to DECREASE the time/div setting (go faster)
        snprintf(l1, sizeof l1, "*** ALIASED -- decrease TIME/DIV *** ");
    } else {
        snprintf(l1, sizeof l1, "Vpp:%4ldmV  Vmx:%4ldmV  Vmn:%4ldmV",
                 (long)d->meas.vpp_mv, (long)d->meas.vmax_mv, (long)d->meas.vmin_mv);
    }
    if (d->meas.freq_hz >= 1000)
        snprintf(l2, sizeof l2, "Vavg:%4ldmV  Vrms:%4ldmV  F:%lu.%lukHz",
                 (long)d->meas.vavg_mv, (long)d->meas.vrms_mv,
                 (unsigned long)(d->meas.freq_hz / 1000), (unsigned long)((d->meas.freq_hz % 1000) / 100));
    else
        snprintf(l2, sizeof l2, "Vavg:%4ldmV  Vrms:%4ldmV  F:%4luHz ",
                 (long)d->meas.vavg_mv, (long)d->meas.vrms_mv, (unsigned long)d->meas.freq_hz);

    ST7789_DrawString(4, FOOTER_Y + 10, l1, COLOR_YELLOW, COLOR_BLACK, 1);
    ST7789_DrawString(4, FOOTER_Y + 20, l2, COLOR_CYAN,   COLOR_BLACK, 1);
}

void DSO_ProcessFrame(dso_t *d) {
    if (!d->new_frame) return;
    d->new_frame = false;

    const uint16_t *buf = (d->ready_buf == 1) ? (const uint16_t *)d->buf_b : (const uint16_t *)d->buf_a;
    measure(d, buf);
    if (!d->hold) DSO_DrawWaveform(d, buf);
    DSO_DrawMeasurements(d);

    ST7789_PrintF(140, 6, COLOR_CYAN, COLOR_DARKGRAY, 1, "T:%-7s V:%-7s  ", DSO_Timebases[d->tb_idx].label, DSO_Voltbases[d->vb_idx].label);
    ST7789_DrawString(260, 6, d->hold ? "HOLD" : (d->run ? " RUN" : "STOP"), d->hold ? COLOR_RED : COLOR_GREEN, COLOR_DARKGRAY, 1);
}
