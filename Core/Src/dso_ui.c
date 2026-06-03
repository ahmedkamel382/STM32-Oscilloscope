/**
 * ============================================================================
 *  dso_ui.c  --  Menu / Analysis / Memory UI layer.  Sits on top of the DSO
 *  engine and the ST7789 driver; calls the engine's own functions unchanged
 *  while in Live Scope mode, so nothing about sampling or the button actions
 *  changes.
 * ============================================================================
 */
#include "dso_ui.h"
#include "dso_ai.h"
#include "st7789.h"
#include <string.h>
#include <stdio.h>

/* ---- layout (mirrors the constants used inside dso.c) ----------------- */
#define UI_GRAPH_X   8
#define UI_GRAPH_Y   20
#define UI_GRAPH_W   300
#define UI_GRAPH_H   120
#define HDR_H        18
#define FOOT_Y       160
#define FOOT_H       12

/* ---- button -> nav mapping (active-low, separate debounce from dso.c) -- */
enum { NAV_LEFT = 0, NAV_RIGHT, NAV_OK, NAV_CANCEL };

static const struct { GPIO_TypeDef *port; uint16_t pin; } s_nav[4] = {
    { BTN_TIMEUP_GPIO_Port, BTN_TIMEUP_Pin },   /*  <      (PB12) */
    { BTN_TIMEDN_GPIO_Port, BTN_TIMEDN_Pin },   /*  >      (PB13) */
    { BTN_VOLT_GPIO_Port,   BTN_VOLT_Pin   },   /*  OK     (PB14) */
    { BTN_HOLD_GPIO_Port,   BTN_HOLD_Pin   },   /*  X/back (PB15) */
};
static uint8_t  s_nv_state[4] = { 1, 1, 1, 1 };
static uint32_t s_nv_t[4]     = { 0, 0, 0, 0 };

static int nav_edge(int i) {
    uint8_t now = HAL_GPIO_ReadPin(s_nav[i].port, s_nav[i].pin) ? 1 : 0;
    uint32_t t  = HAL_GetTick();
    int edge = 0;
    if (now != s_nv_state[i] && (t - s_nv_t[i]) > 30) {
        s_nv_state[i] = now;
        s_nv_t[i] = t;
        if (now == 0) edge = 1;          /* falling edge = press */
    }
    return edge;
}

/* Wait (briefly) until all buttons are released, so a press that selected the
 * current screen is not re-detected by the OTHER button handler after we
 * switch modes.  Bounded so it can never hang. */
static void wait_all_released(void) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 500) {
        int all_up = 1;
        for (int i = 0; i < 4; ++i)
            if (HAL_GPIO_ReadPin(s_nav[i].port, s_nav[i].pin) == 0) all_up = 0;
        if (all_up) break;
        HAL_Delay(5);
    }
    HAL_Delay(20);
    for (int i = 0; i < 4; ++i) s_nv_state[i] = 1;   /* resync nav debounce */
}

/* ---- UI state --------------------------------------------------------- */
typedef enum {
    UI_MENU = 0, UI_SCOPE, UI_ANALYSIS, UI_MEMORY, UI_COMPARE, UI_ABOUT
} ui_mode_t;

static ui_mode_t s_mode;
static int       s_menu_sel;
static int       s_act_sel;        /* analysis action bar selection */
static dso_ai_result_t s_ai;       /* last analysis result          */

/* ---- Signal memory ---------------------------------------------------- */
#define MEM_MAX  6
#define SNAP_N   150               /* decimated waveform points per snapshot */

typedef struct {
    bool             used;
    uint16_t         id;
    uint8_t          tb_idx, vb_idx;
    dso_sig_class_t  cls;
    dso_meas_t       meas;
    int16_t          wave[SNAP_N]; /* probe mV, decimated from the frozen buf */
} snap_t;

static snap_t   s_mem[MEM_MAX];
static uint8_t  s_mem_write;       /* round-robin write pointer */
static uint16_t s_next_id;
static int      s_mem_sel;         /* highlighted visible row   */
static int      s_cmpA = -1, s_cmpB = -1;  /* slots chosen to compare */

static int used_count(void) {
    int c = 0;
    for (int i = 0; i < MEM_MAX; ++i) if (s_mem[i].used) c++;
    return c;
}
static int visible_slot(int vis) {           /* visible row -> storage slot */
    int c = 0;
    for (int i = 0; i < MEM_MAX; ++i)
        if (s_mem[i].used) { if (c == vis) return i; c++; }
    return -1;
}

static const uint16_t *frozen_buf(dso_t *d) {
    return (d->ready_buf == 1) ? (const uint16_t *)d->buf_b
                               : (const uint16_t *)d->buf_a;
}

static uint16_t save_snapshot(dso_t *d) {
    snap_t *s = &s_mem[s_mem_write];
    s->used   = true;
    s->id     = s_next_id++;
    s->tb_idx = d->tb_idx;
    s->vb_idx = d->vb_idx;
    s->cls    = s_ai.cls;
    s->meas   = d->meas;
    const uint16_t *buf = frozen_buf(d);
    for (int k = 0; k < SNAP_N; ++k) {
        int idx = (int)((long)k * SAMPLE_COUNT / SNAP_N);
        s->wave[k] = (int16_t)DSO_AdcToProbeMv(buf[idx]);
    }
    s_mem_write = (s_mem_write + 1) % MEM_MAX;
    return s->id;
}

/* ---- small drawing helpers -------------------------------------------- */
static void ui_header(const char *title) {
    ST7789_FillRect(0, 0, ST7789_WIDTH, HDR_H, COLOR_DARKGRAY);
    ST7789_DrawString(4, 6, title, COLOR_YELLOW, COLOR_DARKGRAY, 1);
}
static void ui_footer(const char *legend) {
    ST7789_FillRect(0, FOOT_Y, ST7789_WIDTH, FOOT_H, COLOR_DARKGRAY);
    ST7789_DrawString(4, FOOT_Y + 2, legend, COLOR_LIGHTGRAY, COLOR_DARKGRAY, 1);
}

/* a horizontal row of selectable buttons; the selected one is highlighted */
static void draw_actionbar(const char *const *items, int n, int sel, int y) {
    ST7789_FillRect(0, y, ST7789_WIDTH, 14, COLOR_BLACK);
    int x = 8;
    for (int i = 0; i < n; ++i) {
        int w = (int)strlen(items[i]) * 6 + 8;
        uint16_t bg = (i == sel) ? COLOR_BLUE : COLOR_DARKGRAY;
        uint16_t fg = (i == sel) ? COLOR_WHITE : COLOR_LIGHTGRAY;
        ST7789_FillRect(x, y, w, 14, bg);
        ST7789_DrawString(x + 4, y + 4, items[i], fg, bg, 1);
        x += w + 6;
    }
}

/* ====================================================================== */
/*  HOME MENU                                                             */
/* ====================================================================== */
static void draw_menu(void) {
    ST7789_FillScreen(COLOR_BLACK);
    ui_header("AAST DSO   -   MAIN MENU");

    static const char *items[3] = { "1  Live Scope", "2  Signal Memory",
                                     "3  About / Help" };
    for (int i = 0; i < 3; ++i) {
        int y = 32 + i * 24;
        if (i == s_menu_sel) {
            ST7789_FillRect(8, y - 3, 304, 20, COLOR_BLUE);
            ST7789_DrawString(16, y, items[i], COLOR_WHITE, COLOR_BLUE, 2);
        } else {
            ST7789_DrawString(16, y, items[i], COLOR_LIGHTGRAY, COLOR_BLACK, 2);
        }
    }
    ST7789_PrintF(8, FOOT_Y - 14, COLOR_GRAY, COLOR_BLACK, 1,
                  "Saved signals: %d / %d", used_count(), MEM_MAX);
    ui_footer("PB12 <   PB13 >   PB14 OK   PB15 X");
}

/* ====================================================================== */
/*  ANALYSIS SCREEN                                                       */
/* ====================================================================== */
static const char *const ANALYSIS_ACTIONS[3] = { "Resume", "Save", "Menu" };

static void draw_analysis(dso_t *d) {
    ST7789_FillScreen(COLOR_BLACK);
    ui_header("SIGNAL ANALYSIS");

    ST7789_DrawString(8, 22, s_ai.name, DSO_AI_ClassColor(s_ai.cls), COLOR_BLACK, 2);
    ST7789_PrintF(232, 24, COLOR_LIGHTGRAY, COLOR_BLACK, 1, "conf %u%%", s_ai.confidence);

    ST7789_DrawString(6, 42, s_ai.why, COLOR_YELLOW, COLOR_BLACK, 1);

    ST7789_DrawString(6, 54, "Suggested actions:", COLOR_CYAN, COLOR_BLACK, 1);
    for (int i = 0; i < s_ai.advice_n; ++i)
        ST7789_DrawString(10, 64 + i * 9, s_ai.advice[i], COLOR_LIGHTGRAY, COLOR_BLACK, 1);

    if (d->meas.freq_hz >= 1000)
        ST7789_PrintF(6, 104, COLOR_GREEN, COLOR_BLACK, 1,
                      "Vpp %ldmV  F %lu.%lukHz  Vrms %ldmV",
                      (long)d->meas.vpp_mv,
                      (unsigned long)(d->meas.freq_hz / 1000),
                      (unsigned long)((d->meas.freq_hz % 1000) / 100),
                      (long)d->meas.vrms_mv);
    else
        ST7789_PrintF(6, 104, COLOR_GREEN, COLOR_BLACK, 1,
                      "Vpp %ldmV  F %luHz  Vrms %ldmV",
                      (long)d->meas.vpp_mv, (unsigned long)d->meas.freq_hz,
                      (long)d->meas.vrms_mv);

    /* denoised view: cleaned peak-to-peak + leftover noise estimate */
    if (s_ai.denoised)
        ST7789_PrintF(6, 116, COLOR_CYAN, COLOR_BLACK, 1,
                      "Clean Vpp %ldmV   noise ~%ldmV",
                      (long)s_ai.vpp_clean_mv, (long)s_ai.noise_mv);

    draw_actionbar(ANALYSIS_ACTIONS, 3, s_act_sel, 132);
    ui_footer("PB12/13 pick   PB14 do   PB15 resume");
}

/* ====================================================================== */
/*  MEMORY BROWSER                                                        */
/* ====================================================================== */
static void draw_memory(void) {
    int used = used_count();
    ST7789_FillScreen(COLOR_BLACK);
    ST7789_FillRect(0, 0, ST7789_WIDTH, HDR_H, COLOR_DARKGRAY);
    ST7789_PrintF(4, 6, COLOR_YELLOW, COLOR_DARKGRAY, 1,
                  "SIGNAL MEMORY   (%d / %d)", used, MEM_MAX);

    if (used == 0) {
        ST7789_DrawString(24, 64, "No snapshots saved yet.", COLOR_LIGHTGRAY, COLOR_BLACK, 1);
        ST7789_DrawString(24, 80, "In scope, press HOLD to freeze,", COLOR_GRAY, COLOR_BLACK, 1);
        ST7789_DrawString(24, 92, "then OK > Save on the analyzer.", COLOR_GRAY, COLOR_BLACK, 1);
        ui_footer("PB15 X  back to menu");
        return;
    }

    for (int r = 0; r < used; ++r) {
        int slot = visible_slot(r);
        snap_t *s = &s_mem[slot];
        int y = 24 + r * 21;
        const char *mark = (slot == s_cmpA) ? "[A] " :
                           (slot == s_cmpB) ? "[B] " : "    ";
        uint16_t fg = (r == s_mem_sel) ? COLOR_WHITE : COLOR_LIGHTGRAY;
        if (r == s_mem_sel) ST7789_FillRect(4, y - 2, 312, 18, COLOR_BLUE);
        uint16_t bg = (r == s_mem_sel) ? COLOR_BLUE : COLOR_BLACK;
        ST7789_PrintF(8, y, fg, bg, 1, "%s#%-2u %-12s Vpp%4ldmV  F%5luHz",
                      mark, s->id, DSO_AI_ClassName(s->cls),
                      (long)s->meas.vpp_mv, (unsigned long)s->meas.freq_hz);
    }

    if (s_cmpA >= 0 && s_cmpB < 0)
        ST7789_DrawString(8, FOOT_Y - 14, "A picked -- choose B to compare",
                          COLOR_ORANGE, COLOR_BLACK, 1);
    ui_footer("PB12/13 move  PB14 pick A/B  PB15 X");
}

/* ====================================================================== */
/*  COMPARE SCREEN                                                        */
/* ====================================================================== */
static uint16_t map_mv_y(int32_t mv, int32_t lo, int32_t hi) {
    if (hi <= lo) hi = lo + 1;
    int32_t y = UI_GRAPH_Y + UI_GRAPH_H - 1
              - (int32_t)((int64_t)(mv - lo) * (UI_GRAPH_H - 1) / (hi - lo));
    if (y < UI_GRAPH_Y) y = UI_GRAPH_Y;
    if (y > UI_GRAPH_Y + UI_GRAPH_H - 1) y = UI_GRAPH_Y + UI_GRAPH_H - 1;
    return (uint16_t)y;
}

static void plot_wave(const int16_t *w, int32_t lo, int32_t hi, uint16_t color) {
    int16_t px = -1, py = -1;
    for (int k = 0; k < SNAP_N; ++k) {
        int x = UI_GRAPH_X + (int)((long)k * UI_GRAPH_W / SNAP_N);
        int y = map_mv_y(w[k], lo, hi);
        if (px >= 0) ST7789_DrawLine(px, py, (int16_t)x, (int16_t)y, color);
        px = (int16_t)x; py = (int16_t)y;
    }
}

static void draw_compare(void) {
    snap_t *A = &s_mem[s_cmpA];
    snap_t *B = &s_mem[s_cmpB];

    ST7789_FillScreen(COLOR_BLACK);
    ui_header("COMPARE   A=cyan  B=yellow");
    ST7789_DrawRect(UI_GRAPH_X - 1, UI_GRAPH_Y - 1, UI_GRAPH_W + 2, UI_GRAPH_H + 2,
                    COLOR_LIGHTGRAY);

    /* common autoscale across both waveforms (and include 0) */
    int32_t lo = 0, hi = 0;
    for (int k = 0; k < SNAP_N; ++k) {
        if (A->wave[k] < lo) lo = A->wave[k];
        if (A->wave[k] > hi) hi = A->wave[k];
        if (B->wave[k] < lo) lo = B->wave[k];
        if (B->wave[k] > hi) hi = B->wave[k];
    }
    int32_t pad = (hi - lo) / 10 + 50;
    lo -= pad; hi += pad;

    plot_wave(A->wave, lo, hi, COLOR_CYAN);
    plot_wave(B->wave, lo, hi, COLOR_YELLOW);

    ST7789_PrintF(6, UI_GRAPH_Y + UI_GRAPH_H + 4, COLOR_CYAN, COLOR_BLACK, 1,
                  "A #%-2u %-11s Vpp%4ldmV F%5luHz",
                  A->id, DSO_AI_ClassName(A->cls),
                  (long)A->meas.vpp_mv, (unsigned long)A->meas.freq_hz);
    ST7789_PrintF(6, UI_GRAPH_Y + UI_GRAPH_H + 14, COLOR_YELLOW, COLOR_BLACK, 1,
                  "B #%-2u %-11s Vpp%4ldmV F%5luHz",
                  B->id, DSO_AI_ClassName(B->cls),
                  (long)B->meas.vpp_mv, (unsigned long)B->meas.freq_hz);
    ST7789_PrintF(6, UI_GRAPH_Y + UI_GRAPH_H + 24, COLOR_WHITE, COLOR_BLACK, 1,
                  "d: Vpp%+ldmV  F%+ldHz  Vrms%+ldmV",
                  (long)(B->meas.vpp_mv - A->meas.vpp_mv),
                  (long)((long)B->meas.freq_hz - (long)A->meas.freq_hz),
                  (long)(B->meas.vrms_mv - A->meas.vrms_mv));
    ui_footer("PB15 X  back to memory");
}

/* ====================================================================== */
/*  ABOUT / HELP                                                          */
/* ====================================================================== */
static void draw_about(void) {
    ST7789_FillScreen(COLOR_BLACK);
    ui_header("ABOUT / HELP");
    static const char *lines[] = {
        "AAST DSO - STM32F401 Black Pill",
        "ST7789 172x320, landscape mode",
        "",
        "AI Analyzer = on-device expert",
        "system (not an LLM). Freeze a",
        "wave with HOLD and it tells you",
        "what it is, why, and what to do.",
        "",
        "In Live Scope the 4 buttons keep",
        "their jobs: PB12/13 time, PB14",
        "V/div, PB15 HOLD = freeze+analyze.",
    };
    int n = (int)(sizeof(lines) / sizeof(lines[0]));
    for (int i = 0; i < n; ++i)
        ST7789_DrawString(6, 24 + i * 11, lines[i], COLOR_LIGHTGRAY, COLOR_BLACK, 1);
    ui_footer("PB15 X  back to menu");
}

/* ====================================================================== */
/*  Mode switching                                                        */
/* ====================================================================== */
static void switch_mode(dso_t *d, ui_mode_t m) {
    wait_all_released();
    s_mode = m;
    switch (m) {
        case UI_MENU:     draw_menu();        break;
        case UI_SCOPE:    DSO_RedrawStaticUI(d); break;
        case UI_ANALYSIS: s_act_sel = 0; draw_analysis(d); break;
        case UI_MEMORY:   draw_memory();      break;
        case UI_COMPARE:  draw_compare();     break;
        case UI_ABOUT:    draw_about();       break;
    }
}

/* ====================================================================== */
/*  Per-mode tasks                                                        */
/* ====================================================================== */
static void menu_task(dso_t *d) {
    if (nav_edge(NAV_LEFT))  { s_menu_sel = (s_menu_sel + 2) % 3; draw_menu(); }
    if (nav_edge(NAV_RIGHT)) { s_menu_sel = (s_menu_sel + 1) % 3; draw_menu(); }
    if (nav_edge(NAV_OK)) {
        switch (s_menu_sel) {
            case 0: d->hold = false; switch_mode(d, UI_SCOPE); break;
            case 1: s_mem_sel = 0; s_cmpA = s_cmpB = -1; switch_mode(d, UI_MEMORY); break;
            case 2: switch_mode(d, UI_ABOUT); break;
        }
    }
}

static void analysis_task(dso_t *d) {
    if (nav_edge(NAV_LEFT))  { s_act_sel = (s_act_sel + 2) % 3; draw_actionbar(ANALYSIS_ACTIONS, 3, s_act_sel, 132); }
    if (nav_edge(NAV_RIGHT)) { s_act_sel = (s_act_sel + 1) % 3; draw_actionbar(ANALYSIS_ACTIONS, 3, s_act_sel, 132); }
    if (nav_edge(NAV_OK)) {
        if (s_act_sel == 0) {                 /* Resume */
            d->hold = false; switch_mode(d, UI_SCOPE);
        } else if (s_act_sel == 1) {          /* Save   */
            uint16_t id = save_snapshot(d);
            ST7789_FillRect(0, 120, ST7789_WIDTH, 10, COLOR_BLACK);
            ST7789_PrintF(6, 120, COLOR_GREEN, COLOR_BLACK, 1,
                          "Saved as #%u  (%d/%d used)", id, used_count(), MEM_MAX);
        } else {                              /* Menu   */
            d->hold = false; switch_mode(d, UI_MENU);
        }
    }
    if (nav_edge(NAV_CANCEL)) { d->hold = false; switch_mode(d, UI_SCOPE); }
}

static void memory_task(dso_t *d) {
    int used = used_count();
    if (used > 0) {
        if (nav_edge(NAV_LEFT))  { s_mem_sel = (s_mem_sel - 1 + used) % used; draw_memory(); }
        if (nav_edge(NAV_RIGHT)) { s_mem_sel = (s_mem_sel + 1) % used; draw_memory(); }
        if (nav_edge(NAV_OK)) {
            int slot = visible_slot(s_mem_sel);
            if (s_cmpA < 0)                       s_cmpA = slot;
            else if (s_cmpB < 0 && slot != s_cmpA){ s_cmpB = slot; switch_mode(d, UI_COMPARE); return; }
            else                                  { s_cmpA = slot; s_cmpB = -1; }
            draw_memory();
        }
    } else {
        (void)nav_edge(NAV_LEFT); (void)nav_edge(NAV_RIGHT); (void)nav_edge(NAV_OK);
    }
    if (nav_edge(NAV_CANCEL)) { s_cmpA = s_cmpB = -1; switch_mode(d, UI_MENU); }
}

static void compare_task(dso_t *d) {
    if (nav_edge(NAV_CANCEL)) switch_mode(d, UI_MEMORY);
}

static void about_task(dso_t *d) {
    if (nav_edge(NAV_CANCEL)) switch_mode(d, UI_MENU);
}

/* ====================================================================== */
/*  Public entry points                                                   */
/* ====================================================================== */
void DSO_UI_Init(dso_t *d) {
    (void)d;
    memset(s_mem, 0, sizeof(s_mem));
    s_mem_write = 0;
    s_next_id   = 1;
    s_menu_sel  = 0;
    s_mode      = UI_MENU;
    draw_menu();
}

void DSO_UI_Task(dso_t *d) {
    switch (s_mode) {
        case UI_SCOPE:
            /* Live reading: hand the buttons straight back to the engine,
             * exactly as the original firmware did. */
            DSO_PollButtons(d);
            DSO_ProcessFrame(d);
            /* HOLD pressed => we just "stopped reading" => analyze the frame. */
            if (d->hold) {
                DSO_AI_Analyze(d, frozen_buf(d), &s_ai);
                switch_mode(d, UI_ANALYSIS);
            }
            break;
        case UI_MENU:     menu_task(d);     break;
        case UI_ANALYSIS: analysis_task(d); break;
        case UI_MEMORY:   memory_task(d);   break;
        case UI_COMPARE:  compare_task(d);  break;
        case UI_ABOUT:    about_task(d);    break;
    }
}
