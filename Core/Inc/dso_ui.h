/**
 * ============================================================================
 *  dso_ui.h  --  Enhanced UI layer for the AAST DSO.
 *
 *  Adds, WITHOUT touching the existing engine:
 *    - A boot MENU navigated with the 4 buttons as  <  >  OK  X
 *    - "Live Scope" mode that hands the buttons straight back to the original
 *      DSO_PollButtons / DSO_ProcessFrame (time, V/div, HOLD = unchanged).
 *    - When you HOLD (stop reading) it runs the on-device Signal Analyzer and
 *      shows "what the signal is / why it looks like that / what to do".
 *    - A SIGNAL MEMORY that stores frozen readings so you can COMPARE two of
 *      them (overlaid waveforms + measurement deltas).
 *
 *  Integration (main.c) -- only 3 tiny edits, see the patched main.c:
 *      after DSO_Start(&g_dso):      DSO_UI_Init(&g_dso);
 *      inside the while(1) loop, replace
 *          DSO_PollButtons(&g_dso); DSO_ProcessFrame(&g_dso);
 *      with
 *          DSO_UI_Task(&g_dso);
 *  Everything else (USB streaming, ADC/DMA, measurements) stays identical.
 * ============================================================================
 */
#ifndef DSO_UI_H
#define DSO_UI_H

#include "dso.h"

/* Call once, after DSO_Init + DSO_Start.  Shows the home menu. */
void DSO_UI_Init(dso_t *d);

/* Call every iteration of the main loop (replaces the two direct calls). */
void DSO_UI_Task(dso_t *d);

#endif /* DSO_UI_H */
