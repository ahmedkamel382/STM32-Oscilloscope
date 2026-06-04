# AAST DSO — Laptop Companion App

A desktop oscilloscope viewer for the STM32F401 DSO. It reads the device's
existing USB-CDC stream (**no firmware change needed**) and adds a big zoomable
display, cursors, a readings database, and a **real offline AI classifier**.

## Features
- **Live waveform** on a fast PyQtGraph plot. Mouse-drag to pan, scroll to zoom;
  *View → Autoscale* fits the trace.
- **Cursors** (tick the "Cursors" box): two time + two voltage lines; the status
  bar shows Δt and ΔV.
- **0 V offset line** — draggable dashed reference line.
- **Measurements panel** — Vpp / Vmax / Vmin / Vavg / Vrms / Freq / Period,
  straight from the device (so they match the ST7789).
- **Calibration switch** — Config C (current firmware) or Config A (after the
  fix), so the plotted voltage matches the screen.
- **Readings database (SQLite)** — Save a frame, then Overlay one on the live
  trace, Compare two side-by-side, Delete, or Export the list to CSV.
- **AI classifier** — a genuine scikit-learn RandomForest trained offline on
  synthesised sine/square/triangle/noise/DC frames. No internet required.

## Install
```
pip install -r requirements.txt
```

## Run
1. Connect the Black Pill's **own USB-C cable** to the laptop (not the ST-Link).
2. `python dso_app.py`
3. Pick the serial port (press ↻ to rescan) and click **Connect**.

The model `dso_model.joblib` ships ready-trained. To retrain it yourself:
```
python train_model.py
```

## Files
- `dso_app.py` — the GUI application
- `dso_protocol.py` — parses the 736-byte USB packet (96-byte header + 320 samples)
- `dso_features.py` — shared feature extractor (used by app **and** trainer)
- `train_model.py` — synthesises data and trains `dso_model.joblib`
- `dso_model.joblib` — the trained classifier
- `dso_readings.db` — created on first save

## Notes
- If `dso_model.joblib` or scikit-learn is missing, the classifier falls back to
  a crest-factor heuristic so the app still runs.
- A flat trace (<~10 mV motion) is reported as **DC** directly.
- The classifier reaches 100% on sine/square/triangle and ~80% on noise vs DC
  (they overlap at low amplitude) on held-out synthetic data — report it honestly.
