#!/usr/bin/env python3
"""
dso_app.py  --  AAST DSO  |  Laptop companion application.

Reads the STM32 USB-CDC stream (no firmware change), shows the live waveform on a
big zoomable plot, with draggable cursors and a 0 V offset line, a measurements
panel, a SQLite readings database (save / recall / overlay / compare) and a REAL
offline AI waveform classifier (scikit-learn RandomForest, trained by train_model.py).

Run:   python dso_app.py
Deps:  pip install pyqtgraph pyserial numpy joblib scikit-learn
"""
import sys, time, csv, sqlite3, datetime
import numpy as np

from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from serial.tools import list_ports
import serial

import dso_protocol as P
from dso_features import extract

DB_PATH = "dso_readings.db"
MODEL_PATH = "dso_model.joblib"


# =====================================================================
#  Offline AI classifier (graceful fallback if model/sklearn missing)
# =====================================================================
class Classifier:
    def __init__(self):
        self.model = None
        self.classes = None
        try:
            import joblib
            d = joblib.load(MODEL_PATH)
            self.model, self.classes = d["model"], d["classes"]
            self.kind = "trained RandomForest"
        except Exception as e:
            self.kind = f"heuristic (no model: {e})"

    def predict(self, samples):
        # Degenerate guard: an essentially flat trace is DC, which the model
        # (trained on noisy frames) can read with low confidence. A real
        # instrument calls this DC directly.
        s = np.asarray(samples, dtype=np.float64)
        if (s.max() - s.min()) < 12:          # < ~10 mV of motion
            return "DC", 97.0
        feats = extract(samples).reshape(1, -1)
        if self.model is not None:
            proba = self.model.predict_proba(feats)[0]
            i = int(np.argmax(proba))
            return self.classes[i].upper(), float(proba[i]) * 100.0
        # fallback: crest-factor rule
        cr = feats[0, 0]
        if cr < 1.2:   return "SQUARE", 60.0
        if cr < 1.55:  return "SINE", 60.0
        if cr < 2.1:   return "TRIANGLE", 60.0
        return "NOISE", 50.0


# =====================================================================
#  SQLite readings database
# =====================================================================
class DB:
    def __init__(self, path=DB_PATH):
        self.cx = sqlite3.connect(path)
        self.cx.execute("""CREATE TABLE IF NOT EXISTS readings(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT, label TEXT, klass TEXT,
            vpp INTEGER, vmax INTEGER, vmin INTEGER, vavg INTEGER,
            vrms INTEGER, freq INTEGER, period INTEGER,
            cal TEXT, samples BLOB)""")
        self.cx.commit()

    def save(self, frame, klass, cal_name, label=""):
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.cx.execute(
            "INSERT INTO readings(ts,label,klass,vpp,vmax,vmin,vavg,vrms,freq,"
            "period,cal,samples) VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
            (ts, label, klass, frame.vpp, frame.vmax, frame.vmin, frame.vavg,
             frame.vrms, frame.freq, frame.period, cal_name,
             frame.samples.astype("<u2").tobytes()))
        self.cx.commit()

    def all(self):
        return self.cx.execute(
            "SELECT id,ts,label,klass,vpp,freq FROM readings ORDER BY id DESC"
        ).fetchall()

    def get(self, rid):
        row = self.cx.execute(
            "SELECT samples,cal,klass,vpp,freq,ts,label FROM readings WHERE id=?",
            (rid,)).fetchone()
        if not row:
            return None
        samples = np.frombuffer(row[0], dtype="<u2").copy()
        return dict(samples=samples, cal=row[1], klass=row[2], vpp=row[3],
                    freq=row[4], ts=row[5], label=row[6])

    def delete(self, rid):
        self.cx.execute("DELETE FROM readings WHERE id=?", (rid,))
        self.cx.commit()


# =====================================================================
#  Serial reader thread
# =====================================================================
class SerialReader(QtCore.QThread):
    frameReady = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._run = True
        self.sync = P.StreamSync()

    def run(self):
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.1)
        except Exception as e:
            self.status.emit(f"Open failed: {e}")
            return
        self.status.emit(f"Connected {self.port}")
        while self._run:
            try:
                data = ser.read(4096)
                if data:
                    for fr in self.sync.feed(data):
                        self.frameReady.emit(fr)
            except Exception as e:
                self.status.emit(f"Read error: {e}")
                break
        try:
            ser.close()
        except Exception:
            pass
        self.status.emit("Disconnected")

    def stop(self):
        self._run = False
        self.wait(800)


# =====================================================================
#  Main window
# =====================================================================
class DSOApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AAST DSO  \u2014  Laptop Scope")
        self.resize(1180, 720)

        self.db = DB()
        self.clf = Classifier()
        self.reader = None
        self.cal_name = "C"
        self.frozen = False
        self.last_frame = None
        self.overlay_curve = None

        self._build_ui()
        self._refresh_ports()
        self._refresh_table()
        self.statusBar().showMessage(f"AI: {self.clf.kind}")

    # ---------- UI ----------
    def _build_ui(self):
        pg.setConfigOptions(antialias=True, background="#0b0f14", foreground="#cdd6e0")
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # --- left: plot ---
        left = QtWidgets.QVBoxLayout()
        root.addLayout(left, 3)

        ctrl = QtWidgets.QHBoxLayout()
        left.addLayout(ctrl)
        self.port_box = QtWidgets.QComboBox()
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_conn.clicked.connect(self._toggle_connect)
        self.btn_refresh = QtWidgets.QPushButton("\u21bb")
        self.btn_refresh.setFixedWidth(32)
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_freeze = QtWidgets.QPushButton("Freeze")
        self.btn_freeze.setCheckable(True)
        self.btn_freeze.clicked.connect(self._toggle_freeze)
        self.cal_combo = QtWidgets.QComboBox()
        self.cal_combo.addItems(["Config C (offset 825, x2.0)",
                                 "Config A (offset 1266, x4.3)"])
        self.cal_combo.currentIndexChanged.connect(self._cal_changed)
        self.chk_cursors = QtWidgets.QCheckBox("Cursors")
        self.chk_cursors.toggled.connect(self._toggle_cursors)
        for w in (QtWidgets.QLabel("Port:"), self.port_box, self.btn_refresh,
                  self.btn_conn, self.btn_freeze, QtWidgets.QLabel("Cal:"),
                  self.cal_combo, self.chk_cursors):
            ctrl.addWidget(w)
        ctrl.addStretch(1)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Sample")
        self.plot.setLabel("left", "Probe voltage", units="mV")
        self.plot.setYRange(-3000, 6000)
        left.addWidget(self.plot, 1)

        self.curve = self.plot.plot(pen=pg.mkPen("#39d353", width=2))
        # 0 V offset reference line (draggable)
        self.zero_line = pg.InfiniteLine(angle=0, movable=True,
                                         pen=pg.mkPen("#888", style=QtCore.Qt.DashLine))
        self.zero_line.setValue(0)
        self.plot.addItem(self.zero_line)

        # cursors (hidden until enabled)
        self.cur_t1 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen("#f5a623"))
        self.cur_t2 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen("#f5a623"))
        self.cur_v1 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen("#4aa3ff"))
        self.cur_v2 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen("#4aa3ff"))
        self.cur_t1.setValue(80); self.cur_t2.setValue(240)
        self.cur_v1.setValue(-500); self.cur_v2.setValue(500)
        for c in (self.cur_t1, self.cur_t2, self.cur_v1, self.cur_v2):
            c.sigPositionChanged.connect(self._update_cursor_readout)
        self.cursor_lbl = pg.LabelItem(justify="left")
        self.cursor_lbl.setText("")

        # --- right: measurements + AI + database ---
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        self.meas_lbl = QtWidgets.QLabel("\u2014")
        self.meas_lbl.setStyleSheet("font-family:Consolas,monospace;font-size:13px;")
        self.meas_lbl.setAlignment(QtCore.Qt.AlignTop)
        box1 = QtWidgets.QGroupBox("Measurements")
        b1 = QtWidgets.QVBoxLayout(box1); b1.addWidget(self.meas_lbl)
        right.addWidget(box1)

        self.ai_lbl = QtWidgets.QLabel("\u2014")
        self.ai_lbl.setStyleSheet("font-size:18px;font-weight:bold;")
        box2 = QtWidgets.QGroupBox("AI classifier")
        b2 = QtWidgets.QVBoxLayout(box2); b2.addWidget(self.ai_lbl)
        right.addWidget(box2)

        box3 = QtWidgets.QGroupBox("Readings database")
        b3 = QtWidgets.QVBoxLayout(box3)
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["ID", "Time", "Class", "Vpp", "Freq"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        b3.addWidget(self.table)
        row = QtWidgets.QHBoxLayout(); b3.addLayout(row)
        for txt, fn in [("Save", self._save_reading), ("Overlay", self._overlay),
                        ("Compare 2", self._compare), ("Delete", self._delete),
                        ("Export CSV", self._export)]:
            btn = QtWidgets.QPushButton(txt); btn.clicked.connect(fn); row.addWidget(btn)
        right.addWidget(box3, 1)

        self._build_menu()

    def _build_menu(self):
        m = self.menuBar()
        fm = m.addMenu("&File")
        fm.addAction("Save reading", self._save_reading)
        fm.addAction("Export CSV", self._export)
        fm.addSeparator(); fm.addAction("Quit", self.close)
        vm = m.addMenu("&View")
        vm.addAction("Autoscale", lambda: self.plot.enableAutoRange())
        vm.addAction("Reset Y (-3..6 V)", lambda: self.plot.setYRange(-3000, 6000))
        vm.addAction("Toggle cursors", lambda: self.chk_cursors.toggle())
        hm = m.addMenu("&Help")
        hm.addAction("About", self._about)

    # ---------- ports / connect ----------
    def _refresh_ports(self):
        self.port_box.clear()
        ports = list(list_ports.comports())
        for p in ports:
            self.port_box.addItem(f"{p.device}  ({p.description})", p.device)
        if not ports:
            self.port_box.addItem("no ports found", None)

    def _toggle_connect(self):
        if self.reader is None:
            port = self.port_box.currentData()
            if not port:
                self.statusBar().showMessage("No serial port selected")
                return
            self.reader = SerialReader(port)
            self.reader.frameReady.connect(self._on_frame)
            self.reader.status.connect(lambda s: self.statusBar().showMessage(s))
            self.reader.start()
            self.btn_conn.setText("Disconnect")
        else:
            self.reader.stop(); self.reader = None
            self.btn_conn.setText("Connect")

    def _cal_changed(self, idx):
        self.cal_name = "A" if idx == 1 else "C"
        if self.last_frame:
            self._render(self.last_frame)

    def _toggle_freeze(self, on):
        self.frozen = on
        self.btn_freeze.setText("Frozen" if on else "Freeze")

    def _toggle_cursors(self, on):
        items = (self.cur_t1, self.cur_t2, self.cur_v1, self.cur_v2)
        for c in items:
            (self.plot.addItem if on else self.plot.removeItem)(c)
        if on:
            self.plot.addItem(self.cursor_lbl)
            self._update_cursor_readout()

    def _update_cursor_readout(self):
        dt = abs(self.cur_t2.value() - self.cur_t1.value())
        dv = abs(self.cur_v2.value() - self.cur_v1.value())
        self.cursor_lbl.setText(f"\u0394t = {dt:.1f} samples   \u0394V = {dv:.0f} mV")
        self.statusBar().showMessage(f"\u0394t={dt:.1f} samp  \u0394V={dv:.0f} mV")

    # ---------- live frame ----------
    def _on_frame(self, frame):
        self.last_frame = frame
        if not self.frozen:
            self._render(frame)

    def _render(self, frame):
        cal = P.CAL[self.cal_name]
        mv = frame.probe_mv(cal)
        self.curve.setData(np.arange(mv.size), mv)
        self.meas_lbl.setText(
            f"Vpp   : {frame.vpp:>6} mV\n"
            f"Vmax  : {frame.vmax:>6} mV\n"
            f"Vmin  : {frame.vmin:>6} mV\n"
            f"Vavg  : {frame.vavg:>6} mV\n"
            f"Vrms  : {frame.vrms:>6} mV\n"
            f"Freq  : {frame.freq:>6} Hz\n"
            f"Period: {frame.period:>6} us")
        klass, conf = self.clf.predict(frame.samples)
        self._last_class = klass
        self.ai_lbl.setText(f"{klass}\n{conf:.0f}% confidence")

    # ---------- database actions ----------
    def _save_reading(self):
        if not self.last_frame:
            self.statusBar().showMessage("No frame to save yet"); return
        klass = getattr(self, "_last_class", "?")
        label, ok = QtWidgets.QInputDialog.getText(self, "Save reading",
                                                   "Optional label:")
        self.db.save(self.last_frame, klass, self.cal_name, label if ok else "")
        self._refresh_table()
        self.statusBar().showMessage("Reading saved")

    def _refresh_table(self):
        rows = self.db.all()
        self.table.setRowCount(len(rows))
        for r, (rid, ts, label, klass, vpp, freq) in enumerate(rows):
            vals = [str(rid), ts, klass or "", f"{vpp} mV", f"{freq} Hz"]
            for c, v in enumerate(vals):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem(v))

    def _selected_ids(self):
        ids = []
        for idx in self.table.selectionModel().selectedRows():
            ids.append(int(self.table.item(idx.row(), 0).text()))
        return ids

    def _overlay(self):
        ids = self._selected_ids()
        if not ids:
            self.statusBar().showMessage("Select a saved row to overlay"); return
        rec = self.db.get(ids[0])
        mv = P.adc_to_probe_mv(rec["samples"], P.CAL[rec["cal"]])
        if self.overlay_curve:
            self.plot.removeItem(self.overlay_curve)
        self.overlay_curve = self.plot.plot(
            np.arange(mv.size), mv, pen=pg.mkPen("#f5a623", width=1, style=QtCore.Qt.DashLine))
        self.statusBar().showMessage(f"Overlaid reading #{ids[0]} ({rec['ts']})")

    def _compare(self):
        ids = self._selected_ids()
        if len(ids) < 2:
            self.statusBar().showMessage("Select TWO rows to compare"); return
        a, b = self.db.get(ids[0]), self.db.get(ids[1])
        dlg = QtWidgets.QDialog(self); dlg.setWindowTitle(f"Compare #{ids[0]} vs #{ids[1]}")
        dlg.resize(820, 480); lay = QtWidgets.QVBoxLayout(dlg)
        pw = pg.PlotWidget(); pw.showGrid(x=True, y=True, alpha=0.3)
        pw.setLabel("left", "mV"); pw.setLabel("bottom", "Sample")
        lay.addWidget(pw)
        ma = P.adc_to_probe_mv(a["samples"], P.CAL[a["cal"]])
        mb = P.adc_to_probe_mv(b["samples"], P.CAL[b["cal"]])
        pw.plot(np.arange(ma.size), ma, pen=pg.mkPen("#4aa3ff", width=2),
                name=f"#{ids[0]}")
        pw.plot(np.arange(mb.size), mb, pen=pg.mkPen("#f5a623", width=2),
                name=f"#{ids[1]}")
        pw.addLegend()
        lay.addWidget(QtWidgets.QLabel(
            f"#{ids[0]}: {a['klass']}  Vpp {a['vpp']}mV  {a['freq']}Hz   |   "
            f"#{ids[1]}: {b['klass']}  Vpp {b['vpp']}mV  {b['freq']}Hz   |   "
            f"\u0394Vpp {b['vpp']-a['vpp']}mV  \u0394F {b['freq']-a['freq']}Hz"))
        dlg.exec_()

    def _delete(self):
        for rid in self._selected_ids():
            self.db.delete(rid)
        self._refresh_table()

    def _export(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export CSV", "dso_readings.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "time", "label", "class", "vpp", "freq"])
            for row in self.db.all():
                w.writerow(row)
        self.statusBar().showMessage(f"Exported {path}")

    def _about(self):
        QtWidgets.QMessageBox.information(self, "About",
            "AAST DSO laptop companion\n\n"
            "Reads the STM32 USB-CDC stream (no firmware change), shows the live\n"
            "waveform with zoom/pan/cursors, stores readings in SQLite, and runs a\n"
            "trained offline RandomForest waveform classifier.\n\n"
            f"AI backend: {self.clf.kind}")

    def closeEvent(self, e):
        if self.reader:
            self.reader.stop()
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = DSOApp(); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
