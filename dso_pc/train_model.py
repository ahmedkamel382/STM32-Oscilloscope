#!/usr/bin/env python3
"""
train_model.py  --  Trains the REAL offline waveform classifier (no internet).

A genuine scikit-learn RandomForest, not a rule table. It learns to tell
sine / square / triangle / noise / DC apart from shape features.

We synthesise training frames that look like what the STM32 streams (320 samples,
12-bit range, random amplitude / frequency / phase / offset / duty / noise). It
trains on FEATURES from the shared dso_features module, so the live app and the
model always compute features the exact same way.

Run once:   python train_model.py
Output:     dso_model.joblib   (loaded automatically by dso_app.py)
"""
import numpy as np, joblib
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from dso_features import extract, FEATURE_NAMES   # single source of truth

N = 320
CLASSES = ["sine", "square", "triangle", "noise", "dc"]
rng = np.random.default_rng(42)

def _frame(wave, amp, offset, noise):
    y = offset + amp * wave + rng.normal(0, noise, wave.size)
    return np.clip(y, 0, 4095)

def gen(cls):
    spc   = rng.uniform(8, 160)
    t     = np.arange(N)
    phase = rng.uniform(0, 2*np.pi)
    amp   = rng.uniform(150, 1900)
    offset= rng.uniform(amp+40, 4095-amp-40)
    noise = rng.uniform(1, 25)
    ph    = 2*np.pi*t/spc + phase
    if cls == "sine":
        w = np.sin(ph)
    elif cls == "square":
        duty = rng.uniform(0.3, 0.7)
        w = np.where((ph/(2*np.pi)) % 1.0 < duty, 1.0, -1.0)
    elif cls == "triangle":
        frac = (ph/(2*np.pi)) % 1.0
        w = 2.0*np.abs(2.0*(frac-0.5)) - 1.0
        if rng.random() < 0.4:
            w = 2.0*frac - 1.0                      # sawtooth variant
    elif cls == "noise":
        w = rng.normal(0, 0.6, N); amp = rng.uniform(300,1900)
        offset = rng.uniform(1200,2900); noise = 0.0
    else:  # dc
        w = np.zeros(N); amp = 0.0
        offset = rng.uniform(200,3900); noise = rng.uniform(1,12)
    return _frame(w, amp, offset, noise)

def build(per_class=1200):
    X, y = [], []
    for ci, c in enumerate(CLASSES):
        for _ in range(per_class):
            X.append(extract(gen(c))); y.append(ci)
    return np.array(X, dtype=np.float32), np.array(y)

def main():
    print("Synthesising training waveforms ...")
    X, y = build()
    Xtr, Xte, ytr, yte = train_test_split(X, y, test_size=0.2,
                                          random_state=1, stratify=y)
    print(f"  train={len(Xtr)} test={len(Xte)} features={X.shape[1]}")
    clf = RandomForestClassifier(n_estimators=160, max_depth=14,
                                 random_state=1, n_jobs=-1)
    clf.fit(Xtr, ytr)
    print("\nHeld-out performance:")
    print(classification_report(yte, clf.predict(Xte), target_names=CLASSES))
    joblib.dump({"model": clf, "classes": CLASSES, "features": FEATURE_NAMES},
                "dso_model.joblib")
    print("Saved -> dso_model.joblib")

if __name__ == "__main__":
    main()
