"""
dso_features.py  --  Extract calibration-independent features from a 320-sample
frame, used by BOTH the offline trainer and the live app so they always agree.

Features are computed from raw ADC counts (0..4095) and normalized, so they do
not depend on the front-end calibration or the absolute amplitude.
"""
import numpy as np

FEATURE_NAMES = [
    "crest",            # peak/rms  (sine~1.41, square~1.0, triangle~1.73)
    "center_frac",      # fraction of samples near the mean
    "extremes_frac",    # fraction near the top/bottom of the range
    "zcr",              # mean-crossing rate (normalized)
    "spectral_flatness",# 1.0 ~ white noise, ~0 ~ single tone
    "fund_prominence",  # strongest FFT bin / total spectral energy
    "odd_harm_ratio",   # energy at 3f/5f vs fundamental (square/triangle high)
    "abs_diff",         # mean |x[n]-x[n-1]| normalized by ptp
    "diff_crest",       # crest factor of the derivative (square edges spike)
    "bimodality",       # histogram bimodality (square sits at 2 levels)
]


def extract(samples):
    """samples: array-like of 320 ADC counts. Returns np.float32 feature vector."""
    x = np.asarray(samples, dtype=np.float64)
    n = x.size
    mean = x.mean()
    ac = x - mean
    ptp = x.max() - x.min()
    if ptp < 1:
        ptp = 1.0
    rms = np.sqrt(np.mean(ac * ac))
    rms_safe = rms if rms > 1e-6 else 1e-6

    crest = (ptp / 2.0) / rms_safe

    band = 0.10 * ptp
    center_frac = np.mean(np.abs(ac) < band)
    extremes_frac = np.mean((x > x.max() - band) | (x < x.min() + band))

    # mean-crossings
    sign = np.sign(ac)
    sign[sign == 0] = 1
    zcr = np.mean(sign[1:] != sign[:-1])

    # spectrum (ignore DC bin)
    win = np.hanning(n)
    mag = np.abs(np.fft.rfft(ac * win))
    mag[0] = 0.0
    mtot = mag.sum() + 1e-9
    # spectral flatness
    p = mag + 1e-9
    flat = np.exp(np.mean(np.log(p))) / (np.mean(p) + 1e-9)
    # fundamental
    k = int(np.argmax(mag))
    fund_prom = mag[k] / mtot
    # odd-harmonic energy near 3k, 5k
    def bin_energy(c):
        if c <= 0 or c >= mag.size:
            return 0.0
        lo, hi = max(0, c - 1), min(mag.size, c + 2)
        return mag[lo:hi].sum()
    fund_e = bin_energy(k) + 1e-9
    odd_harm = (bin_energy(3 * k) + bin_energy(5 * k)) / fund_e

    d = np.diff(x)
    abs_diff = np.mean(np.abs(d)) / ptp
    d_rms = np.sqrt(np.mean((d - d.mean()) ** 2))
    diff_crest = (np.abs(d).max()) / (d_rms + 1e-6)

    # bimodality: split range in halves around mid, see how empty the middle is
    mid = (x.max() + x.min()) / 2.0
    half = ptp / 4.0
    near_mid = np.mean(np.abs(x - mid) < half)
    bimodality = 1.0 - near_mid   # high if samples avoid the middle (square)

    return np.array([crest, center_frac, extremes_frac, zcr, flat,
                     fund_prom, odd_harm, abs_diff, diff_crest, bimodality],
                    dtype=np.float32)
