"""
dso_protocol.py  --  Parses the AAST DSO USB-CDC packet stream.

The firmware (main.c) sends one fixed 736-byte packet ~20x/second:
    bytes [0..95]   : ASCII  "DSO,vmax,vmin,vpp,vavg,vrms,freq,period\n"  (space padded)
    bytes [96..735] : 320 raw ADC samples, little-endian uint16 (0..4095)

This module only READS that stream; no firmware change is needed.
"""
import numpy as np

HEADER_LEN  = 96
SAMPLE_COUNT = 320
PACKET_LEN  = HEADER_LEN + SAMPLE_COUNT * 2     # 736
SYNC        = b"DSO,"

# --- Front-end calibration (must match dso.h FE_HW_CONFIG) ---------------
# Config C (current firmware): offset 825, gain 20/10
# Config A (after the fix)   : offset 1266, gain 43/10
CAL = {
    "A": dict(offset_mv=1266, gain_num=43, gain_den=10),
    "C": dict(offset_mv=825,  gain_num=20, gain_den=10),
}
VREF_MV = 3300
ADC_MAX = 4095


def adc_to_probe_mv(counts, cal):
    """Vectorised count -> probe millivolts, mirroring DSO_AdcToProbeMv()."""
    counts = np.asarray(counts, dtype=np.float64)
    vadc = counts * VREF_MV / ADC_MAX
    return (vadc - cal["offset_mv"]) * cal["gain_num"] / cal["gain_den"]


class Frame:
    """One decoded packet: header measurements + raw samples."""
    __slots__ = ("vmax", "vmin", "vpp", "vavg", "vrms", "freq", "period",
                 "samples")

    def __init__(self, fields, samples):
        (self.vmax, self.vmin, self.vpp, self.vavg,
         self.vrms, self.freq, self.period) = fields
        self.samples = samples            # np.uint16 array, length 320

    def probe_mv(self, cal):
        return adc_to_probe_mv(self.samples, cal)


def parse_packet(buf):
    """Parse exactly one PACKET_LEN slice. Returns Frame or None."""
    if len(buf) < PACKET_LEN:
        return None
    header = bytes(buf[:HEADER_LEN])
    if not header.startswith(SYNC):
        return None
    try:
        line = header.split(b"\n", 1)[0].decode("ascii", "ignore")
        parts = line.split(",")
        # parts[0] == "DSO", then 7 numbers
        nums = [int(p) for p in parts[1:8]]
        if len(nums) != 7:
            return None
    except Exception:
        return None
    samples = np.frombuffer(bytes(buf[HEADER_LEN:PACKET_LEN]), dtype="<u2").copy()
    if samples.size != SAMPLE_COUNT:
        return None
    return Frame(nums, samples)


class StreamSync:
    """Accumulates serial bytes and yields complete frames, re-syncing on SYNC."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        # cap runaway buffering
        if len(self.buf) > PACKET_LEN * 8:
            self.buf = self.buf[-PACKET_LEN * 4:]
        frames = []
        while True:
            idx = self.buf.find(SYNC)
            if idx < 0:
                if len(self.buf) > PACKET_LEN:
                    self.buf = self.buf[-3:]   # keep tail in case SYNC is split
                break
            if idx > 0:
                del self.buf[:idx]             # drop junk before SYNC
            if len(self.buf) < PACKET_LEN:
                break
            frame = parse_packet(self.buf[:PACKET_LEN])
            del self.buf[:PACKET_LEN]
            if frame is not None:
                frames.append(frame)
        return frames


def build_test_packet(samples, vmax=1000, vmin=-1000, vpp=2000, vavg=0,
                      vrms=707, freq=1000, period=1000):
    """Build a byte-exact firmware packet for testing the parser."""
    s = (f"DSO,{vmax},{vmin},{vpp},{vavg},{vrms},{freq},{period}\n").encode("ascii")
    hdr = bytearray(b" " * HEADER_LEN)
    hdr[:len(s)] = s
    hdr[95:96] = b"\n"
    samples = np.asarray(samples, dtype="<u2")
    return bytes(hdr) + samples.tobytes()
