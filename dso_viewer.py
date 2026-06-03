#!/usr/bin/env python3
"""
================================================================================
  DSO Viewer  --  STM32F401 USB Oscilloscope, PC display
================================================================================
  Connects to the STM32F401 Black Pill over USB CDC (Virtual COM Port) and
  shows the waveform, on-board measurements, and FFT in real time.

  Packet layout shipped by main.c (fixed 736 bytes per packet):
      bytes [0..95]   ASCII "DSO,vmax,vmin,vpp,vavg,vrms,freq,period\n"
                      padded to exactly 96 bytes (95th byte is '\n').
      bytes [96..735] 320 raw ADC samples, little-endian uint16.

  Requirements:
      pip install pyserial numpy matplotlib

  Usage:
      python dso_viewer.py                  (auto-detect)
      python dso_viewer.py COM5             (Windows)
      python dso_viewer.py /dev/ttyACM0     (Linux / macOS)
================================================================================
"""

import sys
import time
import struct
import argparse

import serial
import serial.tools.list_ports
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# =============================================================================
#  Configuration  --  MUST match firmware (dso.h)
# =============================================================================
BAUD_RATE       = 115200    # USB CDC ignores this, but pyserial needs a value
SAMPLE_COUNT    = 320
HEADER_BYTES    = 96
PACKET_BYTES    = HEADER_BYTES + SAMPLE_COUNT * 2   # = 736 bytes total

VREF_MV         = 3300
ADC_MAX         = 4095

# Calibration constants -- IDENTICAL to dso.h
# V_probe_mv = (V_adc_mv - FE_OFFSET_MV) * FE_GAIN_NUM / FE_GAIN_DEN
FE_OFFSET_MV    = 1266
FE_GAIN_NUM     = 43
FE_GAIN_DEN     = 10


# =============================================================================
#  Auto-detect STM32 USB serial port
# =============================================================================
def find_stm32_port():
    """Pick a likely STM32 VCP from the system."""
    ports = list(serial.tools.list_ports.comports())
    priority_words = ('STM32', 'STLINK', 'VIRTUAL COM', 'USB SERIAL', 'CDC', 'ACM')
    for p in ports:
        desc = (p.description or '').upper()
        if any(w in desc for w in priority_words):
            return p.device
    return ports[0].device if ports else None


# =============================================================================
#  Serial connection
# =============================================================================
def connect_serial(port_name=None):
    if not port_name:
        port_name = find_stm32_port()
        if port_name is None:
            print("[ERROR] No serial port found.  Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"   {p.device:<14}  {p.description}")
            sys.exit(1)
        print(f"[INFO] Auto-detected port: {port_name}")

    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.25,
        )
        time.sleep(0.5)
        ser.reset_input_buffer()
        print(f"[INFO] Connected to {port_name}")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port_name}: {e}")
        sys.exit(1)


# =============================================================================
#  Streaming reader  --  sync on "DSO," header, read exactly PACKET_BYTES
# =============================================================================
class DSOReader:
    """
    Maintains a rolling byte buffer.  Each frame, swallows whatever is
    waiting on the wire then keeps slicing complete 736-byte packets.
    """
    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.latest_meas    = None
        self.latest_samples = None
        self.packet_count = 0
        self.error_count  = 0

    def update(self):
        if not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.buf.extend(self.ser.read(waiting))
        except serial.SerialException:
            return

        # Cap buffer growth so a stalled GUI doesn't eat all memory
        if len(self.buf) > 8 * PACKET_BYTES:
            self.buf = self.buf[-4 * PACKET_BYTES:]

        # Pull every complete packet from the rolling buffer
        while True:
            idx = self.buf.find(b'DSO,')
            if idx < 0 or (len(self.buf) - idx) < PACKET_BYTES:
                break
            pkt = bytes(self.buf[idx:idx + PACKET_BYTES])
            del self.buf[:idx + PACKET_BYTES]
            self._parse_packet(pkt)

    def _parse_packet(self, pkt):
        try:
            header = pkt[:HEADER_BYTES].split(b'\n', 1)[0].decode('ascii', 'ignore')
            parts  = header.strip().split(',')
            if len(parts) < 8 or parts[0] != 'DSO':
                self.error_count += 1
                return

            meas = {
                'vmax_mv'  : int(parts[1]),
                'vmin_mv'  : int(parts[2]),
                'vpp_mv'   : int(parts[3]),
                'vavg_mv'  : int(parts[4]),
                'vrms_mv'  : int(parts[5]),
                'freq_hz'  : int(parts[6]),
                'period_us': int(parts[7]),
            }

            raw = struct.unpack(f'<{SAMPLE_COUNT}H', pkt[HEADER_BYTES:])
            adc = np.asarray(raw, dtype=np.float64)
            v_adc_mv   = adc * VREF_MV / ADC_MAX
            v_probe_mv = (v_adc_mv - FE_OFFSET_MV) * FE_GAIN_NUM / FE_GAIN_DEN

            self.latest_meas    = meas
            self.latest_samples = v_probe_mv
            self.packet_count  += 1

        except (ValueError, struct.error, UnicodeDecodeError):
            self.error_count += 1


# =============================================================================
#  Plot window
# =============================================================================
class DSOPlotter:
    def __init__(self, reader):
        self.reader = reader

        self.fig = plt.figure(figsize=(13, 8), facecolor='#1a1a2e')
        self.fig.suptitle('AAST DSO  -  STM32F401 Digital Storage Oscilloscope',
                          color='white', fontsize=14, fontweight='bold')

        # Main waveform
        self.ax_wave = self.fig.add_subplot(2, 2, (1, 2))
        self.ax_wave.set_facecolor('#0a0a0a')
        self.line_wave, = self.ax_wave.plot([], [], color='#00ff00', lw=1.4)
        self.ax_wave.set_xlim(0, SAMPLE_COUNT)
        self.ax_wave.set_ylim(-5000, 5000)
        self.ax_wave.set_xlabel('Sample',           color='white')
        self.ax_wave.set_ylabel('Probe voltage (mV)', color='white')
        self.ax_wave.set_title('Waveform',          color='yellow')
        self.ax_wave.tick_params(colors='white')
        self.ax_wave.grid(True, alpha=0.3, color='gray')
        self.ax_wave.axhline(y=0, color='red', ls='--', alpha=0.5, lw=0.8)
        for i in range(-4, 5):
            if i:
                self.ax_wave.axhline(y=i * 1000, color='#333333', ls=':',
                                     alpha=0.5, lw=0.5)

        # Measurements panel
        self.ax_meas = self.fig.add_subplot(2, 2, 3)
        self.ax_meas.set_facecolor('#16213e')
        self.ax_meas.set_xlim(0, 1)
        self.ax_meas.set_ylim(0, 1)
        self.ax_meas.axis('off')
        self.ax_meas.set_title('Measurements', color='yellow')
        self.meas_text = self.ax_meas.text(
            0.05, 0.95, 'Waiting for data...',
            transform=self.ax_meas.transAxes,
            color='cyan', fontsize=11,
            verticalalignment='top', fontfamily='monospace')

        # FFT panel
        self.ax_fft = self.fig.add_subplot(2, 2, 4)
        self.ax_fft.set_facecolor('#0a0a0a')
        self.line_fft, = self.ax_fft.plot([], [], color='#ff8c00', lw=1.0)
        self.ax_fft.set_xlim(0, 5000)
        self.ax_fft.set_ylim(0, 100)
        self.ax_fft.set_xlabel('Frequency (Hz)', color='white')
        self.ax_fft.set_ylabel('Magnitude (dB)', color='white')
        self.ax_fft.set_title('FFT spectrum',    color='yellow')
        self.ax_fft.tick_params(colors='white')
        self.ax_fft.grid(True, alpha=0.3, color='gray')

        self.status_text = self.fig.text(
            0.5, 0.02, 'Initializing...', ha='center', color='lime',
            fontsize=9, fontfamily='monospace')

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        self.start_time  = time.time()
        self.frame_count = 0

    @staticmethod
    def compute_fft(samples):
        if samples is None or len(samples) < 16:
            return np.array([]), np.array([])
        window   = np.hanning(len(samples))
        spectrum = np.fft.rfft(samples * window)
        mag_db   = 20.0 * np.log10(np.abs(spectrum) + 1e-10)
        # Approx sample rate at the default 100 us/div timebase:
        #   ARR = 261, PSC = 0  ->  Fs = 84e6 / 262 ~= 320.6 kHz
        fs = 320_610
        freqs = np.fft.rfftfreq(len(samples), 1 / fs)
        return freqs, mag_db

    def update(self, frame):
        self.reader.update()
        meas    = self.reader.latest_meas
        samples = self.reader.latest_samples

        if samples is not None:
            x = np.arange(len(samples))
            self.line_wave.set_data(x, samples)

            vmax = float(np.max(samples))
            vmin = float(np.min(samples))
            margin = max(500.0, (vmax - vmin) * 0.2)
            self.ax_wave.set_ylim(vmin - margin, vmax + margin)

            freqs, mag_db = self.compute_fft(samples)
            if len(freqs):
                self.line_fft.set_data(freqs, mag_db)
                if len(mag_db) > 1:
                    peak = int(np.argmax(mag_db[1:])) + 1
                    peak_freq = float(freqs[min(peak, len(freqs) - 1)])
                    self.ax_fft.set_xlim(0, max(1000.0,
                                                min(peak_freq * 5.0, 50_000.0)))

        if meas is not None:
            freq_disp = (f"{meas['freq_hz'] / 1000:.2f} kHz"
                         if meas['freq_hz'] >= 1000
                         else f"{meas['freq_hz']} Hz")

            block = (f"  Vmax : {meas['vmax_mv']:>+7d} mV\n"
                     f"  Vmin : {meas['vmin_mv']:>+7d} mV\n"
                     f"  Vpp  : {meas['vpp_mv']:>7d} mV\n"
                     f"  Vavg : {meas['vavg_mv']:>+7d} mV\n"
                     f"  Vrms : {meas['vrms_mv']:>7d} mV\n"
                     f"  Freq : {freq_disp:>11}\n"
                     f"  Per. : {meas['period_us']:>7d} us")
            self.meas_text.set_text(block)

            # Signal-type heuristic for the status bar
            if abs(meas['vavg_mv']) > 100 and meas['freq_hz'] < 5:
                sig_type, color = 'DC signal', 'cyan'
            elif meas['freq_hz'] >= 5:
                sig_type, color = 'AC signal', 'yellow'
            else:
                sig_type, color = 'No signal / 0 V', 'lime'

            elapsed = max(time.time() - self.start_time, 0.001)
            fps = self.frame_count / elapsed
            self.status_text.set_text(
                f"[{sig_type}]   "
                f"Packets {self.reader.packet_count}   "
                f"Errors {self.reader.error_count}   "
                f"FPS {fps:5.1f}")
            self.status_text.set_color(color)

        self.frame_count += 1
        return self.line_wave, self.line_fft, self.meas_text, self.status_text

    def run(self):
        self.ani = FuncAnimation(
            self.fig, self.update,
            interval=50, blit=False, cache_frame_data=False)
        plt.show()


# =============================================================================
#  Entry point
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="STM32 DSO USB viewer")
    parser.add_argument('port', nargs='?', default=None,
                        help="Serial port (e.g. COM5 or /dev/ttyACM0). "
                             "If omitted, auto-detects.")
    args = parser.parse_args()

    print("=" * 70)
    print("  AAST DSO Viewer   -   STM32F401 USB Oscilloscope")
    print("=" * 70)
    print("\nAvailable serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"   {p.device:<14}  {p.description}")
    print()

    ser = connect_serial(args.port)
    reader  = DSOReader(ser)
    plotter = DSOPlotter(reader)
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        ser.close()
        print("[INFO] Disconnected.")


if __name__ == '__main__':
    main()
