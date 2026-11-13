import sys, struct, time, wave
import numpy as np
import serial
import matplotlib.pyplot as plt

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 15
FS, N = 16000, 128
SYNC = b"\xAA\x55\xAA\x55"
FMT = "<III%dh" % N
SIZE = struct.calcsize(FMT)   # 268

ser = serial.Serial(PORT, 115200, timeout=0.1)
buf = bytearray()
samples, counters = [], []
stalls, resyncs = 0, 0
print(f"Recording {SECONDS:.0f} s from {PORT} ... (Ctrl-C to stop early)")
t0 = time.time()
try:
    while time.time() - t0 < SECONDS:
        buf += ser.read(4096)
        while True:
            i = buf.find(SYNC)
            if i < 0:
                del buf[:-3]          # keep a possible partial sync
                break
            if i > 0:
                resyncs += 1
                del buf[:i]
            if len(buf) < SIZE:
                break
            _, cnt, st, *s = struct.unpack(FMT, bytes(buf[:SIZE]))
            del buf[:SIZE]
            counters.append(cnt)
            samples.append(s)
            stalls = st
except KeyboardInterrupt:
    pass
ser.close()

if not samples:
    sys.exit("No frames received. Is the capture firmware flashed and the serial monitor closed?")

x = np.array(samples, dtype=np.int16).reshape(-1)
c = np.array(counters, dtype=np.int64)
gaps = np.flatnonzero(np.diff(c) != 1)
lost = int(sum(c[g + 1] - c[g] - 1 for g in gaps))

with wave.open("capture.wav", "wb") as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS)
    w.writeframes(x.tobytes())

xf = x.astype(np.float64)
rms = np.sqrt(np.mean((xf - xf.mean()) ** 2))
dbfs = 20 * np.log10((rms + 1e-9) / 32768)

win = np.hanning(len(xf))
spec = np.abs(np.fft.rfft(xf * win))
freqs = np.fft.rfftfreq(len(xf), 1 / FS)
mask = freqs > 50
pk = freqs[mask][np.argmax(spec[mask])]

u = x.view(np.uint16)
ones = [float(np.mean((u >> b) & 1)) for b in range(16)]
stuck_low = [b for b in range(8) if ones[b] < 0.001 or ones[b] > 0.999]

print(f"frames={len(c)}  gaps={len(gaps)}  lost_frames={lost}  resyncs={resyncs}  firmware_stalls={stalls}")
print(f"mean={xf.mean():.1f}  min={x.min()}  max={x.max()}  rms={rms:.1f} ({dbfs:.1f} dBFS, 16-bit)")
print(f"FFT peak (>50 Hz) = {pk:.1f} Hz")
print(f"stuck low bits (0..7): {stuck_low if stuck_low else 'none'}")
print("saved capture.wav")

fig, ax = plt.subplots(2, 1, figsize=(10, 7))
ax[0].plot(np.arange(len(x)) / FS, x, lw=0.5)
ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("sample")
ax[1].plot(freqs, 20 * np.log10(spec / spec.max() + 1e-12), lw=0.5)
ax[1].set_xlabel("Hz"); ax[1].set_ylabel("dB rel. peak"); ax[1].set_ylim(-120, 5)
plt.tight_layout(); plt.show()