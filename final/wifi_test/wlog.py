"""wlog.py: timestamped serial logger and summary for wifi_diag.
usage: python wlog.py COM5 base_ap --minutes 10   (needs: pip install pyserial)"""
import argparse, collections, os, re, statistics, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("port"); ap.add_argument("label")
ap.add_argument("--minutes", type=float, default=10)
args = ap.parse_args()

os.makedirs("logs", exist_ok=True)
path = f"logs/{args.label}_{time.strftime('%H%M%S')}.log"
log = open(path, "w", buffering=1)
kv = re.compile(r"(\w+)=(-?[\d.]+)")
t0 = time.time()
boot_t, cfg, boots, port_drops = None, "", 0, 0
counts = collections.Counter()
stall_events, last_err_t, pre_stall = [], -1e9, None
recent = collections.deque(maxlen=6)
S = []

def uptime(): return time.time() - (boot_t or t0)

def handle(line):
    global boot_t, cfg, boots, last_err_t, pre_stall
    now = time.time()
    log.write(f"{now - t0:9.2f} {line}\n")
    if line.startswith("CFG"):
        boot_t, cfg = now, line; boots += 1
    elif line.startswith("S "):
        S.append({k: float(v) for k, v in kv.findall(line)}); recent.append(line)
    elif line.startswith("[CYW43]"):
        for key in ("STALL", "do_ioctl", "send_ethernet", "Bus error"):
            if key in line: counts[key] += 1
        if now - last_err_t > 5:                    # a new burst of driver errors
            stall_events.append(uptime())
            if pre_stall is None: pre_stall = list(recent)
        last_err_t = now
    elif line.startswith(("LINK DEAD", "link recovered", "BOOT:")):
        counts[line.split(",")[0]] += 1

ser, end = None, t0 + args.minutes * 60
try:
    while time.time() < end:
        if ser is None:
            try: ser = serial.Serial(args.port, 115200, timeout=0.5)
            except serial.SerialException: time.sleep(0.5); continue
        try: raw = ser.readline()
        except (serial.SerialException, OSError):
            port_drops += 1; ser = None
            log.write(f"{time.time() - t0:9.2f} <<PORT DROPPED>>\n"); continue
        if raw: handle(raw.decode(errors="replace").strip())
except KeyboardInterrupt:fffff
    pass

print(f"\nlog: {path}\n{cfg}")
print(f"duration {(time.time() - t0) / 60:.1f} min, boots seen {boots}, port drops {port_drops}")
print("driver/link messages:", dict(counts) or "none")
print("stall events at uptime (s):", [round(x) for x in stall_events] or "none")
def col(k): return [s[k] for s in S if k in s]
for k, name in (("smax", "worst udp_sendto ms"), ("loopmax", "worst loop gap ms"),
                ("ledmax", "worst LED call ms"), ("maxgap", "worst rx gap ms")):
    v = col(k)
    if v: print(f"{name}: max {max(v):.1f}, median {statistics.median(v):.1f}")
for k in ("rssi", "prssi"):
    v = [x for x in col(k) if x != 0]
    if v: print(f"{k}: min {min(v):.0f} median {statistics.median(v):.0f} max {max(v):.0f}")
if S: print(f"lost (last) {S[-1].get('lost', 0):.0f}, rx total {S[-1].get('rx', 0):.0f}")
if pre_stall:
    print("\nstats lines just before the first stall:")
    for l in pre_stall: print("  " + l)