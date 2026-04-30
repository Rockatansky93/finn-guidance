"""
LC29H BA DR diagnostic / probe.

This is the v2 of lc29h_sniffer.py, rewritten against the authoritative
LC29H(BA,CA,DA,EA) DR&RTK Application Note v1.1. The original sniffer
used command/parser logic copied from reader.rs which turned out to be
wrong on multiple counts (see strategy doc). This tool:

  1. Identifies the firmware variant via $PQTMVERNO. The BA ships in
     "two-wheel" and "four-wheel" software builds; the DR sentence set
     differs between them. Until we know which we're talking to, we
     can't pick the right enable command.

  2. Listens raw for N seconds to see what the module is currently
     emitting (carries over from v1).

  3. Sends *correct* config commands per the spec:
       - PQTMCFGEINSMSG with numeric Type=1 (Set), not the string "W".
       - PAIR6010,2,1 to ensure $PQTMDRCAL stays on (already on for
         this module, but harmless to re-assert).
       - PQTMSAVEPAR to commit to NVS.
       - $PQTMSRR to soft-reset so the new config takes effect THIS
         session (the spec mandates a reset for PQTMCFGEINSMSG).

  4. Reopens after reset and watches for $PQTMINS / $PQTMDRPVA. Decodes
     each sentence using the CORRECT field offsets from the app note.

  5. Includes a `--read-only` mode that probes the firmware version and
     the current configuration of the DR-related messages without
     sending any writes. Useful for sanity-checking what state the
     module is in before/after a real reader.rs run.

Usage:
    python lc29h_probe.py --port COM3
    python lc29h_probe.py --port COM3 --read-only
    python lc29h_probe.py --port COM3 --no-reset      # send config but skip the soft reset

Requires: pip install pyserial
"""

import argparse
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime

# Force line-buffered stdout so output appears live in Git Bash / VS Code
# terminals where Python's default block buffering would otherwise hide all
# our progress prints until the script exits.
try:
    sys.stdout.reconfigure(line_buffering=True)
except (AttributeError, Exception):
    pass

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# --- NMEA helpers ------------------------------------------------------------

def nmea_checksum(body):
    cs = 0
    for ch in body.encode("ascii"):
        cs ^= ch
    return f"{cs:02X}"


def build_cmd(body):
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def sentence_type(line):
    """Coarse tag for tally output."""
    if not line.startswith("$"):
        return "<non-NMEA>"
    head = line.split(",", 1)[0]
    if len(head) >= 6 and head[1] == "G":
        return "$G..." + head[3:]
    return head


# --- Field decoders (CORRECTED per spec v1.1) -------------------------------

# §3.1.6 PQTMINS
#   $PQTMINS,<Timestamp>,<SolType>,<Lat>,<Lon>,<Height>,<VEL_N>,<VEL_E>,<VEL_D>,<Roll>,<Pitch>,<Heading>
PQTMINS_FIELDS = [
    "header",      # 0  $PQTMINS
    "Timestamp",   # 1  ms since power-on
    "SolType",     # 2  0/1/2/3 — see SOLTYPE_NAMES
    "Lat",         # 3  deg
    "Lon",         # 4  deg
    "Height",      # 5  m
    "VEL_N",       # 6  m/s
    "VEL_E",       # 7  m/s
    "VEL_D",       # 8  m/s
    "Roll",        # 9  deg
    "Pitch",       # 10 deg
    "Heading",     # 11 deg
]

# §3.1.6 SolType
SOLTYPE_NAMES_INS = {
    "0": "DR not ready, roll/pitch only",
    "1": "DR not ready, GNSS+roll/pitch+rel-heading",
    "2": "GNSS+DR (calibrated)",
    "3": "DR only",
}

# §3.1.14 PQTMDRPVA
#   $PQTMDRPVA,<MsgVer>,<Timestamp>,<Time>,<SolType>,<Lat>,<Lon>,<Alt>,<Sep>,
#              <VelN>,<VelE>,<VelD>,<Spd>,<Roll>,<Pitch>,<Heading>
PQTMDRPVA_FIELDS = [
    "header",      # 0  $PQTMDRPVA
    "MsgVer",      # 1  always 1
    "Timestamp",   # 2  ms since power-on
    "Time",        # 3  hhmmss.sss
    "SolType",     # 4  0/1/2/3 — see SOLTYPE_NAMES_DRPVA
    "Lat",         # 5
    "Lon",         # 6
    "Alt",         # 7
    "Sep",         # 8  geoidal separation
    "VelN",        # 9
    "VelE",        # 10
    "VelD",        # 11
    "Spd",         # 12 ground speed m/s
    "Roll",        # 13 deg
    "Pitch",       # 14 deg
    "Heading",     # 15 deg
]

# §3.1.14 SolType (note: different mapping from PQTMINS!)
SOLTYPE_NAMES_DRPVA = {
    "0": "no fix",
    "1": "GNSS only",
    "2": "GNSS+DR (combined)",
    "3": "DR only",
}

# §3.1.1 PQTMDRCAL
#   $PQTMDRCAL,<MsgVer>,<CalState>,<NavType>
DRCAL_CALSTATE = {
    "0": "not calibrated",
    "1": "lightly calibrated",
    "2": "fully calibrated",
}
DRCAL_NAVTYPE = {
    "0": "no position",
    "1": "GNSS only",
    "2": "DR only",
    "3": "GNSS+DR",
}


def split_body(line):
    """Split a sentence into fields, dropping the *checksum tail."""
    body = line.split("*", 1)[0]
    return body.split(",")


def fmt_pqtmins(line):
    parts = split_body(line)
    def f(i): return parts[i] if i < len(parts) else ""
    sol = f(2)
    return (
        f"PQTMINS  sol={sol}({SOLTYPE_NAMES_INS.get(sol,'?')})  "
        f"roll={f(9):>9}  pitch={f(10):>9}  hdg={f(11):>9}"
    )


def fmt_pqtmdrpva(line):
    parts = split_body(line)
    def f(i): return parts[i] if i < len(parts) else ""
    sol = f(4)
    return (
        f"PQTMDRPVA sol={sol}({SOLTYPE_NAMES_DRPVA.get(sol,'?')})  "
        f"roll={f(13):>10}  pitch={f(14):>10}  hdg={f(15):>10}  spd={f(12):>6}"
    )


def fmt_pqtmdrcal(line):
    parts = split_body(line)
    def f(i): return parts[i] if i < len(parts) else ""
    cal, nav = f(2), f(3)
    return (
        f"PQTMDRCAL cal={cal}({DRCAL_CALSTATE.get(cal,'?')})  "
        f"nav={nav}({DRCAL_NAVTYPE.get(nav,'?')})"
    )


# --- Capture ----------------------------------------------------------------

def capture(ser, secs, label):
    """Read for `secs` seconds. Decode DR-related sentences inline. Tally rest."""
    print(f"\n=== {label} — capturing for {secs:.0f}s ===")
    tally = Counter()
    samples = defaultdict(str)
    t_end = time.monotonic() + secs
    interesting = 0

    while time.monotonic() < t_end:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"  serial error: {e}")
            break
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue

        st = sentence_type(line)
        tally[st] += 1
        if st not in samples:
            samples[st] = line

        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if line.startswith("$PQTMINS"):
            print(f"  [{ts}] {fmt_pqtmins(line)}")
            interesting += 1
        elif line.startswith("$PQTMDRPVA"):
            print(f"  [{ts}] {fmt_pqtmdrpva(line)}")
            interesting += 1
        elif line.startswith("$PQTMIMU,"):
            # Print every Nth raw IMU sample - they fire at 10Hz so we'd flood
            # the log otherwise. Show enough to see whether values are moving.
            if tally[st] % 5 == 1:
                print(f"  [{ts}] {line}")
        elif line.startswith("$PQTMDRCAL"):
            # Print every Nth one to show change without spamming
            if tally[st] % 5 == 1:
                print(f"  [{ts}] {fmt_pqtmdrcal(line)}")

    print(f"\n--- {label} summary ---")
    if not tally:
        print("  (nothing received — check port/baud/power/cable)")
        return tally, samples
    width = max(len(k) for k in tally)
    for st, n in tally.most_common():
        print(f"  {st.ljust(width)}  {n:5d}  e.g. {samples[st][:120]}")
    print(f"\n  DR navigation sentences (PQTMINS/PQTMDRPVA) observed: {interesting}")
    return tally, samples


# --- Synchronous query helpers ----------------------------------------------

def query(ser, cmd, expect_prefixes, wait=0.6):
    """Send a command and collect responses for `wait` seconds.

    Returns every line whose prefix matches any of `expect_prefixes`. Other
    streaming sentences (GGA/VTG/DRCAL/etc) are discarded but counted in
    the printed summary so we can see whether the module is alive.
    """
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    deadline = time.monotonic() + wait
    matched = []
    other = Counter()
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        if any(line.startswith(p) for p in expect_prefixes):
            matched.append(line)
        else:
            other[sentence_type(line)] += 1
    return matched


# --- Probe steps ------------------------------------------------------------

def probe_firmware_version(ser):
    """Send $PQTMVERNO and report the firmware string.

    Per the GNSS Protocol Spec §2.3.1, the response is:
        $PQTMVERNO,<VerStr>,<BuildDate>,<BuildTime>*<cs>
    """
    print("\n=== Probe: firmware version ($PQTMVERNO) ===")
    cmd = build_cmd("PQTMVERNO")
    print(f"  -> {cmd.decode('ascii').strip()}")
    replies = query(ser, cmd, expect_prefixes=("$PQTMVERNO,",), wait=1.0)
    if not replies:
        print("  ** No $PQTMVERNO response received within 1s. **")
        print("     Either the firmware doesn't support this query (unlikely) or")
        print("     the response was eaten by the streaming NMEA buffer. Try again.")
        return None
    line = replies[0]
    print(f"  <- {line}")
    parts = split_body(line)
    if len(parts) >= 4:
        ver, date, build_time = parts[1], parts[2], parts[3]
        print(f"     VerStr   = {ver}")
        print(f"     BuildDate = {date}")
        print(f"     BuildTime = {build_time}")
        # Look for two-wheel vs four-wheel hint in the version string
        ver_lower = ver.lower()
        if "tw" in ver_lower or "two" in ver_lower or "2w" in ver_lower:
            print("     (heuristic: looks like a TWO-WHEEL build → PQTMINS path)")
        elif "fw" in ver_lower or "four" in ver_lower or "4w" in ver_lower:
            print("     (heuristic: looks like a FOUR-WHEEL build → PQTMDRPVA path)")
        else:
            print("     (heuristic: variant not obvious from version string —")
            print("      we'll determine it empirically by which messages it accepts)")
        return ver
    return None


def probe_pqtmcfgeinsmsg_get(ser):
    """Read the current PQTMCFGEINSMSG state.

    Per spec §3.1.9, sending Get (Type=0) returns the current INS/IMU/GPS/Rate.
        //Get:  $PQTMCFGEINSMSG,0*0E
        //Resp: $PQTMCFGEINSMSG,0,<INS_Enabled>,<IMU_Enabled>,<GPS_Enabled>,<Rate>
        //  (or $PQTMEINSMSG variant — observed in v1.1 docs and our earlier capture)
    """
    print("\n=== Probe: PQTMCFGEINSMSG GET (current INS msg config) ===")
    cmd = build_cmd("PQTMCFGEINSMSG,0")
    print(f"  -> {cmd.decode('ascii').strip()}")
    replies = query(ser, cmd, expect_prefixes=("$PQTMCFGEINSMSG", "$PQTMEINSMSG"), wait=0.8)
    if not replies:
        print("  ** No reply. Either the firmware doesn't support PQTMCFGEINSMSG")
        print("     (likely a four-wheel build → use PQTMDRPVA path instead),")
        print("     or the Get form differs on this firmware revision.")
        return None
    for r in replies:
        print(f"  <- {r}")
    # Parse the first reply's enable flags
    line = replies[0]
    parts = split_body(line)
    # Layouts seen: $PQTMCFGEINSMSG,<Type>,<INS>,<IMU>,<GPS>,<Rate>
    #          or:  $PQTMEINSMSG,<Type>,<INS>,<IMU>,<GPS>,<Rate>
    if len(parts) >= 6:
        ins_en, imu_en, gps_en, rate = parts[2], parts[3], parts[4], parts[5]
        print(f"     PQTMINS enabled = {ins_en}")
        print(f"     PQTMIMU enabled = {imu_en}")
        print(f"     PQTMGPS enabled = {gps_en}")
        print(f"     output rate     = {rate} Hz")
        if ins_en == "1":
            print("     ** PQTMINS is already enabled. If we're not seeing it, the")
            print("        problem is upstream of enable — possibly module needs a")
            print("        reset, or this firmware doesn't support PQTMINS at all. **")
        return line
    return None


def send_correct_config(ser, fix_rate_hz, do_reset, enable_imu=False):
    """Send the corrected enable sequence per spec v1.1."""
    print("\n=== Sending CORRECTED config commands ===")

    # Step 1: Enable PQTMINS via PQTMCFGEINSMSG.
    # CORRECTED: <Type>=1 (Set), not "W".
    # INS_Enabled=1, IMU_Enabled=enable_imu, GPS_Enabled=0, Rate=10Hz.
    imu_flag = 1 if enable_imu else 0
    cmd = build_cmd(f"PQTMCFGEINSMSG,1,1,{imu_flag},0,10")
    label = "enable PQTMINS+PQTMIMU @ 10Hz" if enable_imu else "enable PQTMINS @ 10Hz"
    print(f"  -> {label}: {cmd.decode('ascii').strip()}")
    replies = query(ser, cmd, expect_prefixes=("$PQTMCFGEINSMSG", "$PQTMEINSMSG"), wait=0.8)
    saw_ok = False
    for r in replies:
        print(f"     <- {r}")
        if "OK" in r:
            saw_ok = True
    if not saw_ok:
        print("     (no OK ack seen — config write may have failed)")

    # Step 2: Make sure DR-telemetry messages stay on (PQTMDRCAL especially).
    # Per spec §3.2.1: $PAIR6010,<Type>,<Output_State>
    # Type 2 = $PQTMDRCAL.
    cmd = build_cmd("PAIR6010,2,1")
    print(f"  -> ensure PQTMDRCAL on: {cmd.decode('ascii').strip()}")
    replies = query(ser, cmd, expect_prefixes=("$PAIR001",), wait=0.4)
    for r in replies:
        print(f"     <- {r}")

    # Step 3: Save params to NVS. Spec §3.1.4: $PQTMSAVEPAR,OK*72 on success.
    cmd = build_cmd("PQTMSAVEPAR")
    print(f"  -> save to NVS: {cmd.decode('ascii').strip()}")
    replies = query(ser, cmd, expect_prefixes=("$PQTMSAVEPAR",), wait=1.0)
    for r in replies:
        print(f"     <- {r}")

    # Step 4: Soft reset so PQTMCFGEINSMSG actually takes effect.
    # Per the GNSS protocol spec the canonical reset is $PAIR007 (HOT_START)
    # or $PAIR009 (COLD_START). Hot start preserves almanac/ephemeris and
    # comes back fastest — that's what we want.
    if do_reset:
        cmd = build_cmd("PAIR007")
        print(f"  -> soft reset (hot start): {cmd.decode('ascii').strip()}")
        ser.write(cmd)
        ser.flush()
        # The module typically goes silent for a few seconds and then resumes
        # output. Give it 4s before we read again.
        print("     waiting 4s for module to come back...")
        time.sleep(4.0)
        ser.reset_input_buffer()
    else:
        print("  (skipping reset — config will take effect at next power cycle)")


# --- Main -------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--raw-secs", type=float, default=6.0,
                    help="seconds of raw capture before any writes")
    ap.add_argument("--post-secs", type=float, default=15.0,
                    help="seconds of capture after config + reset")
    ap.add_argument("--read-only", action="store_true",
                    help="just probe state — never send a write or reset")
    ap.add_argument("--no-reset", action="store_true",
                    help="send config but skip the soft reset")
    ap.add_argument("--enable-imu", action="store_true",
                    help="also enable raw $PQTMIMU output. Use to verify the IMU itself "
                         "is alive when PQTMINS reports zero attitude.")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: could not open {args.port}: {e}")
        sys.exit(2)

    time.sleep(0.2)
    ser.reset_input_buffer()

    try:
        # Phase 1: raw observation — what is the module doing right now?
        capture(ser, args.raw_secs, "PHASE 1: raw capture (no commands sent)")

        # Phase 2: identify firmware
        probe_firmware_version(ser)

        # Phase 3: read current PQTMCFGEINSMSG state
        probe_pqtmcfgeinsmsg_get(ser)

        if args.read_only:
            print("\n--read-only specified; not sending any writes or reset.")
            return

        # Phase 4: send the corrected config
        send_correct_config(ser, fix_rate_hz=10, do_reset=not args.no_reset,
                            enable_imu=args.enable_imu)

        # Phase 5: see what happens
        capture(ser, args.post_secs, "PHASE 5: post-config capture")

    finally:
        ser.close()

    print("\nDone.")
    print("\nWhat to look for in PHASE 5:")
    print("  - $PQTMINS streaming      → two-wheel firmware, PQTMINS path works.")
    print("  - $PQTMDRPVA streaming    → four-wheel firmware, use PQTMDRPVA path.")
    print("  - Neither, but PQTMDRCAL  → enable command still wrong, or this firmware")
    print("    keeps streaming           needs a different sentence to expose attitude.")
    print("  - Roll/Pitch are 0/empty  \u2192 expected indoors. The IMU outputs gravity-only")
    print("    on stationary bench       attitude; tilt the module ~10\u00b0 and rerun to see")
    print("                              roll respond. SolType=0 is normal pre-cal.")


if __name__ == "__main__":
    main()
