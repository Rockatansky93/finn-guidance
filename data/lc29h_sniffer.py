"""
LC29H BA serial sniffer / DR diagnostic.

Two phases:
  1. RAW phase  — open the port at 115200 and dump every line for N seconds
                  with a tally of sentence types. Don't send anything.
                  This shows what the module is *currently* configured to do.

  2. CONFIG phase — replay the same enable commands finn-guidance sends,
                    then dump again. This shows what the module does after
                    config — and most importantly, whether $PQTMINS appears.

For each $PQTMINS we see, we break out the field values so you can read
nav_type / roll / pitch / heading / sat count directly without parsing
the whole thing in your head.

Usage:
    python lc29h_sniffer.py --port COM3
    python lc29h_sniffer.py --port COM3 --skip-config   # raw only
    python lc29h_sniffer.py --port COM3 --raw-secs 10 --post-secs 20

Requires: pip install pyserial
"""

import argparse
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# --- Sentence-type detection -------------------------------------------------

def sentence_type(line: str) -> str:
    """Return a coarse sentence-type tag for tallying."""
    if not line.startswith("$"):
        return "<non-NMEA>"
    head = line.split(",", 1)[0]  # e.g. "$GNGGA"
    # Group all $G* talkers together by message ID
    if len(head) >= 6 and head[1] == "G":
        return "$G..." + head[3:]   # "$GNGGA" -> "$G...GGA"
    return head


def nmea_checksum(body: str) -> str:
    """Compute NMEA XOR checksum for the given body (no leading '$', no '*')."""
    cs = 0
    for ch in body.encode("ascii"):
        cs ^= ch
    return f"{cs:02X}"


def build_cmd(body: str) -> bytes:
    """Build a full NMEA command line with checksum and CRLF."""
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


# --- $PQTMINS field decoder --------------------------------------------------

# Per the LC29H BA PQTM spec used by finn-guidance/parser.rs:
#   $PQTMINS,<MsgVer>,<TOW>,<InsNavType>,<Lat>,<Lon>,<Alt>,<AltMSL>,
#            <Speed2D>,<Speed3D>,<Roll>,<Pitch>,<Heading>,
#            <HACC>,<HDOP>,<PDOP>,<NumSV>*<cs>
PQTMINS_FIELDS = [
    "header", "MsgVer", "TOW", "InsNavType",
    "Lat", "Lon", "Alt", "AltMSL",
    "Speed2D", "Speed3D",
    "Roll", "Pitch", "Heading",
    "HACC", "HDOP", "PDOP", "NumSV",
]

NAV_TYPE_NAMES = {
    "0": "no solution",
    "1": "GNSS only",
    "2": "DR only",
    "3": "GNSS + DR (combined)",
}


def decode_pqtmins(line: str) -> dict:
    """Pull the interesting fields out of a $PQTMINS sentence."""
    body = line.split("*", 1)[0]
    parts = body.split(",")
    out = {}
    for i, name in enumerate(PQTMINS_FIELDS):
        if i < len(parts):
            out[name] = parts[i]
    return out


def fmt_pqtmins(line: str) -> str:
    f = decode_pqtmins(line)
    nav = f.get("InsNavType", "?")
    nav_name = NAV_TYPE_NAMES.get(nav, "?")
    return (
        f"PQTMINS  nav={nav}({nav_name})  "
        f"roll={f.get('Roll','?'):>7}  "
        f"pitch={f.get('Pitch','?'):>7}  "
        f"hdg={f.get('Heading','?'):>7}  "
        f"spd2D={f.get('Speed2D','?'):>6}  "
        f"sv={f.get('NumSV','?')}"
    )


# --- Capture loop ------------------------------------------------------------

def capture(ser, secs: float, label: str, show_pqtmins: bool = True):
    """Read for `secs` seconds. Print every $PQTMINS in detail. Tally the rest."""
    print(f"\n=== {label} - capturing for {secs:.0f}s ===")
    tally = Counter()
    samples = defaultdict(list)   # one example per sentence type
    t_end = time.monotonic() + secs
    pqtmins_count = 0

    while time.monotonic() < t_end:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"  serial error: {e}")
            break
        if not raw:
            continue
        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue

        st = sentence_type(line)
        tally[st] += 1
        if st not in samples:
            samples[st] = line

        if show_pqtmins and line.startswith("$PQTMINS"):
            pqtmins_count += 1
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"  [{ts}] {fmt_pqtmins(line)}")

    print(f"\n--- {label} summary ---")
    if not tally:
        print("  (nothing received - check port, baud, power, cable)")
        return tally, samples
    width = max(len(k) for k in tally)
    for st, n in tally.most_common():
        print(f"  {st.ljust(width)}  {n:5d}  e.g. {samples[st][:120]}")
    if show_pqtmins:
        if pqtmins_count == 0:
            print("\n  ** No $PQTMINS sentences observed. **")
        else:
            print(f"\n  $PQTMINS sentences observed: {pqtmins_count}")
    return tally, samples


# --- Config replay -----------------------------------------------------------

# These mirror reader.rs::ensure_module_config exactly.
CONFIG_COMMANDS = [
    # Enable PQTMINS at 10Hz (1=on), PQTMIMU off (0), PQTMGPS off (0), rate=10Hz
    ("enable PQTMINS @ 10Hz",      build_cmd("PQTMCFGEINSMSG,W,1,0,0,10")),
    # Save to NVS (matches the hardcoded *5A in reader.rs - verify it)
    ("save params to NVS",         build_cmd("PQTMSAVEPAR")),
    # Disable spammy NMEA we don't need
    ("disable GLL",                build_cmd("PAIR062,1,0")),
    ("disable GSA",                build_cmd("PAIR062,2,0")),
    ("disable GSV",                build_cmd("PAIR062,3,0")),
    ("disable RMC",                build_cmd("PAIR062,4,0")),
    # Set fix rate to 10Hz (100ms interval)
    ("set fix rate 10Hz",          build_cmd("PAIR050,100")),
]


def send_config(ser):
    print("\n=== Sending config commands ===")
    expected = nmea_checksum("PQTMSAVEPAR")
    if expected != "5A":
        print(f"  NOTE: reader.rs hardcodes PQTMSAVEPAR*5A, but real checksum is *{expected}.")
        print(f"        That command is being silently rejected by the module.")
    else:
        print(f"  PQTMSAVEPAR checksum *5A in reader.rs is correct.")

    for label, cmd in CONFIG_COMMANDS:
        print(f"  -> {label}: {cmd.decode('ascii').strip()}")
        ser.write(cmd)
        ser.flush()
        t_end = time.monotonic() + 0.25
        replies = []
        while time.monotonic() < t_end:
            line = ser.readline()
            if not line:
                continue
            txt = line.decode("ascii", errors="replace").strip()
            if txt.startswith("$PAIR001") or txt.startswith("$PQTM"):
                replies.append(txt)
        for r in replies[:3]:
            print(f"     <- {r}")
        time.sleep(0.1)


# --- Main --------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--raw-secs", type=float, default=8.0,
                    help="seconds to capture before sending any config")
    ap.add_argument("--post-secs", type=float, default=15.0,
                    help="seconds to capture after sending config")
    ap.add_argument("--skip-config", action="store_true",
                    help="don't send config commands; just observe")
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
        capture(ser, args.raw_secs, "PHASE 1: raw capture (no config sent)")
        if not args.skip_config:
            send_config(ser)
            time.sleep(0.5)
            ser.reset_input_buffer()
            capture(ser, args.post_secs, "PHASE 2: post-config capture")
    finally:
        ser.close()

    print("\nDone.\n")
    print("Interpretation guide:")
    print("  - If PHASE 1 shows $PQTMINS, the module already has it enabled (NVS persisted).")
    print("  - If PHASE 1 has none and PHASE 2 has them, the runtime enable works (NVS save may be failing).")
    print("  - If PHASE 2 still has no $PQTMINS, the PQTMCFGEINSMSG command form is wrong for this firmware.")
    print("  - InsNavType=0 = module has no nav solution yet (expected indoors). Roll/pitch will be empty/0.")
    print("  - InsNavType=1 = GNSS only, IMU not contributing. DR is uncalibrated.")
    print("  - InsNavType=2 or 3 = DR is active. Roll/pitch should be live.")


if __name__ == "__main__":
    main()
