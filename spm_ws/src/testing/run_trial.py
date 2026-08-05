#!/usr/bin/env python3
"""
spm data collection utility
opens teensy and platform_esp32c6 serial ports:
1. send command over serial to teensy to start a traj
2. log all outputs (des rotation matrix + quaternions from imu) from both serials 
with timestamps (make sure teensy serial is clean)
3. write to csv/generate plots
4. calculate geometric orientation error (max)

python3 run_trial.py --teensy /dev/ttyACM0 --dongle /dev/ttyACM2 --cmd [#] --duration [#] --out trial01.csv

--cmd: passed through teensy serial command handler
"""

import argparse
import csv
import queue
import sys
import threading
import time

import serial


def reader_thread(name, ser, out_q, stop_evt):
    buf = b""
    while not stop_evt.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as e:
            out_q.put((time.monotonic(), name, f"[ERROR] {e}"))
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = time.monotonic()
            text = line.decode(errors="replace").strip("\r")
            if text:
                out_q.put((t, name, text))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--teensy", required=True, help="teensy serial port (usually /dev/ttyACM0)")
    ap.add_argument("--dongle", required=True, help="dongle serial port (usually /dev/ttyACM2) or acm1")
    ap.add_argument("--teensy-baud", type=int, default=115200)
    ap.add_argument("--dongle-baud", type=int, default=115200)
    ap.add_argument("--cmd", required=True, help="traj command sent to teensy")
    ap.add_argument("--duration", type=float, default=5.0, help="max seconds to log after trigger")
    ap.add_argument("--out", required=True, help="output csv path")
    ap.add_argument("--settle", type=float, default=0.5, help="seconds to log before sending the trigger")
    args = ap.parse_args()

    print(f"Opening Teensy port {args.teensy} @ {args.teensy_baud}...")
    teensy = serial.Serial(args.teensy, args.teensy_baud, timeout=0.05)
    print(f"Opening dongle port {args.dongle} @ {args.dongle_baud}...")
    dongle = serial.Serial(args.dongle, args.dongle_baud, timeout=0.05)

    # Let both boards finish any boot chatter
    time.sleep(0.5)
    teensy.reset_input_buffer()
    dongle.reset_input_buffer()

    out_q = queue.Queue()
    stop_evt = threading.Event()

    t_thread = threading.Thread(target=reader_thread, args=("TEENSY", teensy, out_q, stop_evt), daemon=True)
    d_thread = threading.Thread(target=reader_thread, args=("DONGLE", dongle, out_q, stop_evt), daemon=True)
    t_thread.start()
    d_thread.start()

    rows = []
    t_start = time.monotonic()

    print(f"Logging baseline for {args.settle}s before trigger...")
    while time.monotonic() - t_start < args.settle:
        try:
            rows.append(out_q.get(timeout=0.1))
        except queue.Empty:
            pass

    trigger_t = time.monotonic()
    teensy.write(args.cmd.encode())
    teensy.flush()
    rows.append((trigger_t, "TRIGGER", f"sent cmd={args.cmd!r}"))
    print(f"Sent trigger '{args.cmd}' to Teensy. Logging for up to {args.duration}s...")

    done_seen = False
    log_deadline = trigger_t + args.duration
    while time.monotonic() < log_deadline and not done_seen:
        try:
            row = out_q.get(timeout=0.1)
        except queue.Empty:
            continue
        rows.append(row)
        if row[1] == "TEENSY" and "[TRAJ] Done" in row[2]:
            done_seen = True
            print("Trajectory reported done — grabbing a short tail then stopping.")
            tail_deadline = time.monotonic() + 0.5
            while time.monotonic() < tail_deadline:
                try:
                    rows.append(out_q.get(timeout=0.1))
                except queue.Empty:
                    pass

    stop_evt.set()
    time.sleep(0.2)
    teensy.close()
    dongle.close()

    rows.sort(key=lambda r: r[0])
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_s", "source", "line"])
        for t, src, line in rows:
            w.writerow([f"{t - trigger_t:.6f}", src, line])

    print(f"Wrote {len(rows)} lines to {args.out}")
    if not done_seen:
        print("[WARN] Never saw '[TRAJ] Done' from Teensy — duration timeout hit instead. "
              "Check trajectory duration vs --duration, or a stall/e-stop occurred.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)