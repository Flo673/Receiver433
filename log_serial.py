#!/usr/bin/env python3
"""Read RSSI-test receiver CSV lines from the Feather's serial port and log
them with a PC-side timestamp to a CSV file.

Two transmitters share this receiver (board_id "4" = Transmitter433 bare
RSSI-test transmitter, board_id "5" = Transmitter433wBno with a BNO055
orientation sample added), each with its own set of columns, so rows are
routed to separate per-board files. Each connection (the initial one, and any
reconnection after a serial hiccup) gets its own timestamped session folder
under "<out-basename>_sessions/", so a dropped connection never appends into
a file from before the drop -- e.g. --out rssi_log.csv produces
rssi_log_sessions/20260831_142305/rssi_log_board4.csv. Any line that isn't a
CSV row for board 4 or 5 (startup/status messages, or board 5's Orientation/
Quaternion/Calibration lines for the WebSerial viewer) is still logged, with
a PC-side timestamp, to rssi_log_other.csv in the same session folder.

Meant to run unattended for hours: a serial error (USB unplugged, board
reset, port reassigned) is caught and retried indefinitely rather than
crashing the script. Only Ctrl+C stops it.

Press the spacebar to start a sample (Windows console only); press it again
to stop. Only the first row received after start and the last row received
before stop are stamped with the sample number -- everything logged in
between is left blank.

Usage:
    python log_serial.py [--port COM5] [--baud 115200] [--out log.csv]

If --port is omitted, the script auto-detects an Adafruit Feather (USB VID
0x239A) among the connected serial ports, falling back to the only port
present if there's just one, and re-detects it the same way on reconnect.
"""

import argparse
import csv
import datetime
import os
import sys
import time

import serial
import serial.tools.list_ports

try:
    import msvcrt
except ImportError:
    msvcrt = None

ADAFRUIT_USB_VID = 0x239A
RECONNECT_DELAY_S = 3
# Packets arrive ~once/second per board; this much total silence means the
# connection dropped, even if the OS never raised a read error for it (a
# common case on Windows when a USB CDC device is unplugged or resets).
INACTIVITY_TIMEOUT_S = 30

# Column names for each board's payload, in wire order, including the leading
# board_id field. Field count (including board_id) must match what the
# receiver firmware prints for that board. "sample" is appended by this script,
# not sent by the firmware -- see poll_space_toggle() / the sampler state below.
BOARD_COLUMNS = {
    "4": [
        "board_id", "packet_count", "battery_voltage", "rssi", "snr",
        "rssi_datasheet", "noise_floor", "sample",
    ],
    "5": [
        "board_id", "packet_count", "battery_voltage", "rssi", "snr",
        "rssi_datasheet", "noise_floor",
        "heading", "pitch", "roll", "qw", "qx", "qy", "qz",
        "cal_sys", "cal_gyro", "cal_accel", "cal_mag", "sample",
    ],
}


def poll_space_toggle():
    """Non-blocking: returns True if the spacebar was pressed since the last call."""
    if msvcrt is None:
        return False
    toggled = False
    while msvcrt.kbhit():
        if msvcrt.getch() == b" ":
            toggled = True
    return toggled


def find_port():
    """Return a single candidate serial port device name, or None if none/ambiguous."""
    ports = list(serial.tools.list_ports.comports())
    feather_ports = [p for p in ports if p.vid == ADAFRUIT_USB_VID]
    if len(feather_ports) == 1:
        return feather_ports[0].device
    if len(feather_ports) == 0 and len(ports) == 1:
        return ports[0].device
    return None


def autodetect_port():
    port = find_port()
    if port is not None:
        return port

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Plug in the receiver and/or pass --port explicitly.")
    names = ", ".join(p.device for p in ports)
    sys.exit(f"Could not auto-detect the receiver; pass --port explicitly. Available ports: {names}")


def wait_for_serial(explicit_port, baud):
    """Blocks until the serial port can be opened, retrying indefinitely."""
    while True:
        port = explicit_port or find_port()
        if port:
            try:
                return serial.Serial(port, baud, timeout=1), port
            except serial.SerialException as e:
                print(f"Failed to open {port} ({e}); retrying in {RECONNECT_DELAY_S}s...")
        else:
            print(f"Waiting for receiver to appear; retrying in {RECONNECT_DELAY_S}s...")
        time.sleep(RECONNECT_DELAY_S)


def out_path_for_board(base_path, board_id):
    return f"{os.path.splitext(os.path.basename(base_path))[0]}_board{board_id}{os.path.splitext(base_path)[1]}"


def out_path_for_other(base_path):
    return f"{os.path.splitext(os.path.basename(base_path))[0]}_other.csv"


def new_session_dir(base_out):
    root_dir = os.path.dirname(os.path.abspath(base_out)) or "."
    base_name = os.path.splitext(os.path.basename(base_out))[0]
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(root_dir, f"{base_name}_sessions", timestamp)
    os.makedirs(session_dir, exist_ok=True)
    return session_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM5 (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="rssi_log.csv", help="Output CSV path template")
    args = parser.parse_args()

    ser, port = wait_for_serial(args.port, args.baud)
    session_dir = new_session_dir(args.out)
    print(f"Session started -> {session_dir}")

    open_files = {}
    writers = {}
    pending_row = {board_id: None for board_id in BOARD_COLUMNS}
    just_started = {board_id: False for board_id in BOARD_COLUMNS}
    sample_number = 0
    sample_active = False

    def flush_pending(board_id):
        row = pending_row[board_id]
        if row is not None and board_id in writers:
            writers[board_id].writerow(row)
            open_files[board_id].flush()
        pending_row[board_id] = None

    def get_writer(board_id):
        if board_id not in writers:
            path = os.path.join(session_dir, out_path_for_board(args.out, board_id))
            f = open(path, "a", newline="")
            writer = csv.writer(f)
            writer.writerow(["timestamp"] + BOARD_COLUMNS[board_id])
            f.flush()
            open_files[board_id] = f
            writers[board_id] = writer
            print(f"Logging board {board_id} -> {path}")
        return writers[board_id]

    def get_other_writer():
        if "other" not in writers:
            path = os.path.join(session_dir, out_path_for_other(args.out))
            f = open(path, "a", newline="")
            writer = csv.writer(f)
            writer.writerow(["timestamp", "line"])
            f.flush()
            open_files["other"] = f
            writers["other"] = writer
            print(f"Logging unmatched lines -> {path}")
        return writers["other"]

    def log_other(line):
        writer = get_other_writer()
        writer.writerow([datetime.datetime.now().isoformat(timespec="seconds"), line])
        open_files["other"].flush()

    def close_all_files():
        for f in open_files.values():
            f.close()
        open_files.clear()
        writers.clear()

    def reconnect(old_ser):
        for board_id in BOARD_COLUMNS:
            flush_pending(board_id)  # don't lose a still-buffered row to a dropped connection
        try:
            old_ser.close()
        except serial.SerialException:
            pass
        close_all_files()
        new_ser, new_port = wait_for_serial(args.port, args.baud)
        new_session_dir_path = new_session_dir(args.out)
        print(f"Reconnected on {new_port} -> new session {new_session_dir_path}")
        return new_ser, new_port, new_session_dir_path

    if msvcrt is None:
        print("Spacebar sampling isn't supported outside a Windows console; logging continues as normal.")
    else:
        print("Press spacebar to start a sample, press it again to stop.")

    print(f"Reading {port} (Ctrl+C to stop)")
    last_activity = time.monotonic()
    try:
        while True:
            if poll_space_toggle():
                sample_active = not sample_active
                if sample_active:
                    sample_number += 1
                    just_started = {board_id: True for board_id in BOARD_COLUMNS}
                    print(f"\nSample {sample_number} started.")
                else:
                    for board_id in BOARD_COLUMNS:
                        row = pending_row[board_id]
                        if row is not None and row[-1] == "":
                            row[-1] = sample_number
                        flush_pending(board_id)
                    print(f"\nSample {sample_number} stopped.")

            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as e:
                print(f"\nSerial error ({e}); reconnecting...")
                ser, port, session_dir = reconnect(ser)
                last_activity = time.monotonic()
                continue

            if not line:
                if time.monotonic() - last_activity > INACTIVITY_TIMEOUT_S:
                    print(f"\nNo data for {INACTIVITY_TIMEOUT_S}s; assuming the connection dropped, reconnecting...")
                    ser, port, session_dir = reconnect(ser)
                    last_activity = time.monotonic()
                continue

            last_activity = time.monotonic()
            fields = line.split(",")
            board_id = fields[0]
            expected_columns = BOARD_COLUMNS.get(board_id)
            if expected_columns is None or len(fields) != len(expected_columns) - 1:
                # Startup/status messages, orientation/quaternion/calibration
                # lines for the WebSerial viewer, or an unrecognized board_id --
                # not a CSV row for either board, so it goes to the catch-all
                # log instead of a per-board file. (-1 excludes the "sample"
                # column, which this script appends, not the firmware.)
                log_other(line)
                print(line)
                continue

            get_writer(board_id)
            timestamp = datetime.datetime.now().isoformat(timespec="seconds")

            sample_value = ""
            if sample_active and just_started[board_id]:
                sample_value = sample_number
                just_started[board_id] = False

            row = [timestamp, *fields, sample_value]
            flush_pending(board_id)  # the previously held-back row is confirmed not the sample's last row
            if sample_active:
                pending_row[board_id] = row  # hold back in case this turns out to be the sample's last row
            else:
                writers[board_id].writerow(row)
                open_files[board_id].flush()
            print(*row)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for board_id in BOARD_COLUMNS:
            flush_pending(board_id)
        try:
            ser.close()
        except serial.SerialException:
            pass
        close_all_files()


if __name__ == "__main__":
    sys.exit(main())
