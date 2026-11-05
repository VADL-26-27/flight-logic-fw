"""Download a LOG1 flight over USART2 or decode a saved dump to CSV."""

import argparse
import csv
from pathlib import Path
import struct
import sys
import time
import zlib


SECTOR_SIZE = 4096
LOG_START = 0x1000
LOG_SECTORS = (32 * 1024 * 1024 - LOG_START) // SECTOR_SIZE
HEADER = struct.Struct("<4s5I")
RECORD = struct.Struct("<I10fI")
CRC = struct.Struct("<I")
FIELDS = (
    "timestamp_ms", "yaw", "pitch", "roll", "ax", "ay", "az", "pressure",
    "altitude", "velocity", "acceleration", "flight_state", "phase",
)
STATES = ("STANDBY", "LAUNCH", "DESCENT", "DEPLOYMENT", "LANDED")


def receive_dump(port, output):
    port.reset_input_buffer()
    port.write(b"LOG DUMP\n")
    banner = b"\r\nLOG DUMP 1\r\n"
    refused = b"\r\nLOG DUMP REFUSED\r\n"
    tail = b""
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        tail = (tail + port.read(1))[-len(refused):]
        if tail.endswith(refused):
            raise ValueError("board refused LOG DUMP; check LOG STATUS on the ground")
        if tail.endswith(banner):
            break
    else:
        raise ValueError("timed out waiting for LOG DUMP response")

    sectors = 0
    while True:
        tag = port.read(1)
        if tag == b"E":
            return sectors
        if tag == b"X":
            raise ValueError("board reported a flash read error")
        if tag != b"D" or sectors == LOG_SECTORS:
            raise ValueError(f"invalid or missing frame tag after {sectors} sectors")
        data = port.read(SECTOR_SIZE)
        if len(data) != SECTOR_SIZE:
            raise ValueError(f"serial timeout in sector {sectors}")
        output.write(data)
        sectors += 1


def parse_dump(source, output):
    """Write the valid prefix to CSV and return (records, dropped).

    A missing or invalid footer raises ValueError after writing that prefix.
    """
    writer = csv.writer(output)
    writer.writerow(FIELDS)
    sequence = ground_records = total_records = 0
    expected_ground = None
    seen_flight = False

    while sector := source.read(SECTOR_SIZE):
        if len(sector) != SECTOR_SIZE:
            raise ValueError(f"truncated sector {sequence}")
        if sector == b"\xff" * SECTOR_SIZE:
            break
        magic, kind, flight_id, index, count, info = HEADER.unpack_from(sector)
        if magic != b"LOG1" or flight_id != LOG_START or index != sequence:
            raise ValueError(f"invalid header or sequence in sector {sequence}")
        if kind not in (1, 2, 3, 4) or (sequence == 0) != (kind == 1):
            raise ValueError(f"unexpected block type in sector {sequence}")

        data_block = kind in (2, 3)
        if data_block and not 1 <= count <= 80:
            raise ValueError(f"invalid record count in sector {sequence}")
        payload_end = HEADER.size + (count * RECORD.size if data_block else 0)
        expected_crc, = CRC.unpack_from(sector, payload_end)
        if zlib.crc32(sector[:payload_end]) != expected_crc:
            raise ValueError(f"CRC mismatch in sector {sequence}")

        if kind == 1:
            if count != 0 or info > 3000:
                raise ValueError("invalid prelaunch record count")
            expected_ground = info
        elif kind == 4:
            if count != total_records or ground_records != expected_ground:
                raise ValueError("footer record count does not match the log")
            return total_records, info
        else:
            if kind == 2:
                if seen_flight or ground_records + count > expected_ground:
                    raise ValueError(f"unexpected ground records in sector {sequence}")
                ground_records += count
            else:
                if ground_records != expected_ground:
                    raise ValueError("flight data starts before all ground records")
                seen_flight = True
            phase = "ground" if kind == 2 else "flight"
            records = list(RECORD.iter_unpack(sector[HEADER.size:payload_end]))
            if any(row[-1] >= len(STATES) for row in records):
                raise ValueError(f"invalid flight state in sector {sequence}")
            for *values, state in records:
                writer.writerow((*values, STATES[state], phase))
            total_records += count
        sequence += 1

    raise ValueError(f"no valid completion footer; recovered {total_records} records")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    download = commands.add_parser("download", help="save the raw serial dump")
    download.add_argument("port", help="serial port, e.g. COM12")
    download.add_argument("output", type=Path, help="new .bin output file")
    parse = commands.add_parser("parse", help="decode and verify a saved dump")
    parse.add_argument("input", type=Path)
    parse.add_argument("output", type=Path, help="new .csv output file")
    args = parser.parse_args()

    try:
        if args.command == "download":
            try:
                import serial
            except ImportError:
                parser.error("download requires pyserial (python -m pip install pyserial)")
            with serial.Serial(args.port, 115200, timeout=1.5, write_timeout=2) as port:
                with args.output.open("xb") as output:
                    sectors = receive_dump(port, output)
            print(f"Saved {sectors} sectors to {args.output}. Run parse to verify the log.")
        else:
            with args.input.open("rb") as source, args.output.open(
                "x", encoding="utf-8", newline=""
            ) as output:
                count, dropped = parse_dump(source, output)
            print(f"Verified {count} records. Firmware reports {dropped} dropped.")
    except (OSError, ValueError) as error:
        print(f"{error}. Any output already written was preserved.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
