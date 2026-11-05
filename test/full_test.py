"""Monitor flash-test text and VectorNav IMU packets on ONE UART stream.

Run: python combined_flash_imu_test.py --port COM12 --baud 115200
Test parser without serial hardware: python combined_flash_imu_test.py --self-test

Firmware must enqueue each complete IMU packet/text message without interleaving
their bytes. This only observes firmware reports; it does not command erases or
independently prove that flash storage works. Firmware supplies readback checking.
"""
import argparse
import math
import struct
import time

HEADER = bytes.fromhex("fa 05 08 01 20 00")
PACKET_SIZE = 36


def crc16(data):
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


class MixedParser:
    def __init__(self):
        self.pending = bytearray()
        self.line = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.long_lines = 0

    def feed(self, incoming):
        """Return ('imu', raw_packet) and ('text', line) events in stream order."""
        self.pending.extend(incoming)
        events = []
        pos = 0
        while pos < len(self.pending):
            if self.pending[pos] == HEADER[0]:
                available = len(self.pending) - pos
                n = min(available, len(HEADER))
                if self.pending[pos:pos + n] == HEADER[:n]:
                    # Retain partial headers AND partial packets across reads.
                    if available < PACKET_SIZE:
                        break
                    packet = bytes(self.pending[pos:pos + PACKET_SIZE])
                    if crc16(packet[1:34]) == int.from_bytes(packet[34:36], "big"):
                        events.append(("imu", packet))
                        pos += PACKET_SIZE
                        continue
                    self.crc_errors += 1
                    # Advance one byte, allowing resync within a damaged frame.

            byte = self.pending[pos]
            pos += 1
            if byte == 10:
                if self.line:
                    events.append(("text", self.line.decode("ascii")))
                    self.line.clear()
            elif byte == 13:
                pass
            elif byte == 9 or 32 <= byte <= 126:
                self.line.append(byte)
                if len(self.line) > 4096:
                    self.discarded_bytes += len(self.line)
                    self.long_lines += 1
                    self.line.clear()
            else:
                self.discarded_bytes += 1 + len(self.line)
                self.line.clear()

        del self.pending[:pos]
        return events


def self_test():
    def packet(values):
        body = HEADER + struct.pack("<7f", *values)
        return body + crc16(body[1:]).to_bytes(2, "big")

    # Include LF (0x0A) and FA bytes inside a binary payload.
    tricky = struct.unpack("<f", b"\x0a\xfa\x80\x3f")[0]
    p1 = packet((1, 2, 3, tricky, 5, 6, 101.3))
    p2 = packet((7, 8, 9, 10, 11, 12, 99.9))
    stream = b"Probe OK\r\n" + p1 + b"Hello world! 3\nTest Passed!\n" + p2
    expected = [("text", "Probe OK"), ("imu", p1),
                ("text", "Hello world! 3"), ("text", "Test Passed!"), ("imu", p2)]
    # Every possible split tests partial headers, payloads, CRC, and text lines.
    for split in range(len(stream) + 1):
        parser = MixedParser()
        assert parser.feed(stream[:split]) + parser.feed(stream[split:]) == expected
    for chunk_size in (1, 2, 3, 7, 36, 64):
        parser = MixedParser()
        got = []
        for i in range(0, len(stream), chunk_size):
            got.extend(parser.feed(stream[i:i + chunk_size]))
        assert got == expected
    damaged = bytearray(p1)
    damaged[12] ^= 0x40
    parser = MixedParser()
    events = parser.feed(damaged + b"\x00" + p2 + b"Test Failed!\n")
    assert ("imu", p2) in events and ("imu", p1) not in events
    assert ("text", "Test Failed!") in events and parser.crc_errors >= 1
    parser = MixedParser()
    parser.feed(b"x" * 10000)
    assert len(parser.line) <= 4096 and len(parser.pending) < PACKET_SIZE
    print("PASS: mixed text/binary, all split positions, bytewise reads, CRC resync, bounded buffering")


def monitor(args):
    try:
        import serial
    except ImportError:
        print("Install pyserial first: python -m pip install pyserial")
        return 1

    parser = MixedParser()
    received = packets = passes = failures = 0
    recent = bytearray()
    started = last_status = time.monotonic()
    last_imu_display = float("-inf")
    last_imu = last_text = None
    print(f"Listening on {args.port} at {args.baud} baud. Ctrl+C stops.")
    print("Open this monitor before resetting the STM32.")
    print("[IMU] yaw pitch roll (deg), acceleration (m/s^2), pressure (kPa)")
    print("[TEXT] firmware flash-test messages; [STATUS] reception totals")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as port:
            while True:
                incoming = port.read(min(port.in_waiting or 1, 4096))
                now = time.monotonic()
                if incoming:
                    received += len(incoming)
                    recent.extend(incoming)
                    del recent[:-64]
                    for kind, value in parser.feed(incoming):
                        if kind == "text":
                            last_text = now
                            print(f"[{now - started:8.2f}s] [TEXT] {value}", flush=True)
                            normalized = value.strip().lower()
                            if normalized == "test passed!":
                                passes += 1
                            elif "fail" in normalized:
                                failures += 1
                        else:
                            packets += 1
                            last_imu = now
                            # Parse/count every packet; throttle only console output.
                            if args.imu_hz == 0 or now - last_imu_display >= 1.0 / args.imu_hz:
                                last_imu_display = now
                                yaw, pitch, roll, ax, ay, az, pressure = struct.unpack_from("<7f", value, 6)
                                print(f"[{now - started:8.2f}s] [IMU] "
                                      f"YPR {yaw:8.2f} {pitch:8.2f} {roll:8.2f} | "
                                      f"A {ax:8.3f} {ay:8.3f} {az:8.3f} | "
                                      f"P {pressure:9.3f}", flush=True)
                if now - last_status >= 3.0:
                    imu_age = "never" if last_imu is None else f"{now - last_imu:.1f}s ago"
                    text_age = "never" if last_text is None else f"{now - last_text:.1f}s ago"
                    print(f"[STATUS] RX={received} bytes | IMU={packets} "
                          f"(last {imu_age}) | flash PASS={passes} | failure messages={failures} | "
                          f"CRC errors={parser.crc_errors} | last text {text_age}", flush=True)
                    if received == 0:
                        print("[STATUS] No bytes received: check port, wiring, and firmware output.", flush=True)
                    elif packets == 0:
                        print(f"[STATUS] No valid IMU packets yet. Recent HEX: {recent.hex(' ')}", flush=True)
                    last_status = now
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    finally:
        print(f"Totals: {packets} valid IMU packets; {passes} flash pass reports; "
              f"{failures} failure messages; {parser.crc_errors} CRC errors.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--imu-hz", type=float, default=10.0,
                    help="maximum IMU console updates per second; 0 prints every packet")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if not math.isfinite(args.imu_hz) or args.imu_hz < 0:
        ap.error("--imu-hz must be finite and >= 0")
    if args.self_test:
        self_test()
        return 0
    return monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
