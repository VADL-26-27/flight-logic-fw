PORT = "COM5"
BAUD = 115200
PACKET_SIZE = 36
HEADER = bytes.fromhex("fa 05 08 01 20 00")


def crc16(data):
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


def extract_packets(pending):
    """Consume valid packets; retain incomplete data for the next read."""
    while pending:
        start = pending.find(HEADER)
        if start < 0:
            # A read can end in the middle of the header.
            if len(pending) >= len(HEADER):
                del pending[:len(pending) - len(HEADER) + 1]
            return
        if start:
            del pending[:start]
        if len(pending) < PACKET_SIZE:
            return
        packet = bytes(pending[:PACKET_SIZE])
        if crc16(packet[1:34]) != int.from_bytes(packet[34:36], "big"):
            # Recover one byte at a time, including sync bytes in a bad frame.
            del pending[0]
            continue
        del pending[:PACKET_SIZE]
        yield packet


def main():
    import serial
    import time

    print(f"Listening on {PORT} at {BAUD} baud. Ctrl+C stops.")
    print("Printing complete, CRC-validated 36-byte IMU packets.")
    pending = bytearray()
    received_bytes = 0
    valid_packets = 0
    last_status = time.monotonic()
    recent = bytearray()
    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as port:
            while True:
                incoming = port.read(port.in_waiting or 1)
                if incoming:
                    received_bytes += len(incoming)
                    recent.extend(incoming)
                    del recent[:-36]
                    pending.extend(incoming)
                    for packet in extract_packets(pending):
                        valid_packets += 1
                        print(packet.hex(" "), flush=True)
                now = time.monotonic()
                if now - last_status >= 3:
                    if received_bytes == 0:
                        print("No serial bytes received. With the Teensy powered, reset the STM32 to send 100.", flush=True)
                    elif valid_packets == 0:
                        print(f"Received {received_bytes} bytes, but no valid packets. Recent HEX: {recent.hex(' ')}", flush=True)
                    else:
                        print(f"RX total: {received_bytes} bytes, {valid_packets} valid packets.", flush=True)
                    last_status = now
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
