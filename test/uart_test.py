import struct
import serial

PORT = "COM5"
BAUD = 115200

HEADER = bytes.fromhex("FA 05 08 01 20 00")
PACKET_SIZE = 36


def vn_crc16(data):
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


def main():
    buffer = bytearray()
    valid = 0
    crc_errors = 0

    print(f"Listening on {PORT} at {BAUD} baud. Ctrl+C stops.")

    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as port:
            while True:
                buffer.extend(port.read(port.in_waiting or 1))

                while True:
                    start = buffer.find(HEADER)

                    if start == -1:
                        # Preserve a possible partial header across reads.
                        if len(buffer) > len(HEADER) - 1:
                            del buffer[: -(len(HEADER) - 1)]
                        break

                    if start:
                        del buffer[:start]

                    if len(buffer) < PACKET_SIZE:
                        break

                    packet = bytes(buffer[:PACKET_SIZE])

                    calculated = vn_crc16(packet[1:34])
                    received = int.from_bytes(packet[34:36], "big")

                    if calculated != received:
                        crc_errors += 1
                        del buffer[0]  # Advance one byte and search again.
                        continue

                    del buffer[:PACKET_SIZE]
                    valid += 1

                    # Seven little-endian float32 values, starting at byte 6.
                    yaw, pitch, roll, ax, ay, az, pressure = (
                        struct.unpack_from("<7f", packet, 6)
                    )

                    print(
                        f"#{valid} "
                        f"yaw={yaw:9.3f} pitch={pitch:9.3f} "
                        f"roll={roll:9.3f} deg | "
                        f"ax={ax:8.3f} ay={ay:8.3f} az={az:8.3f} m/s² | "
                        f"P={pressure:8.3f} kPa | CRC errors={crc_errors}"
                    )

    except KeyboardInterrupt:
        print(f"\nStopped. Valid packets={valid}, CRC errors={crc_errors}")
    except serial.SerialException as error:
        print(f"\nSerial error: {error}")


if __name__ == "__main__":
    main()