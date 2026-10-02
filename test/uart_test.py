import math
import struct
import time

PORT = "COM5"
BAUD = 115200
SAMPLE_RATE = 100
GRAVITY = 9.80665
REFERENCE_PRESSURE = 101.325  # kPa
HEIGHT = 20.0  # Peak height in meters
MOTION_SECONDS = 20.0
REST_SECONDS = 5.0


def crc16(data):
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


def motion_at(t):
    """Return true height, upward velocity, and upward acceleration (SI units)."""
    cycle = MOTION_SECONDS + 2 * REST_SECONDS
    elapsed = t % cycle - REST_SECONDS
    if not 0.0 < elapsed < MOTION_SECONDS:
        return 0.0, 0.0, 0.0

    # sin^4 gives a smooth rise and return, with zero velocity and
    # acceleration at both ends. Velocity changes sign at the peak.
    omega = math.pi / MOTION_SECONDS
    s = math.sin(omega * elapsed)
    c = math.cos(omega * elapsed)
    height = HEIGHT * s**4
    velocity = 4 * HEIGHT * omega * s**3 * c
    acceleration = 4 * HEIGHT * omega**2 * (3 * s**2 * c**2 - s**4)
    return height, velocity, acceleration


def make_packet(height, acceleration):
    # Level attitude; body Z points down. Include gravity in specific force.
    az = -(GRAVITY + acceleration)
    pressure = REFERENCE_PRESSURE * (1.0 - height / 44330.0)**(1.0 / 0.190295)
    payload = struct.pack("<7f", 0, 0, 0, 0, 0, az, pressure)
    body = bytes([0x05, 0x08, 0x01, 0x20, 0x00]) + payload
    return b"\xFA" + body + struct.pack(">H", crc16(body))


def main():
    import serial

    print(f"Sending to {PORT}: rest, climb to {HEIGHT:g} m, descend, repeat.")
    print("SIM lines show the sent motion; RX lines come from the MCU. Ctrl+C stops.")
    try:
        with serial.Serial(PORT, BAUD, timeout=0, write_timeout=1) as port:
            sample = 0
            deadline = time.perf_counter()
            while True:
                # Sample time stays consistent with the MCU's fixed 0.01 s dt.
                t = sample / SAMPLE_RATE
                height, velocity, acceleration = motion_at(t)
                port.write(make_packet(height, acceleration))
                incoming = port.read(port.in_waiting)
                if incoming:
                    print(incoming.decode(errors="replace"), end="", flush=True)
                if sample % SAMPLE_RATE == 0:
                    print(f"\nSIM t={t:5.1f}s h={height:6.2f}m "
                          f"v={velocity:+6.2f}m/s a={acceleration:+6.2f}m/s²")
                sample += 1
                deadline += 1.0 / SAMPLE_RATE
                delay = deadline - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    # Avoid a burst of catch-up packets after a long host pause.
                    deadline = time.perf_counter()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
