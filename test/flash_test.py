import serial

PORT = "COM12"
BAUD = 115200


def main():
    pending = bytearray()

    print(f"Listening on {PORT} at {BAUD} baud.")
    print("Reset the STM32 after opening this script. Ctrl+C stops.")

    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as port:
            while True:
                incoming = port.read(port.in_waiting or 1)
                if not incoming:
                    continue

                pending.extend(incoming)

                while b"\n" in pending:
                    line, _, remaining = pending.partition(b"\n")
                    pending[:] = remaining

                    text = line.rstrip(b"\r").decode(
                        "utf-8", errors="replace"
                    )
                    print(text, flush=True)

                # Prevent unlimited growth if binary data/no newline arrives.
                if len(pending) > 4096:
                    print(f"Unframed data: {pending.hex(' ')}", flush=True)
                    pending.clear()

    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")


if __name__ == "__main__":
    main()