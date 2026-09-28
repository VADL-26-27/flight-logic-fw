import serial

ser = serial.Serial('/dev/tty.usbmodem103', 115200)   # connecting to our port with the baud rate of the board
ser.write(b'A' * 50)                  # send exactly 34 bytes
ser.close()
print("Sent 50 bytes")