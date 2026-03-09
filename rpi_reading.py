import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port as needed
while True:
    data = ser.readline().decode('utf-8').strip()
    print(data)