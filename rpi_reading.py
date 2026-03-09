import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port as needed
for i in range(10):
    data = ser.readline().decode('utf-8').strip()
    print(data)