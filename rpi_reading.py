import serial
import time

port = '/dev/ttyUSB0' 
baud = 115200

try:
    ser = serial.Serial(port, baud, timeout=1)
    
    # wait
    time.sleep(2) 
    
    # FLUSH the buffer
    ser.reset_input_buffer()
    print("Buffer cleared. Listening for I2C Scan...")

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Result: {line}")

except KeyboardInterrupt:
    print("\nClosing...")
    ser.close()
except Exception as e:
    print(f"Error: {e}")