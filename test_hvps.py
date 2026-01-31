import hvps as hvps
import time as time

def main():
    hvps.set_voltage(0.5)
    time.sleep(2)
    hvps.set_voltage(0)