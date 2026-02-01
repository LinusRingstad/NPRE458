import numpy as np
import hvps as motor_module
import time
import RPi.GPIO

if __name__ == '__main__':
    RPi.GPIO.setmode(RPi.GPIO.BCM)

    motor1 = motor_module.Motor({
        "pins": {
            "speed": 13,
            "control1": 5,
            "control2": 6
        }
    })

    motor2 = motor_module.Motor({
        "pins": {
            "speed": 12,
            "control1": 7,
            "control2": 8
        }
    })

    speeds = list(np.linspace(0, 1, 10)) + list(np.linspace(1, 0, 10))
    dt = 5
    motor1.stop()
    motor2.stop()
    time.sleep(dt)

    for speed in speeds:
        print('Motor backward at {}% speed'.format(speed * 100))
        motor1.backward(speed)
        motor2.backward(speed)
        time.sleep(dt)

    motor1.stop()
    motor2.stop()