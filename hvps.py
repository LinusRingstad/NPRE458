from gpiozero import PWMLED
import RPi.GPIO as GPIO
from typing import TypedDict

# JSON snippet
vehicle = {
    "motors": {
        "left": {
            "pins": {
                "speed": 13,
                "control1": 5,
                "control2": 6
            }
        },
        "right": {
            "pins": {
                "speed": 12,
                "control1": 7,
                "control2": 8
            }
        }
    }
}

# TypedDicts for configuration
class PinsConfig(TypedDict):
    speed: int
    control1: int
    control2: int

class Config(TypedDict):
    pins: PinsConfig

# LED class
class LED:
    pin: int

    def __init__(self, config: Config):
        self.pin = config['pin']
        GPIO.setup(self.pin, GPIO.OUT)

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)

# Motor class
class Motor:
    pwm: PWMLED
    control1: LED
    control2: LED

    MAX_SPEED: float = 1.0

    def __init__(self, config: Config):
        self.pwm = PWMLED(config['pins']['speed'])
        self.control1 = LED({'pin': config['pins']['control1']})
        self.control2 = LED({'pin': config['pins']['control2']})

    def stop(self) -> None:
        """Stop the motor"""
        self._output(0.0, True, True)

    def forward(self, speed: float) -> None:
        """Spin the motor forward at a given speed"""
        self.drive(speed, True)

    def backward(self, speed: float) -> None:
        """Spin the motor backward at a given speed"""
        self.drive(speed, False)

    def drive(self, speed: float, direction: bool) -> None:
        """Spin the motor at a given speed and direction

        speed: float in [0, 1]
        direction: True for forward, False for backwards
        """
        self._output(speed, direction, not direction)

    def _output(self, speed: float, control1: bool, control2: bool) -> None:
        """Set the motor's speed, and send out the output signals"""
        # bound the speed to [MIN_SPEED, MAX_SPEED]
        speed = min(Motor.MAX_SPEED, max(0, speed))

        # set speed and output values
        self.pwm.value = speed
        self.control1.on() if control1 else self.control1.off()
        self.control2.on() if control2 else self.control2.off()

# Initialize motors using the JSON configuration
left_motor_config = vehicle["motors"]["left"]
right_motor_config = vehicle["motors"]["right"]

left_motor = Motor(left_motor_config)
right_motor = Motor(right_motor_config)

# Example usage
left_motor.forward(0.8)  # Spin left motor forward at 80% speed
right_motor.backward(0.5)  # Spin right motor backward at 50% speed