import RPi.GPIO as GPIO
from typing import TypedDict, Optional
from gpiozero import PWMLED

# ensure GPIO mode (repository sets this in src/__init__.py, but set here for standalone use)
GPIO.setmode(GPIO.BCM)


# TypedDicts for configuration
class PinsConfig(TypedDict):
    speed: int
    control1: int
    control2: int


class MotorConfig(TypedDict):
    pins: PinsConfig


class LEDConfig(TypedDict):
    pin: int


# LED class (simple wrapper around RPi.GPIO)
class LED:
    pin: int

    def __init__(self, config: LEDConfig):
        self.pin = config['pin']
        GPIO.setup(self.pin, GPIO.OUT)

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)


# Motor class (uses gpiozero.PWMLED for PWM speed)
class Motor:
    pwm: PWMLED
    control1: LED
    control2: LED

    MAX_SPEED: float = 1.0

    def __init__(self, config: MotorConfig):
        self.pwm = PWMLED(config['pins']['speed'])
        self.control1 = LED({'pin': config['pins']['control1']})
        self.control2 = LED({'pin': config['pins']['control2']})

    def stop(self) -> None:
        """Stop the motor"""
        self._output(0.0, True, True)

    def forward(self, speed: float) -> None:
        """Spin the motor forward at a given speed (0.0 - 1.0)"""
        self.drive(speed, True)

    def backward(self, speed: float) -> None:
        """Spin the motor backward at a given speed (0.0 - 1.0)"""
        self.drive(speed, False)

    def drive(self, speed: float, direction: bool) -> None:
        """Spin the motor at a given speed and direction

        speed: float in [0, 1]
        direction: True for forward, False for backwards
        """
        self._output(speed, direction, not direction)

    def _output(self, speed: float, control1: bool, control2: bool) -> None:
        """Set the motor's speed, and send out the output signals"""

        # bound the speed to [0, MAX_SPEED]
        speed = min(Motor.MAX_SPEED, max(0.0, speed))

        # set speed and output values
        self.pwm.value = speed
        if control1:
            self.control1.on()
        else:
            self.control1.off()

        if control2:
            self.control2.on()
        else:
            self.control2.off()


# Module-level motor instances (initialized by init_motors)
_left_motor: Optional[Motor] = None
_right_motor: Optional[Motor] = None


def init_motors(vehicle_config: dict) -> tuple[Motor, Motor]:
    """
    Initialize left and right Motor instances from vehicle config.
    vehicle_config is expected to be the dict at config['vehicle'] containing:
      - 'motors': {'left': { 'pins': {...} }, 'right': { 'pins': {...} } }

    Returns (left_motor, right_motor)
    """
    global _left_motor, _right_motor

    left_cfg = vehicle_config['motors']['left']
    right_cfg = vehicle_config['motors']['right']

    _left_motor = Motor(left_cfg)
    _right_motor = Motor(right_cfg)

    return _left_motor, _right_motor


def control_left(speed: float, forward: bool = True) -> None:
    """
    Control the left motor independently.

    speed: float in [0.0, 1.0]. If 0.0, motor will be stopped.
    forward: True to move forward, False to move backward.
    """
    if _left_motor is None:
        raise RuntimeError("Left motor not initialized. Call init_motors() first.")
    if speed <= 0.0:
        _left_motor.stop()
    else:
        if forward:
            _left_motor.forward(speed)
        else:
            _left_motor.backward(speed)


def control_right(speed: float, forward: bool = True) -> None:
    """
    Control the right motor independently.

    speed: float in [0.0, 1.0]. If 0.0, motor will be stopped.
    forward: True to move forward, False to move backward.
    """
    if _right_motor is None:
        raise RuntimeError("Right motor not initialized. Call init_motors() first.")
    if speed <= 0.0:
        _right_motor.stop()
    else:
        if forward:
            _right_motor.forward(speed)
        else:
            _right_motor.backward(speed)


# Example usage when running this file directly:
if __name__ == "__main__":
    # Example vehicle dict (structure should match your config.json)
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

    # initialize motors
    init_motors(vehicle)

    # spin left forward at 80% and right backward at 50%
    control_left(0.8, forward=True)
    control_right(0.5, forward=False)