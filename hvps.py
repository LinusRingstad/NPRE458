from typing import TypedDict
from gpiozero import PWMLED, DigitalOutputDevice


hvps_schema = {
    "knobs": {
        "current": {
            "pins": {
                "pwm": 13,
                "control1": 5,
                "control2": 6
            }
        },
        "voltage": {
            "pins": {
                "pwm": 12,
                "control1": 7,
                "control2": 8
            }
        }
    }
}


class PinsConfig(TypedDict):
    pwm: int
    control1: int
    control2: int


class PowerSupplyConfig(TypedDict):
    pins: PinsConfig


class PowerSupply:
    pwm: PWMLED
    control1: DigitalOutputDevice
    control2: DigitalOutputDevice

    MAX_VALUE: float = 1.0

    def __init__(self, config: PowerSupplyConfig):
        pins = config["pins"]

        self.pwm = PWMLED(pins["pwm"])
        self.control1 = DigitalOutputDevice(pins["control1"])
        self.control2 = DigitalOutputDevice(pins["control2"])

    def output(self, value: float, control1: bool, control2: bool) -> None:

        value = min(self.MAX_VALUE, max(0.0, value))

        self.pwm.value = value
        self.control1.value = control1
        self.control2.value = control2


    def off(self) -> None:
        '''
        gpio off
        '''
        self.pwm.off()
        self.control1.off()
        self.control2.off()


voltage_ps = PowerSupply(hvps_schema["knobs"]["voltage"])
current_ps = PowerSupply(hvps_schema["knobs"]["current"])


def set_voltage(value: float, enable: bool = True) -> None:
    """
    from 0-1
    """
    if enable:
        voltage_ps.output(value, control1=True, control2=False)
    else:
        voltage_ps.off()


def set_current(value: float, enable: bool = True) -> None:
    """
    from 0-1
    """
    if enable:
        current_ps.output(value, control1=True, control2=False)
    else:
        current_ps.off()