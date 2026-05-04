"""
power_control.py
================
Sends voltage and current setpoints to the Arduino DAC (MCP4728) over serial.

The Arduino's MCP4728 DAC has 12-bit resolution (0–4095).
Scaling from the Arduino's calc_v() and calc_i() functions:

    voltage_set (kV) = 1490/4095 * dac_value   ⟹  dac = kV * 4095/1490
    current_set (A)  = 0.0497/4095 * dac_value ⟹  dac = A  * 4095/0.0497

Hardware limits enforced here (before sending to Arduino):
    Voltage:  0 – 1.0  kV   (1000 V)
    Current:  0 – 90   mA

Serial command format sent to Arduino (one line):
    "V<dac_v>,I<dac_i>\\n"
    e.g.  "V1800,I4065\\n"

The Arduino loop() must be updated to read this command and apply the
DAC values instead of using hardcoded numbers. Add this to the Arduino:

    // At top of loop():
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\\n');
      cmd.trim();
      if (cmd.startsWith("V")) {
        int comma = cmd.indexOf(',');
        if (comma > 0) {
          uint16_t vdac = (uint16_t)cmd.substring(1, comma).toInt();
          uint16_t idac = (uint16_t)cmd.substring(comma + 2).toInt();
          vdac = constrain(vdac, 0, 4095);
          idac = constrain(idac, 0, 4095);
          mcp.setChannelValue(MCP4728_CHANNEL_A, vdac);
          mcp.setChannelValue(MCP4728_CHANNEL_B, idac);
        }
      }
    }

Usage:
    ctrl = PowerController()
    ok, err = ctrl.start()

    ctrl.set_voltage(0.5)    # kV
    ctrl.set_current(50.0)   # mA
    ctrl.set_both(0.5, 50.0) # send in one command
    ctrl.zero()              # safe: set both to 0

    ctrl.stop()
"""

import threading
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

# DAC full scale
DAC_MAX = 4095

# Physical limits
MAX_VOLTAGE_KV = 1.0    # kV
MAX_CURRENT_MA = 90.0   # mA

# Arduino scaling (from calc_v and calc_i in sketch)
# voltage_kv = 1490/4095 * dac  →  dac = voltage_kv * 4095/1490
VOLTAGE_KV_TO_DAC = 4095.0 / 1.490      # 1490 V max → counts/kV
# current_A  = 0.0497/4095 * dac →  dac = current_A * 4095/0.0497
CURRENT_A_TO_DAC  = 4095.0 / 0.0497     # counts/A


def _kv_to_dac(kv: float) -> int:
    """Clamp and convert kV setpoint to 12-bit DAC value."""
    kv  = max(0.0, min(MAX_VOLTAGE_KV, kv))
    return int(round(kv * VOLTAGE_KV_TO_DAC))


def _ma_to_dac(ma: float) -> int:
    """Clamp and convert mA setpoint to 12-bit DAC value."""
    ma  = max(0.0, min(MAX_CURRENT_MA, ma))
    a   = ma / 1000.0
    return int(round(a * CURRENT_A_TO_DAC))


class PowerController:
    """
    Sends DAC setpoints to the Arduino over the shared serial port.

    Thread-safe: set_voltage / set_current / set_both can be called from
    any thread (including the GUI main thread).

    NOTE: This opens the same /dev/ttyACM0 as ArduinoReader. Because both
    read and write over the same USB-serial link, you must use the SAME
    serial.Serial object for both. Pass the shared `ser` object via
    from_serial() rather than calling start() independently.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud

        self._ser     = None
        self._lock    = threading.Lock()
        self._running = False

        # Last sent setpoints (kV, mA)
        self._v_kv = 0.0
        self._i_ma = 0.0

    # ------------------------------------------------------------------
    def start(self):
        """
        Open serial port independently (use only if ArduinoReader is NOT running).
        Returns (True, None) or (False, error_string).
        """
        if not SERIAL_AVAILABLE:
            return False, "pyserial not installed.  pip install pyserial"

        try:
            self._ser     = serial.Serial(self.port, self.baud, timeout=1)
            self._running = True
            time.sleep(2.0)   # let Arduino reset after DTR toggle
            self._ser.reset_input_buffer()
            # Always start at zero for safety
            self._send(0, 0)
            return True, None
        except Exception as exc:
            return False, str(exc)

    def attach_serial(self, ser):
        """
        Attach to an already-open serial.Serial object (shared with ArduinoReader).
        Call this instead of start() when sharing the port.
        """
        self._ser     = ser
        self._running = True
        self._send(0, 0)   # safe zero on attach

    def stop(self):
        """Zero outputs and release port (only if we own it)."""
        if self._running:
            try:
                self._send(0, 0)
            except Exception:
                pass
        self._running = False

    # ------------------------------------------------------------------
    def set_voltage(self, kv: float):
        """Set voltage setpoint (kV). Clamped to 0–1.0 kV."""
        self._v_kv = max(0.0, min(MAX_VOLTAGE_KV, kv))
        self._send(_kv_to_dac(self._v_kv), _ma_to_dac(self._i_ma))

    def set_current(self, ma: float):
        """Set current setpoint (mA). Clamped to 0–90 mA."""
        self._i_ma = max(0.0, min(MAX_CURRENT_MA, ma))
        self._send(_kv_to_dac(self._v_kv), _ma_to_dac(self._i_ma))

    def set_both(self, kv: float, ma: float):
        """Set both voltage (kV) and current (mA) in a single serial write."""
        self._v_kv = max(0.0, min(MAX_VOLTAGE_KV, kv))
        self._i_ma = max(0.0, min(MAX_CURRENT_MA, ma))
        self._send(_kv_to_dac(self._v_kv), _ma_to_dac(self._i_ma))

    def zero(self):
        """Immediately set both outputs to 0 (safe shutdown)."""
        self._v_kv = 0.0
        self._i_ma = 0.0
        self._send(0, 0)

    @property
    def voltage_kv(self) -> float:
        return self._v_kv

    @property
    def current_ma(self) -> float:
        return self._i_ma

    # ------------------------------------------------------------------
    def _send(self, dac_v: int, dac_i: int):
        """Write the DAC command to serial. Thread-safe."""
        if self._ser is None or not self._running:
            return
        cmd = f"V{dac_v},I{dac_i}\n".encode("ascii")
        with self._lock:
            try:
                self._ser.write(cmd)
            except Exception as exc:
                print(f"[PowerController] Serial write error: {exc}")
