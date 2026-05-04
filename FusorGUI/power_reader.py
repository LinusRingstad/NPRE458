"""
power_reader.py
===============
Reads the voltage and current ADC readings from the Arduino serial stream
and converts them to physical units.

Serial format (Arduino → Pi):
    voltage_set, voltageADC, current_set, currentADC, pressure

Calibration observations:
    voltageADC:  5 counts  → 0.46 kV   ⟹  scale = 0.46/5  = 0.092  kV/count
    currentADC: 17 counts  → 49.4 mA   ⟹  scale = 49.4/17 ≈ 2.906  mA/count

Power (W) = voltage_read (V) × current_read (A)

Usage:
    reader = PowerReader()
    ok, err = reader.start()
    data = reader.get_data()
    # data: { "voltage_kv": float, "current_ma": float, "power_w": float }
    # or None if no data yet
    reader.stop()
"""

import threading
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_PORT  = "/dev/ttyACM0"
DEFAULT_BAUD  = 115200
STARTUP_DELAY = 2.0
READ_TIMEOUT  = 1.0
STALE_TIMEOUT = 5.0

# ADC scaling factors derived from calibration observations
VOLTAGE_SCALE_KV_PER_COUNT = 0.46 / 5.0     # kV per ADC count
CURRENT_SCALE_MA_PER_COUNT = 49.4 / 17.0    # mA per ADC count


class PowerReader:
    """
    Thread-safe reader for measured voltage, current, and computed power.
    Shares the same Arduino serial stream as PressureReader — use only one
    serial connection and distribute the parsed fields (see ArduinoReader
    below) or run this standalone if pressure is read separately.

    NOTE: If PressureReader and PowerReader both open /dev/ttyACM0 they will
    conflict. Use the combined ArduinoReader class at the bottom of this file
    instead, which parses all fields in one thread and exposes separate getters.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port  = port
        self.baud  = baud

        self._ser         = None
        self._thread      = None
        self._running     = False
        self._lock        = threading.Lock()

        self._data        = None   # dict or None
        self._last_update = None
        self._status      = "waiting"
        self._error_msg   = ""

    # ------------------------------------------------------------------
    def start(self):
        if not SERIAL_AVAILABLE:
            msg = "pyserial not installed.  pip install pyserial"
            self._set_error(msg)
            return False, msg

        try:
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=READ_TIMEOUT)
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        self._running = True
        self._thread  = threading.Thread(
            target=self._read_loop, daemon=True, name="PowerReader"
        )
        self._thread.start()
        return True, None

    def stop(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ------------------------------------------------------------------
    def get_data(self):
        """
        Returns dict or None.
        Dict keys:
            voltage_kv  float  measured voltage in kV
            current_ma  float  measured current in mA
            power_w     float  computed power in W
        """
        with self._lock:
            if (self._last_update is not None and
                    time.monotonic() - self._last_update > STALE_TIMEOUT):
                self._status    = "error"
                self._error_msg = "No data received for >5 s"
                return None
            return self._data

    def get_status(self):
        with self._lock:
            return self._status, self._error_msg

    # ------------------------------------------------------------------
    def _read_loop(self):
        time.sleep(STARTUP_DELAY)
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

        while self._running:
            try:
                raw  = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self._parse_line(line)
            except Exception as exc:
                if self._running:
                    self._set_error(str(exc))
                break

    def _parse_line(self, line: str):
        """
        Format: voltage_set, voltageADC, current_set, currentADC, pressure
        We use fields 1 (voltageADC) and 3 (currentADC).
        """
        parts = line.split(",")
        if len(parts) < 5:
            return
        try:
            voltage_adc = float(parts[1].strip())
            current_adc = float(parts[3].strip())
        except ValueError:
            return

        voltage_kv = voltage_adc * VOLTAGE_SCALE_KV_PER_COUNT
        current_ma = current_adc * CURRENT_SCALE_MA_PER_COUNT
        power_w    = (voltage_kv * 1000) * (current_ma / 1000)   # V × A = W

        with self._lock:
            self._data = {
                "voltage_kv": voltage_kv,
                "current_ma": current_ma,
                "power_w":    power_w,
            }
            self._last_update = time.monotonic()
            self._status      = "ok"
            self._error_msg   = ""

    def _set_error(self, msg):
        with self._lock:
            self._status    = "error"
            self._error_msg = msg


# ===========================================================================
# ArduinoReader — single serial connection, exposes both pressure and power
# ===========================================================================

class ArduinoReader:
    """
    Opens /dev/ttyACM0 ONCE and parses all 5 fields in a single thread.
    Use this instead of running PressureReader and PowerReader separately,
    which would both try to open the same serial port.

    Usage:
        arduino = ArduinoReader()
        ok, err = arduino.start()

        pressure = arduino.get_pressure()   # float (Torr) or None
        power    = arduino.get_power()      # dict or None
        status, msg = arduino.get_status()

        arduino.stop()
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port  = port
        self.baud  = baud

        self._ser         = None
        self._thread      = None
        self._running     = False
        self._lock        = threading.Lock()

        self._pressure    = None
        self._power_data  = None
        self._last_update = None
        self._status      = "waiting"
        self._error_msg   = ""

    # ------------------------------------------------------------------
    def start(self):
        if not SERIAL_AVAILABLE:
            msg = "pyserial not installed.  pip install pyserial"
            self._set_error(msg)
            return False, msg

        try:
            self._ser = serial.Serial(self.port, self.baud,
                                      timeout=READ_TIMEOUT)
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        self._running = True
        self._thread  = threading.Thread(
            target=self._read_loop, daemon=True, name="ArduinoReader"
        )
        self._thread.start()
        return True, None

    def stop(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ------------------------------------------------------------------
    def get_pressure(self):
        """Return latest pressure in Torr, or None."""
        with self._lock:
            if (self._last_update is not None and
                    time.monotonic() - self._last_update > STALE_TIMEOUT):
                self._status    = "error"
                self._error_msg = "No data received for >5 s"
                return None
            return self._pressure

    def get_power(self):
        """Return latest power dict, or None."""
        with self._lock:
            return self._power_data

    def get_status(self):
        with self._lock:
            return self._status, self._error_msg

    # ------------------------------------------------------------------
    def _read_loop(self):
        time.sleep(STARTUP_DELAY)
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

        while self._running:
            try:
                raw  = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self._parse_line(line)
            except Exception as exc:
                if self._running:
                    self._set_error(str(exc))
                break

    def _parse_line(self, line: str):
        parts = line.split(",")
        if len(parts) < 5:
            return
        try:
            voltage_adc = float(parts[1].strip())
            current_adc = float(parts[3].strip())
            pressure    = float(parts[4].strip())
        except ValueError:
            return

        voltage_kv = voltage_adc * VOLTAGE_SCALE_KV_PER_COUNT
        current_ma = current_adc * CURRENT_SCALE_MA_PER_COUNT
        power_w    = (voltage_kv * 1000) * (current_ma / 1000)

        with self._lock:
            self._pressure   = pressure
            self._power_data = {
                "voltage_kv": voltage_kv,
                "current_ma": current_ma,
                "power_w":    power_w,
            }
            self._last_update = time.monotonic()
            self._status      = "ok"
            self._error_msg   = ""

    def _set_error(self, msg):
        with self._lock:
            self._status    = "error"
            self._error_msg = msg
