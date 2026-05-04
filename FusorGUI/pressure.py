"""
pressure.py
===========
Reads the pressure field from the Arduino serial stream.

Serial format (Arduino → Pi), one line per loop():
    voltage_set, voltageADC, current_set, currentADC, pressure
    e.g.  "655.2,5,49.4,17,2.31e-05"

Only the 5th field (index 4) is used here — the pressure value already
converted to Torr by the Arduino's calc_p_ccpg() function.

Usage:
    reader = PressureReader()
    ok, err = reader.start()
    val = reader.get_pressure()       # float (Torr) or None
    status, msg = reader.get_status() # "waiting" | "ok" | "error"
    reader.stop()
"""

import threading
import time
import re

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


class PressureReader:
    """
    Thread-safe serial reader — extracts only the pressure field from the
    5-field CSV that the Arduino sends each loop iteration.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port  = port
        self.baud  = baud

        self._ser         = None
        self._thread      = None
        self._running     = False
        self._lock        = threading.Lock()

        self._pressure    = None
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
            target=self._read_loop, daemon=True, name="PressureReader"
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
        with self._lock:
            if (self._last_update is not None and
                    time.monotonic() - self._last_update > STALE_TIMEOUT):
                self._status    = "error"
                self._error_msg = "No data received for >5 s"
                return None
            return self._pressure

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
        Expected format:  voltage_set,voltageADC,current_set,currentADC,pressure
        Pressure is field index 4 (last field).
        Falls back to parsing the first float found if the format changes.
        """
        parts = line.split(",")
        value = None

        if len(parts) >= 5:
            # Normal 5-field format — take the last field
            try:
                value = float(parts[4].strip())
            except ValueError:
                pass

        if value is None:
            # Fallback: find first float in line (handles old single-value format)
            match = re.search(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", line)
            if match:
                try:
                    value = float(match.group())
                except ValueError:
                    pass

        if value is not None:
            with self._lock:
                self._pressure    = value
                self._last_update = time.monotonic()
                self._status      = "ok"
                self._error_msg   = ""

    def _set_error(self, msg):
        with self._lock:
            self._status    = "error"
            self._error_msg = msg