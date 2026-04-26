"""
pressure.py
===========
Serial pressure gauge reader for the Plasma Control System GUI.

Reads lines from an Arduino over USB serial. The Arduino is expected to
send one pressure value per line, either:
  - A bare float:           "12.34"
  - A labelled float:       "Pressure: 12.34"
  - A labelled float+unit:  "Pressure: 12.34 mTorr"

Any line that cannot be parsed is silently ignored.

Usage:
    reader = PressureReader()           # default port + baud
    ok, err = reader.start()            # open serial, start background thread
    if not ok:
        print("Serial error:", err)

    val = reader.get_pressure()         # float (mTorr) or None
    status, msg = reader.get_status()   # ("ok"|"error"|"waiting", msg)

    reader.stop()                       # close port, stop thread
"""

import threading
import time
import re

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_PORT  = "/dev/ttyACM0"
DEFAULT_BAUD  = 115200
STARTUP_DELAY = 2.0    # seconds to wait after opening port before reading
READ_TIMEOUT  = 1.0    # serial readline timeout (seconds)
STALE_TIMEOUT = 5.0    # seconds before a reading is considered stale


class PressureReader:
    """
    Thread-safe serial reader for the Arduino pressure gauge controller.

    The background thread continuously reads lines from the serial port
    and parses the latest pressure value. The GUI calls get_pressure()
    from the main thread with no risk of blocking.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port  = port
        self.baud  = baud

        self._ser         = None
        self._thread      = None
        self._running     = False
        self._lock        = threading.Lock()

        # Shared state (always accessed under _lock)
        self._pressure    = None        # latest parsed float (mTorr)
        self._last_update = None        # time.monotonic() of last good parse
        self._status      = "waiting"   # "waiting" | "ok" | "error"
        self._error_msg   = ""

    # ------------------------------------------------------------------
    def start(self):
        """
        Open the serial port and start the background reader thread.
        Returns (True, None) on success or (False, error_string) on failure.
        """
        if not SERIAL_AVAILABLE:
            msg = ("pyserial is not installed.\n"
                   "Install it with:  pip install pyserial")
            self._set_error(msg)
            return False, msg

        try:
            self._ser = serial.Serial(
                self.port, self.baud, timeout=READ_TIMEOUT
            )
        except Exception as exc:
            msg = str(exc)
            self._set_error(msg)
            return False, msg

        self._running = True
        self._thread  = threading.Thread(
            target=self._read_loop, daemon=True, name="PressureReader"
        )
        self._thread.start()
        return True, None

    # ------------------------------------------------------------------
    def stop(self):
        """Stop the background thread and close the serial port."""
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ------------------------------------------------------------------
    def get_pressure(self):
        """
        Return the most recent pressure reading as a float (mTorr),
        or None if no valid reading has arrived yet.
        Automatically marks the status as 'error' if data goes stale.
        """
        with self._lock:
            if (self._last_update is not None and
                    time.monotonic() - self._last_update > STALE_TIMEOUT):
                self._status    = "error"
                self._error_msg = "No data received for >5 s"
                return None
            return self._pressure

    def get_status(self):
        """
        Return (status, message).
          "waiting" — port open, no data yet
          "ok"      — receiving data normally
          "error"   — serial error or stale data
        """
        with self._lock:
            return self._status, self._error_msg

    # ------------------------------------------------------------------
    def _read_loop(self):
        """Background thread: flush startup noise then read lines forever."""
        time.sleep(STARTUP_DELAY)
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self._parse_line(line)
            except Exception as exc:
                if self._running:
                    self._set_error(str(exc))
                break

    def _parse_line(self, line):
        """
        Extract a float from a line such as:
            "12.34"
            "Pressure: 12.34"
            "Pressure: 12.34 mTorr"
        Updates shared state under the lock.
        """
        match = re.search(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", line)
        if match:
            try:
                value = float(match.group())
                with self._lock:
                    self._pressure    = value
                    self._last_update = time.monotonic()
                    self._status      = "ok"
                    self._error_msg   = ""
            except ValueError:
                pass    # malformed — ignore silently

    def _set_error(self, msg):
        with self._lock:
            self._status    = "error"
            self._error_msg = msg