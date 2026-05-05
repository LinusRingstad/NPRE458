"""
plasma_controller.py
====================
Closed-loop optimizer for electron temperature (Te) and plasma density (ne).

Algorithm: Proportional-Integral (PI) hill-climber
---------------------------------------------------
Every CONTROL_INTERVAL seconds the controller:
  1. Reads the latest neural-network radial profile (already computed by the
     20-second prediction loop in main.py via a shared callback).
  2. Interpolates the profile at the user-specified radius to get the
     measured value at that point.
  3. Computes error = setpoint - measured.
  4. Updates a PI accumulator:
       integral += error * dt
       output    = Kp * error + Ki * integral
  5. Clamps output to the allowed current range (0–90 mA) and sends it to
     PowerController.
  6. After ALARM_CYCLES consecutive cycles where |relative error| >
     ALARM_THRESHOLD, sets the MSE alarm flag so the GUI can show a warning.
  7. Resets the integral whenever the setpoint changes or the controller
     is disabled, to prevent windup.

Priority rules (enforced in main.py via inhibit()):
  - Voltage OR Current manual controls ON  →  controller INHIBITED.
  - Te and ne cannot both be active; Te has priority.
  - Controller only acts when spectrometer is live and not calibrating.

Safety limits:
  Te setpoint:  0 – 2.0 eV
  ne setpoint:  0 – 1e12 cm⁻³  (1e18 m⁻³)
  Current out:  0 – 90 mA
  Voltage out:  0 – 1.0 kV  (rarely touched; current is the primary knob)

MSE flag:
  Displayed on the control card when the controller has been running for
  ALARM_CYCLES × CONTROL_INTERVAL seconds and the relative error is still
  above ALARM_THRESHOLD.  This means "the plasma cannot reach the setpoint
  with the available current range — try a different setpoint."
"""

import threading
import time
import numpy as np

from power_control import PowerController


# ── Physical limits ──────────────────────────────────────────────────────────
MAX_TE_EV      = 2.0
MAX_NE_CM3     = 1e12          # cm⁻³
MAX_NE_M3      = MAX_NE_CM3 * 1e6

MAX_CURRENT_MA = 90.0
MIN_CURRENT_MA = 0.0
MAX_VOLTAGE_KV = 1.0
MIN_VOLTAGE_KV = 0.0

# ── PI tuning ────────────────────────────────────────────────────────────────
# These are intentionally conservative.  The system has significant latency
# (spectrometer EMA, 20 s prediction window) so aggressive gains cause
# oscillation.  Tune Kp first, then add Ki.

CONTROL_INTERVAL_S = 5.0    # seconds between control updates

# Te controller gains  (output unit: mA per eV of error)
TE_KP = 5.0    # proportional gain  [mA/eV]
TE_KI = 0.5    # integral gain      [mA/(eV·s)]

# ne controller gains  (output unit: mA per relative error)
NE_KP = 8.0    # proportional gain  [mA / (ne_err/ne_target)]
NE_KI = 0.8    # integral gain      [mA / (ne_err/ne_target · s)]

# Alarm: raise flag after this many consecutive cycles above threshold
ALARM_CYCLES    = 6           # 6 × 5 s = 30 s
ALARM_THRESHOLD = 0.10        # 10 % relative error


class PIController:
    """Simple PI state for one controlled variable."""

    def __init__(self, kp: float, ki: float, dt: float,
                 out_min: float, out_max: float):
        self.kp      = kp
        self.ki      = ki
        self.dt      = dt
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def step(self, error: float, feedforward: float = 0.0) -> float:
        """
        Compute new output given error.  feedforward is added after clamping
        (e.g. the current operating point) so the PI acts as an *increment*.
        """
        self._integral += error * self.dt
        # Anti-windup: clamp integral contribution
        max_int = (self.out_max - self.out_min) / max(self.ki * self.dt, 1e-9)
        self._integral = float(np.clip(self._integral, -max_int, max_int))

        raw    = self.kp * error + self.ki * self._integral
        output = float(np.clip(feedforward + raw, self.out_min, self.out_max))
        return output


class PlasmaController:
    """
    Closed-loop PI optimizer for Te and ne targets.

    Start once, leave running. Wire up via:
        ctrl.set_profile_callback(fn)   ← called by main.py whenever a new
                                           profile arrives (every 20 s)
        ctrl.inhibit(True/False)        ← when manual V/I controls are on/off
        ctrl.enable_te(True/False)
        ctrl.enable_ne(True/False)
        ctrl.set_te_target(eV, radius)
        ctrl.set_ne_target(cm3, radius)
    """

    def __init__(self, power_ctrl: PowerController):
        self._power  = power_ctrl
        self._lock   = threading.Lock()

        # Latest profile — written by main.py callback, read by control thread
        self._profile = None

        # PI controllers (one per variable)
        self._te_pi = PIController(TE_KP, TE_KI, CONTROL_INTERVAL_S,
                                   MIN_CURRENT_MA, MAX_CURRENT_MA)
        self._ne_pi = PIController(NE_KP, NE_KI, CONTROL_INTERVAL_S,
                                   MIN_CURRENT_MA, MAX_CURRENT_MA)

        # Te state
        self._te_enabled     = False
        self._te_target      = 1.0
        self._te_radius      = 0.14
        self._te_mse         = None
        self._te_alarm       = False
        self._te_alarm_count = 0
        self._te_measured    = None

        # ne state
        self._ne_enabled     = False
        self._ne_target      = 5e10    # cm⁻³
        self._ne_radius      = 0.14
        self._ne_mse         = None
        self._ne_alarm       = False
        self._ne_alarm_count = 0
        self._ne_measured    = None

        self._inhibited = False
        self._status    = "idle"
        self._running   = False
        self._thread    = None

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, daemon=True, name="PlasmaController"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        try:
            self._power.zero()
        except Exception:
            pass

    # ── Profile callback (called from main.py prediction loop) ───────────────

    def on_new_profile(self, profile: dict):
        """
        Register the latest neural-network profile.
        Called from the GUI main thread after each 20 s prediction.
        profile = {"r": ndarray, "te": ndarray, "ne": ndarray}
        """
        with self._lock:
            self._profile = profile

    # ── Setters ──────────────────────────────────────────────────────────────

    def set_te_target(self, te_ev: float, radius: float):
        te_ev = float(np.clip(te_ev, 0.0, MAX_TE_EV))
        with self._lock:
            self._te_target      = te_ev
            self._te_radius      = radius
            self._te_mse         = None
            self._te_alarm       = False
            self._te_alarm_count = 0
            self._te_pi.reset()

    def set_ne_target(self, ne_cm3: float, radius: float):
        ne_cm3 = float(np.clip(ne_cm3, 0.0, MAX_NE_CM3))
        with self._lock:
            self._ne_target      = ne_cm3
            self._ne_radius      = radius
            self._ne_mse         = None
            self._ne_alarm       = False
            self._ne_alarm_count = 0
            self._ne_pi.reset()

    def enable_te(self, on: bool):
        with self._lock:
            if not on:
                self._te_pi.reset()
                self._te_mse   = None
                self._te_alarm = False
                self._te_alarm_count = 0
            self._te_enabled = on

    def enable_ne(self, on: bool):
        with self._lock:
            if not on:
                self._ne_pi.reset()
                self._ne_mse   = None
                self._ne_alarm = False
                self._ne_alarm_count = 0
            self._ne_enabled = on

    def inhibit(self, on: bool):
        """Inhibit controller when manual V/I controls are active."""
        with self._lock:
            if on and not self._inhibited:
                # Reset PI state so there's no windup when control resumes
                self._te_pi.reset()
                self._ne_pi.reset()
            self._inhibited = on

    # ── Getters (GUI main thread) ─────────────────────────────────────────────

    def get_te_mse(self):
        with self._lock:
            return self._te_mse

    def get_ne_mse(self):
        with self._lock:
            return self._ne_mse

    def get_te_alarm(self):
        with self._lock:
            return self._te_alarm

    def get_ne_alarm(self):
        with self._lock:
            return self._ne_alarm

    def get_te_measured(self):
        with self._lock:
            return self._te_measured

    def get_ne_measured(self):
        with self._lock:
            return self._ne_measured

    def get_status(self):
        with self._lock:
            return self._status

    # ── Background control loop ──────────────────────────────────────────────

    def _loop(self):
        while self._running:
            time.sleep(CONTROL_INTERVAL_S)

            with self._lock:
                inhibited  = self._inhibited
                te_enabled = self._te_enabled
                ne_enabled = self._ne_enabled
                profile    = self._profile

            if inhibited:
                with self._lock:
                    self._status = "inhibited"
                continue

            if profile is None or (not te_enabled and not ne_enabled):
                with self._lock:
                    self._status = "idle"
                continue

            # Te has priority
            if te_enabled:
                self._step_te(profile)
            elif ne_enabled:
                self._step_ne(profile)

    # ── Te control step ──────────────────────────────────────────────────────

    def _step_te(self, profile):
        with self._lock:
            target = self._te_target
            radius = self._te_radius
            pi     = self._te_pi

        measured = self._interp(profile["r"], profile["te"], radius)
        if measured is None:
            return

        error    = target - measured
        rel_err2 = (error / target) ** 2 if abs(target) > 1e-9 else error ** 2

        # PI step — feedforward = current operating point
        new_i = pi.step(error, feedforward=self._power.current_ma)

        with self._lock:
            self._te_measured = measured
            self._te_mse      = rel_err2
            self._status      = "controlling_te"

            if rel_err2 > ALARM_THRESHOLD:
                self._te_alarm_count += 1
            else:
                self._te_alarm_count = 0

            self._te_alarm = self._te_alarm_count >= ALARM_CYCLES

        self._power.set_current(new_i)

        print(f"[Ctrl/Te] target={target:.3f}eV  meas={measured:.3f}eV  "
              f"err={error:+.3f}  I→{new_i:.1f}mA  "
              f"alarm={self._te_alarm_count}/{ALARM_CYCLES}")

    # ── ne control step ──────────────────────────────────────────────────────

    def _step_ne(self, profile):
        with self._lock:
            target_cm3 = self._ne_target
            radius     = self._ne_radius
            pi         = self._ne_pi

        target_m3   = target_cm3 * 1e6
        measured_m3 = self._interp(profile["r"], profile["ne"], radius)
        if measured_m3 is None:
            return

        # Use relative error as the error signal so gains are dimensionless
        rel_error = (target_m3 - measured_m3) / target_m3 if abs(target_m3) > 0 else 0.0
        rel_err2  = rel_error ** 2

        new_i = pi.step(rel_error, feedforward=self._power.current_ma)

        with self._lock:
            self._ne_measured = measured_m3 / 1e6   # store in cm⁻³
            self._ne_mse      = rel_err2
            self._status      = "controlling_ne"

            if rel_err2 > ALARM_THRESHOLD:
                self._ne_alarm_count += 1
            else:
                self._ne_alarm_count = 0

            self._ne_alarm = self._ne_alarm_count >= ALARM_CYCLES

        self._power.set_current(new_i)

        print(f"[Ctrl/ne] target={target_cm3:.3e}cm⁻³  "
              f"meas={measured_m3/1e6:.3e}cm⁻³  "
              f"rel_err={rel_error:+.4f}  I→{new_i:.1f}mA  "
              f"alarm={self._ne_alarm_count}/{ALARM_CYCLES}")

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _interp(r, values, target_r):
        if target_r < r.min() or target_r > r.max():
            return None
        return float(np.interp(target_r, r, values))