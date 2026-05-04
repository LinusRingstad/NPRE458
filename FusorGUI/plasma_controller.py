"""
plasma_controller.py
====================
Closed-loop optimization controller for electron temperature (Te) and
plasma density (ne).

Algorithm
---------
Hill-climbing with perturbation (gradient-free, 1-D coordinate descent):

  Every CONTROL_INTERVAL seconds:
    1. Read latest predicted Te or ne from the neural network profile
       (evaluated at the user-specified radius).
    2. Compute error = setpoint - measured.
    3. Nudge the current setpoint by ±STEP_MA in the direction that reduces
       error (sign of error × gain).
    4. If the error has not improved after CONVERGENCE_CHECKS consecutive
       cycles, raise the MSE_ALARM flag.
    5. Send the new current (and optionally voltage) to PowerController.

Safety limits (enforced here AND in power_control.py):
    Current:  0 – 90 mA
    Voltage:  0 – 1.0 kV
    Te:       0 – 2.0 eV        (setpoint clamped)
    ne:       0 – 1e12 cm⁻³     (setpoint clamped, = 1e18 m⁻³ )

Priority:
    If Voltage OR Current manual controls are ON → controller is INHIBITED.
    Te and ne controllers never run simultaneously (Te has priority).

Usage:
    ctrl = PlasmaController(power_ctrl, predictor, spec_panel)
    ctrl.start()

    ctrl.set_te_target(1.5, radius=0.10)    # eV, metres
    ctrl.enable_te(True)

    ctrl.set_ne_target(5e10, radius=0.10)   # cm⁻³
    ctrl.enable_ne(True)

    ctrl.inhibit(True)   # call when manual power controls are ON
    ctrl.inhibit(False)  # call when manual power controls are OFF

    ctrl.stop()

    # Poll these from the GUI:
    ctrl.get_te_mse()    # float or None
    ctrl.get_ne_mse()    # float or None
    ctrl.get_status()    # "idle" | "controlling_te" | "controlling_ne"
                         # | "inhibited" | "stopped"
"""

import threading
import time
import numpy as np

from power_control import PowerController
from peak_ratios   import extract_ratios


# ── Physical limits ──────────────────────────────────────────────────────────
MAX_TE_EV         = 2.0          # eV
MAX_NE_CM3        = 1e12         # cm⁻³
MAX_NE_M3         = MAX_NE_CM3 * 1e6   # m⁻³ (model units)

MAX_CURRENT_MA    = 90.0
MIN_CURRENT_MA    = 0.0
MAX_VOLTAGE_KV    = 1.0
MIN_VOLTAGE_KV    = 0.0

# ── Controller tuning ────────────────────────────────────────────────────────
CONTROL_INTERVAL_S  = 5.0    # seconds between control updates
STEP_MA             = 1.0    # mA per nudge (current adjustment step)
STEP_KV             = 0.01   # kV per nudge (voltage adjustment step, rarely used)
GAIN                = 1.0    # multiplier on step (increase for faster response)

# After this many consecutive cycles with no improvement → raise MSE alarm
CONVERGENCE_CHECKS  = 6      # 6 × 5 s = 30 s before alarm

# MSE threshold above which the alarm is raised (relative squared error)
MSE_ALARM_THRESHOLD = 0.10   # 10% relative error


class PlasmaController:
    """
    Closed-loop optimizer for Te and ne targets.

    Designed to be started once and left running. Enable/disable individual
    loops via enable_te() / enable_ne(). Call inhibit(True) whenever manual
    power controls are active.
    """

    def __init__(self, power_ctrl: PowerController,
                 predictor,       # PlasmaPredictor instance
                 spec_panel):     # SpectrometerPanel instance (for latest data)

        self._power   = power_ctrl
        self._pred    = predictor
        self._spec    = spec_panel

        self._lock    = threading.Lock()
        self._running = False
        self._thread  = None

        # ── Te controller state ──────────────────────────────────────────
        self._te_enabled  = False
        self._te_target   = 1.0    # eV
        self._te_radius   = 0.14   # m
        self._te_mse      = None   # float or None
        self._te_alarm    = False
        self._te_no_improve = 0

        # ── ne controller state ──────────────────────────────────────────
        self._ne_enabled  = False
        self._ne_target   = 5e10   # cm⁻³
        self._ne_radius   = 0.14   # m
        self._ne_mse      = None
        self._ne_alarm    = False
        self._ne_no_improve = 0

        # ── Shared ──────────────────────────────────────────────────────
        self._inhibited   = False
        self._status      = "idle"

        # Track last error magnitudes for convergence detection
        self._last_te_err2 = None
        self._last_ne_err2 = None

    # ── Public API ───────────────────────────────────────────────────────────

    def start(self):
        """Start the background control thread."""
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, daemon=True, name="PlasmaController"
        )
        self._thread.start()

    def stop(self):
        """Stop the control thread and zero the outputs."""
        self._running = False
        self._power.zero()

    # ── Target setters ───────────────────────────────────────────────────────

    def set_te_target(self, te_ev: float, radius: float):
        te_ev = max(0.0, min(MAX_TE_EV, te_ev))
        with self._lock:
            self._te_target  = te_ev
            self._te_radius  = radius
            self._te_mse     = None
            self._te_alarm   = False
            self._te_no_improve = 0
            self._last_te_err2  = None

    def set_ne_target(self, ne_cm3: float, radius: float):
        ne_cm3 = max(0.0, min(MAX_NE_CM3, ne_cm3))
        with self._lock:
            self._ne_target  = ne_cm3
            self._ne_radius  = radius
            self._ne_mse     = None
            self._ne_alarm   = False
            self._ne_no_improve = 0
            self._last_ne_err2  = None

    # ── Enable / disable ─────────────────────────────────────────────────────

    def enable_te(self, on: bool):
        with self._lock:
            self._te_enabled = on
            if not on:
                self._te_mse   = None
                self._te_alarm = False

    def enable_ne(self, on: bool):
        with self._lock:
            self._ne_enabled = on
            if not on:
                self._ne_mse   = None
                self._ne_alarm = False

    def inhibit(self, on: bool):
        """Inhibit when manual power controls are active."""
        with self._lock:
            self._inhibited = on
            if on:
                self._status = "inhibited"

    # ── Getters (called from GUI main thread) ────────────────────────────────

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

            if inhibited or (not te_enabled and not ne_enabled):
                with self._lock:
                    self._status = "inhibited" if inhibited else "idle"
                continue

            # Get latest spectrum and run prediction
            data = self._spec.get_latest_data()
            if data is None or data.get("calibrating"):
                continue

            ratios, reason = extract_ratios(
                data["wavelengths"], data["intensity"]
            )
            if ratios is None:
                print(f"[Controller] Skipping: {reason}")
                continue

            profile = self._pred.predict_radial_profile(ratios)
            if profile is None:
                continue

            # Te has priority over ne
            if te_enabled:
                self._control_te(profile)
            elif ne_enabled:
                self._control_ne(profile)

    # ── Te control step ──────────────────────────────────────────────────────

    def _control_te(self, profile):
        with self._lock:
            target  = self._te_target
            radius  = self._te_radius

        measured = self._interpolate(profile["r"], profile["te"], radius)
        if measured is None:
            return

        error     = target - measured
        rel_err2  = (error / target) ** 2 if target != 0 else error ** 2

        # Convergence check
        with self._lock:
            self._te_mse  = rel_err2
            self._status  = "controlling_te"

            if self._last_te_err2 is not None:
                if rel_err2 >= self._last_te_err2:
                    self._te_no_improve += 1
                else:
                    self._te_no_improve = 0

            self._last_te_err2 = rel_err2
            self._te_alarm = (
                self._te_no_improve >= CONVERGENCE_CHECKS and
                rel_err2 > MSE_ALARM_THRESHOLD
            )

        print(f"[Controller/Te] target={target:.2f}eV  "
              f"measured={measured:.2f}eV  "
              f"err={error:+.3f}  alarm={self._te_alarm}")

        # Nudge current in direction that should increase/decrease Te
        # Higher current → denser/hotter plasma → higher Te (positive correlation)
        step = np.sign(error) * STEP_MA * GAIN
        new_i = float(np.clip(
            self._power.current_ma + step,
            MIN_CURRENT_MA, MAX_CURRENT_MA
        ))
        self._power.set_current(new_i)

    # ── ne control step ──────────────────────────────────────────────────────

    def _control_ne(self, profile):
        with self._lock:
            target_cm3 = self._ne_target
            radius     = self._ne_radius

        target_m3 = target_cm3 * 1e6   # convert to m⁻³ for comparison with profile

        measured_m3 = self._interpolate(profile["r"], profile["ne"], radius)
        if measured_m3 is None:
            return

        error_m3  = target_m3 - measured_m3
        rel_err2  = (error_m3 / target_m3) ** 2 if target_m3 != 0 else error_m3 ** 2

        with self._lock:
            self._ne_mse = rel_err2
            self._status = "controlling_ne"

            if self._last_ne_err2 is not None:
                if rel_err2 >= self._last_ne_err2:
                    self._ne_no_improve += 1
                else:
                    self._ne_no_improve = 0

            self._last_ne_err2 = rel_err2
            self._ne_alarm = (
                self._ne_no_improve >= CONVERGENCE_CHECKS and
                rel_err2 > MSE_ALARM_THRESHOLD
            )

        print(f"[Controller/ne] target={target_cm3:.3e}cm⁻³  "
              f"measured={measured_m3/1e6:.3e}cm⁻³  "
              f"rel_err={rel_err2:.4f}  alarm={self._ne_alarm}")

        step  = np.sign(error_m3) * STEP_MA * GAIN
        new_i = float(np.clip(
            self._power.current_ma + step,
            MIN_CURRENT_MA, MAX_CURRENT_MA
        ))
        self._power.set_current(new_i)

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _interpolate(r: np.ndarray, values: np.ndarray,
                     target_r: float):
        """
        Linearly interpolate the profile at target_r.
        Returns None if target_r is outside the profile range.
        """
        if target_r < r.min() or target_r > r.max():
            return None
        return float(np.interp(target_r, r, values))
