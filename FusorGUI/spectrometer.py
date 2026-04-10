import threading
import numpy as np
import time

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ---------------------------------------------------------------------------
# Calibration & ROI
# ---------------------------------------------------------------------------
SLOPE     =  1.103       
INTERCEPT = -45.108      
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141
CAMERA_INDEX = 1

# Stacking factor: 0.05 = heavy smoothing, 0.5 = fast reaction
ALPHA = 0.05
CALIBRATION_TIME = 15.0 

class Spectrometer:
    def __init__(self,
                 camera_index=CAMERA_INDEX,
                 slope=SLOPE,
                 intercept=INTERCEPT,
                 start_x=START_X, end_x=END_X,
                 start_y=START_Y, size_y=SIZE_Y):

        self.camera_index = camera_index
        self.slope        = slope
        self.intercept    = intercept
        self.start_x      = start_x
        self.end_x        = end_x
        self.start_y      = start_y
        self.size_y       = size_y

        self._cap      = None
        self._thread   = None
        self._running  = False
        self._lock     = threading.Lock()

        # Shared state
        self._data      = None
        self._status    = "waiting"
        self._error_msg = ""

        # --- New State Variables for Stacking & Background ---
        self._stacked_profile    = None
        self._background_profile = None
        self._start_time         = 0
        self._is_calibrated      = False

        pixel_indices     = np.arange(start_x, end_x)
        self._wavelengths = pixel_indices * slope + intercept

    def start(self):
        if not CV2_AVAILABLE:
            msg = "opencv-python is not installed."
            self._set_error(msg)
            return False, msg

        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            msg = f"Could not open camera at index {self.camera_index}."
            self._set_error(msg)
            return False, msg

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
        cap.set(cv2.CAP_PROP_EXPOSURE, 5)     
        cap.set(cv2.CAP_PROP_GAIN, 255)

        self._cap        = cap
        self._running    = True
        self._start_time = time.time() # Reset timer on start
        self._thread     = threading.Thread(
            target=self._capture_loop, daemon=True, name="Spectrometer"
        )
        self._thread.start()
        return True, None

    def stop(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def get_data(self):
        with self._lock:
            return self._data

    def get_status(self):
        with self._lock:
            # Update status message if still calibrating
            if self._status == "calibrating":
                remaining = max(0, int(CALIBRATION_TIME - (time.time() - self._start_time)))
                return "calibrating", f"Capturing background: {remaining}s remaining"
            return self._status, self._error_msg

    def _capture_loop(self):
        """Background thread: grab frames, stack, and subtract background."""
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                if self._running:
                    self._set_error("Camera read failed.")
                break

            try:
                # 1. Extract ROI and Profile
                roi  = frame[self.start_y : self.start_y + self.size_y,
                              self.start_x : self.end_x]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                current_raw = np.mean(gray, axis=0, dtype=np.float32)

                elapsed = time.time() - self._start_time

                # --- PHASE 1: CALIBRATION ---
                if elapsed < CALIBRATION_TIME:
                    with self._lock: self._status = "calibrating"
                    
                    if self._background_profile is None:
                        self._background_profile = current_raw
                    else:
                        # Exponential moving average to get a clean background
                        self._background_profile = (1 - ALPHA) * self._background_profile + ALPHA * current_raw
                    continue # Don't publish data yet

                # --- PHASE 2: SUBTRACTION & STACKING ---
                self._is_calibrated = True
                
                # Subtract background and clip negatives to zero
                clean_signal = np.maximum(current_raw - self._background_profile, 0)

                if self._stacked_profile is None:
                    self._stacked_profile = clean_signal
                else:
                    # Apply frame stacking (EMA)
                    self._stacked_profile = (1 - ALPHA) * self._stacked_profile + ALPHA * clean_signal

                # Calculate peak on the cleaned/stacked data
                peak_idx   = int(np.argmax(self._stacked_profile))
                peak_pixel = self.start_x + peak_idx
                peak_nm    = peak_pixel * self.slope + self.intercept

                data = {
                    "wavelengths": self._wavelengths,
                    "intensity":   self._stacked_profile, # Smoothed/Subtracted
                    "peak_nm":     peak_nm,
                    "peak_idx":    peak_idx,
                }

                with self._lock:
                    self._data      = data
                    self._status    = "ok"
                    self._error_msg = ""

            except Exception as exc:
                if self._running:
                    self._set_error(str(exc))
                break

    def _set_error(self, msg):
        with self._lock:
            self._status    = "error"
            self._error_msg = msg