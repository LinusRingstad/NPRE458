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
SLOPE     =  1.103       # nm / pixel
INTERCEPT = -45.108      # nm

START_X = 329
END_X   = 968
START_Y = 371
SIZE_Y  = 141

CAMERA_INDEX = 1

# --- Stacking & Calibration Settings ---
ALPHA = 0.05             # Smoothing factor (0.05 = heavy, 0.5 = light)
CALIBRATION_TIME = 15.0  # Seconds to collect background noise floor

class Spectrometer:
    """
    Thread-safe wrapper around an OpenCV USB camera used as a spectrometer.
    Includes automatic 15-second background subtraction and frame stacking.
    """

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

        # --- Stacking/Background State ---
        self._stacked_profile    = None
        self._background_profile = None
        self._start_time         = 0

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
        self._start_time = time.time() # Start calibration timer
        
        # Reset profiles for new run
        self._stacked_profile = None
        self._background_profile = None

        self._thread  = threading.Thread(
            target=self._capture_loop, daemon=True, name="Spectrometer"
        )
        self._thread.start()
        return True, None

    def stop(self):
        self._running = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def get_data(self):
        with self._lock:
            return self._data

    def get_status(self):
        with self._lock:
            # Provide specific feedback if still in the 15s window
            if self._status == "ok":
                elapsed = time.time() - self._start_time
                if elapsed < CALIBRATION_TIME:
                    return "calibrating", f"Capturing background: {int(CALIBRATION_TIME - elapsed)}s"
            return self._status, self._error_msg

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                if self._running:
                    self._set_error("Camera read failed.")
                break

            try:
                # 1. Extract ROI and Intensity
                roi  = frame[self.start_y : self.start_y + self.size_y,
                              self.start_x : self.end_x]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                current_raw = np.mean(gray, axis=0).astype(np.float32)

                elapsed = time.time() - self._start_time

                # 2. Calibration Phase: Capture Background
                if elapsed < CALIBRATION_TIME:
                    if self._background_profile is None:
                        self._background_profile = current_raw
                    else:
                        # Stack background for 15 seconds to get clean noise floor
                        self._background_profile = cv2.addWeighted(
                            self._background_profile, 1 - ALPHA, current_raw, ALPHA, 0
                        ).flatten()
                    
                    # We continue to show raw signal in get_data until calibration finishes
                    intensity_to_publish = current_raw
                
                # 3. Measurement Phase: Subtract and Stack
                else:
                    # Subtract background and clip at 0 (no negative light)
                    subtracted = np.maximum(current_raw - self._background_profile, 0)

                    if self._stacked_profile is None:
                        self._stacked_profile = subtracted
                    else:
                        # Apply Exponential Moving Average (Frame Stacking)
                        self._stacked_profile = cv2.addWeighted(
                            self._stacked_profile, 1 - ALPHA, subtracted, ALPHA, 0
                        ).flatten()
                    
                    intensity_to_publish = self._stacked_profile

                # 4. Peak Detection on processed signal
                peak_idx   = int(np.argmax(intensity_to_publish))
                peak_pixel = self.start_x + peak_idx
                peak_nm    = peak_pixel * self.slope + self.intercept

                data = {
                    "wavelengths": self._wavelengths,
                    "intensity":   intensity_to_publish,
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