"""
spectrometer.py
===============
USB spectrometer reader for the Plasma Control System GUI.
 
Captures frames from the spectrometer camera (via OpenCV), extracts the
intensity profile from a defined region of interest, and applies the
wavelength calibration from the INI file.
 
Calibration (from INI):
  TrimPoint1: pixel 436  -> 435.8 nm
  TrimPoint2: pixel 546  -> 546.1 nm
  Slope  m = (546.1 - 435.8) / (546 - 436) = 1.103 nm/pixel
  Intercept b = 435.8 - (1.103 * 436)       = -45.108
 
Usage:
    spec = Spectrometer()
    ok, err = spec.start()
    if not ok:
        print("Spectrometer error:", err)
 
    data = spec.get_data()
    # data is None until first frame, then a dict:
    # {
    #   "wavelengths": np.ndarray  (nm, one per pixel column in ROI)
    #   "intensity":   np.ndarray  (0-255, averaged over ROI rows)
    #   "peak_nm":     float       (wavelength of strongest peak)
    #   "peak_idx":    int         (index into arrays of the peak)
    # }
 
    spec.stop()
"""
 
import threading
import numpy as np
 
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
 
 
# ---------------------------------------------------------------------------
# Calibration & ROI - adjust to match your spectrometer's INI file
# ---------------------------------------------------------------------------
SLOPE     =  1.103       # nm / pixel
INTERCEPT = -45.108      # nm
 
# Region of interest in the full 1280x720 frame
START_X = 329
END_X   = 968
START_Y = 371
SIZE_Y  = 141
 
# Camera index - 0 is the first USB device; change if the Pi has multiple cameras
CAMERA_INDEX = 1
 
 
class Spectrometer:
    """
    Thread-safe wrapper around an OpenCV USB camera used as a spectrometer.
 
    A background thread continuously captures frames, extracts the ROI,
    averages rows to produce an intensity profile, and stores the result.
    The GUI calls get_data() from the main thread with no blocking.
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
 
        # Shared state (protected by _lock)
        self._data      = None
        self._status    = "waiting"   # "waiting" | "ok" | "error"
        self._error_msg = ""
 
        # Pre-compute wavelength axis (one value per pixel column in ROI)
        pixel_indices     = np.arange(start_x, end_x)
        self._wavelengths = pixel_indices * slope + intercept
 
    # ------------------------------------------------------------------
    def start(self):
        """
        Open the camera and start the capture thread.
        Returns (True, None) on success or (False, error_string) on failure.
        """
        if not CV2_AVAILABLE:
            msg = ("opencv-python is not installed.\n"
                   "Install it with:  pip install opencv-python")
            self._set_error(msg)
            return False, msg
 
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            msg = (f"Could not open camera at index {self.camera_index}.\n"
                   "Check the spectrometer USB is connected and no other\n"
                   "application is using it.")
            self._set_error(msg)
            return False, msg
 
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Manual mode
        cap.set(cv2.CAP_PROP_EXPOSURE, 5)     # Larger values = brighter 2S to 2P0
        cap.set(cv2.CAP_PROP_GAIN, 255)
 
        self._cap     = cap
        self._running = True
        self._thread  = threading.Thread(
            target=self._capture_loop, daemon=True, name="Spectrometer"
        )
        self._thread.start()
        return True, None
 
    # ------------------------------------------------------------------
    def stop(self):
        """Stop the capture thread and release the camera."""
        self._running = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
 
    # ------------------------------------------------------------------
    def get_data(self):
        """
        Return the latest spectrum data dict, or None if not ready.
 
        Keys:
            wavelengths  np.ndarray  wavelength axis in nm
            intensity    np.ndarray  intensity profile 0-255
            peak_nm      float       wavelength of the brightest peak
            peak_idx     int         array index of the peak
        """
        with self._lock:
            return self._data
 
    def get_status(self):
        """Return (status, message) where status is 'waiting', 'ok', or 'error'."""
        with self._lock:
            return self._status, self._error_msg
 
    # ------------------------------------------------------------------
    def _capture_loop(self):
        """Background thread: grab frames and extract intensity profile."""
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                if self._running:
                    self._set_error("Camera read failed - frame not returned.")
                break
 
            try:
                roi  = frame[self.start_y : self.start_y + self.size_y,
                              self.start_x : self.end_x]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
 
                # Average all rows -> 1-D intensity profile
                intensity = np.mean(gray, axis=0).astype(np.float32)
 
                peak_idx   = int(np.argmax(intensity))
                peak_pixel = self.start_x + peak_idx
                peak_nm    = peak_pixel * self.slope + self.intercept
 
                data = {
                    "wavelengths": self._wavelengths,
                    "intensity":   intensity,
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
