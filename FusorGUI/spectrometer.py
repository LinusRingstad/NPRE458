import threading
import numpy as np
import time

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# --- Calibration & ROI ---
SLOPE     =  1.103       
INTERCEPT = -45.108      
START_X, END_X = 329, 968
START_Y, SIZE_Y = 371, 141
CAMERA_INDEX = 1

ALPHA = 0.05             
CALIBRATION_TIME = 15.0  

class Spectrometer:
    def __init__(self, camera_index=CAMERA_INDEX, slope=SLOPE, intercept=INTERCEPT,
                 start_x=START_X, end_x=END_X, start_y=START_Y, size_y=SIZE_Y):

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

        self._data      = None
        self._status    = "waiting"
        self._error_msg = ""

        self._stacked_profile    = None
        self._background_profile = None
        self._start_time         = 0

        # Create the wavelength axis once based on the ROI width
        # This ensures the X-axis always matches the number of points in the intensity profile
        self._wavelengths = np.linspace(
            (self.start_x * self.slope) + self.intercept,
            (self.end_x * self.slope) + self.intercept,
            (self.end_x - self.start_x)
        ).astype(np.float32)

    def start(self):
        if not CV2_AVAILABLE:
            msg = "opencv-python not found."
            self._set_error(msg)
            return False, msg

        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            msg = f"Camera index {self.camera_index} failed."
            self._set_error(msg)
            return False, msg

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._cap = cap
        self._running = True
        self._start_time = time.time()
        
        self._stacked_profile = None
        self._background_profile = None

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True, None

    def get_data(self):
        with self._lock:
            return self._data

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret: break

            try:
                # 1. Slice ROI
                roi = frame[self.start_y : self.start_y + self.size_y, 
                            self.start_x : self.end_x]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                current_raw = np.mean(gray, axis=0, dtype=np.float32)

                elapsed = time.time() - self._start_time

                # 2. Calibration vs Measurement
                if elapsed < CALIBRATION_TIME:
                    if self._background_profile is None:
                        self._background_profile = current_raw
                    else:
                        self._background_profile = (1 - ALPHA) * self._background_profile + ALPHA * current_raw
                    
                    # During calibration, show raw profile
                    processed_intensity = current_raw
                else:
                    # Subtract and Stack
                    subtracted = np.maximum(current_raw - self._background_profile, 0)
                    if self._stacked_profile is None:
                        self._stacked_profile = subtracted
                    else:
                        self._stacked_profile = (1 - ALPHA) * self._stacked_profile + ALPHA * subtracted
                    
                    processed_intensity = self._stacked_profile

                # 3. Final Rescaling for the GUI (Normalize 0-255)
                # This ensures the peak always hits the top of the graph
                max_val = np.max(processed_intensity)
                if max_val > 0:
                    display_intensity = (processed_intensity / max_val) * 255
                else:
                    display_intensity = processed_intensity

                peak_idx = int(np.argmax(display_intensity))
                
                # Update shared data
                with self._lock:
                    self._data = {
                        "wavelengths": self._wavelengths, # Corrected wavelength axis
                        "intensity": display_intensity,   # Scaled 0-255 axis
                        "peak_nm": self._wavelengths[peak_idx],
                        "peak_idx": peak_idx
                    }
                    self._status = "ok"

            except Exception as e:
                self._set_error(str(e))
                break

    def _set_error(self, msg):
        with self._lock:
            self._status = "error"
            self._error_msg = msg

    def stop(self):
        self._running = False
        if self._cap: self._cap.release()