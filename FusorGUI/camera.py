"""
camera.py
=========
Raspberry Pi camera feed module for the Plasma Control System GUI.

Wraps Picamera2 in a background thread and exposes a clean API:
    cam = PlasmaCamera()
    cam.start()          # begin capture
    cam.stop()           # stop capture and release hardware
    cam.get_frame()      # returns latest PIL Image or None

The CameraPanel widget in main.py imports this module and calls
these methods directly — no tkinter code lives here.
"""

import threading
import cv2
from PIL import Image

# ---------------------------------------------------------------------------
# Try to import Picamera2 — fail gracefully on non-Pi systems
# ---------------------------------------------------------------------------
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


# ---------------------------------------------------------------------------
# Camera resolution
# ---------------------------------------------------------------------------
CAPTURE_WIDTH  = 640
CAPTURE_HEIGHT = 480


class PlasmaCamera:
    """
    Thread-safe wrapper around Picamera2.

    Usage:
        cam = PlasmaCamera()
        ok, err = cam.start()
        if not ok:
            print("Camera error:", err)
        frame = cam.get_frame()   # PIL Image or None
        cam.stop()
    """

    def __init__(self, width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT):
        self.width      = width
        self.height     = height
        self._picam     = None
        self._latest    = None          # PIL Image, updated by callback
        self._lock      = threading.Lock()
        self._running   = False

    # ------------------------------------------------------------------
    def start(self):
        """
        Initialise and start the camera.
        Returns (True, None) on success, or (False, error_string) on failure.
        """
        if not PICAMERA_AVAILABLE:
            return False, (
                "picamera2 is not installed.\n"
                "Install it with:  pip install picamera2"
            )

        try:
            self._picam = Picamera2()
            config = self._picam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "BGR888"}
            )
            self._picam.configure(config)
            self._picam.post_callback = self._frame_callback
            self._picam.start()
            self._running = True
            return True, None

        except Exception as exc:
            self._picam = None
            return False, str(exc)

    # ------------------------------------------------------------------
    def stop(self):
        """Stop capture and release the camera hardware."""
        self._running = False
        if self._picam is not None:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
            self._picam = None

    # ------------------------------------------------------------------
    def get_frame(self):
        """
        Return the most recent captured frame as a PIL Image, or None
        if no frame has arrived yet.
        """
        with self._lock:
            return self._latest

    # ------------------------------------------------------------------
    def _frame_callback(self, request):
        """
        Called by Picamera2 on every new frame (runs in a camera thread).
        Converts BGR → RGB and stores a PIL Image.
        """
        if not self._running:
            return
        try:
            frame = request.make_array("main")
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img   = Image.fromarray(frame)
            with self._lock:
                self._latest = img
        except Exception:
            pass    # silently drop corrupt frames
