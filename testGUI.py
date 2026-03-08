import tkinter as tk
from PIL import Image, ImageTk
from picamera2 import Picamera2
import cv2
import threading

class CameraApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Plasma Chamber Camera")

        self.win_width = 640
        self.win_height = 480
        self.latest_img = None
        self.lock = threading.Lock()

        # Create camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "BGR888"}  # Request BGR explicitly
        )
        self.picam2.configure(config)

        # Use a callback to grab frames as they arrive from the camera
        self.picam2.post_callback = self.frame_callback
        self.picam2.start()

        # Tkinter label
        self.label = tk.Label(root)
        self.label.pack(fill=tk.BOTH, expand=True)
        self.root.bind("<Configure>", self.on_resize)

        self.update_display()

    def frame_callback(self, request):
        """Called by picamera2 every time a new frame is ready."""
        with self.lock:
            frame = request.make_array("main")
            # BGR -> RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.latest_img = Image.fromarray(frame)

    def on_resize(self, event):
        if event.widget == self.root:
            self.win_width = event.width
            self.win_height = event.height

    def update_display(self):
        """Tkinter loop — just displays whatever the latest frame is."""
        with self.lock:
            img = self.latest_img

        if img is not None:
            img = img.resize((self.win_width, self.win_height), Image.NEAREST)
            imgtk = ImageTk.PhotoImage(image=img)
            self.label.imgtk = imgtk
            self.label.configure(image=imgtk)

        self.root.after(30, self.update_display)

root = tk.Tk()
root.geometry("640x480")
app = CameraApp(root)
root.mainloop()
