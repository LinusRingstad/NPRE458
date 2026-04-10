"""
Plasma Control System GUI
=========================
Main entry point. Layout and widget structure defined here.
Individual control/plot logic is imported from separate modules (see placeholders below).

Tab Structure:
  - Control Tab: Camera feed | Parameter controls | Live plots + sensor readouts
  - Train Tab:   (reserved for future implementation)
"""

import tkinter as tk
from tkinter import ttk, font
import tkinter.font as tkfont
from PIL import Image, ImageTk

from camera import PlasmaCamera
from pressure import PressureReader
from pressure import PressureReader

# ---------------------------------------------------------------------------
# Module placeholders — replace these stubs with your real implementations
# ---------------------------------------------------------------------------

# from camera_feed       import CameraPanel          # Camera image display
# from controls.pressure import PressureControl       # Pressure control widget
# from controls.power    import PowerControl          # Power control widget
# from controls.temp     import ElectronTempControl   # Electron temperature control
# from controls.density  import PlasmaDensityControl  # Plasma density control
# from plots.temp_plot   import ElectronTempPlot      # Te live plot
# from plots.density_plot import PlasmaDensityPlot   # ne live plot
# from plots.eedf_plot   import EEDFPlot              # EEDF live plot
# from readouts          import PowerReadout, PressureReadout  # Sensor readouts


# ---------------------------------------------------------------------------
# Theme / Style constants
# ---------------------------------------------------------------------------

DARK_BG       = "#0d1117"
PANEL_BG      = "#161b22"
BORDER_COLOR  = "#30363d"
ACCENT        = "#58a6ff"
ACCENT_DIM    = "#1f4068"
TEXT_PRIMARY  = "#e6edf3"
TEXT_MUTED    = "#8b949e"
SUCCESS       = "#3fb950"
WARNING       = "#d29922"
DANGER        = "#f85149"
PLOT_BG       = "#0d1117"

FONT_MONO     = ("Courier New", 10)
FONT_LABEL    = ("Courier New", 9)
FONT_HEADER   = ("Courier New", 11, "bold")
FONT_TITLE    = ("Courier New", 13, "bold")
FONT_VALUE    = ("Courier New", 14, "bold")
FONT_SMALL    = ("Courier New", 8)


# ---------------------------------------------------------------------------
# Helper: styled frame with optional header label
# ---------------------------------------------------------------------------

def make_panel(parent, title=None, padx=6, pady=6, **kwargs):
    """Return a dark-themed LabelFrame or Frame."""
    opts = dict(bg=PANEL_BG, relief="flat", bd=0,
                highlightbackground=BORDER_COLOR, highlightthickness=1)
    opts.update(kwargs)

    if title:
        frame = tk.LabelFrame(parent, text=f"  {title}  ",
                              fg=ACCENT, font=FONT_HEADER,
                              labelanchor="nw", **opts)
    else:
        frame = tk.Frame(parent, **opts)

    # Geometry is managed by the caller (pack or grid).
    # padx/pady are stored as attributes for callers that want them.
    frame._default_padx = padx
    frame._default_pady = pady
    return frame


# ---------------------------------------------------------------------------
# Parameter Control Widget
# ---------------------------------------------------------------------------

class ParameterControl(tk.Frame):
    """
    Full-width parameter card:
      ┌──────────────────────────────────────────────────────┐
      │ [ ON / OFF ]   PARAMETER NAME          value  unit   │
      │                ◀◀ DEC    INC ▶▶                      │  ← only when ON
      │                Setpoint: [______]  unit               │  ← only when ON
      └──────────────────────────────────────────────────────┘
    The ON/OFF button is the dominant visual element.
    Arrows and setpoint row are hidden when OFF, revealed when ON.
    Pass sci_notation=True to display values in scientific notation.
    """

    def __init__(self, parent, label, unit, min_val, max_val,
                 default_val, step=1.0, sci_notation=False, **kwargs):
        super().__init__(parent, bg=PANEL_BG,
                         highlightbackground=BORDER_COLOR,
                         highlightthickness=1,
                         **kwargs)

        self.label        = label
        self.unit         = unit
        self.min_val      = min_val
        self.max_val      = max_val
        self.step         = step
        self.sci_notation = sci_notation
        self.enabled      = tk.BooleanVar(value=False)
        self.value        = tk.DoubleVar(value=default_val)
        self.setpoint     = tk.DoubleVar(value=default_val)
        self._sp_str      = tk.StringVar(value=self._fmt(default_val))
        self._display_str = tk.StringVar(value=self._fmt(default_val))

        self.columnconfigure(1, weight=1)
        self._build()

    def _fmt(self, val):
        """Format value for display — scientific notation if flagged."""
        if self.sci_notation:
            return f"{val:.3e}"
        return f"{val:.4g}"

    # ------------------------------------------------------------------
    def _build(self):
        # ── Col 0: big ON/OFF toggle (full height of card) ──────────────
        self.toggle_btn = tk.Button(
            self,
            text="OFF",
            command=self._on_toggle,
            bg=DANGER,
            fg=TEXT_PRIMARY,
            activebackground=SUCCESS,
            activeforeground=DARK_BG,
            relief="flat", bd=0,
            cursor="hand2",
            font=("Courier New", 13, "bold"),
            width=5,
        )
        self.toggle_btn.grid(row=0, column=0, rowspan=3,
                             sticky="nsew", padx=(8, 12), pady=8)

        # ── Col 1 Row 0: parameter name + current value ──────────────────
        top_row = tk.Frame(self, bg=PANEL_BG)
        top_row.grid(row=0, column=1, sticky="ew", pady=(10, 2))
        top_row.columnconfigure(0, weight=1)

        tk.Label(top_row, text=self.label,
                 bg=PANEL_BG, fg=TEXT_PRIMARY,
                 font=("Courier New", 12, "bold"),
                 anchor="w").grid(row=0, column=0, sticky="w")

        # value + unit on the right of the name row
        val_frame = tk.Frame(top_row, bg=PANEL_BG)
        val_frame.grid(row=0, column=1, sticky="e")

        self.val_display = tk.Label(
            val_frame,
            textvariable=self._display_str,
            width=12,
            bg=DARK_BG, fg=TEXT_PRIMARY,
            font=("Courier New", 15, "bold"),
            anchor="center",
            padx=6, pady=2,
        )
        self.val_display.pack(side="left")

        tk.Label(val_frame, text=self.unit,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left", padx=(4, 8))

        # ── Col 1 Row 1: arrow controls (hidden until ON) ────────────────
        self.arrow_frame = tk.Frame(self, bg=PANEL_BG)
        # NOT gridded yet — shown only when ON

        self.dec_btn = tk.Button(
            self.arrow_frame, text="◀◀  DEC",
            bg=ACCENT_DIM, fg=ACCENT,
            activebackground=ACCENT, activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=("Courier New", 10, "bold"),
            command=self._decrement,
            padx=10, pady=4,
        )
        self.dec_btn.pack(side="left", padx=(0, 6))

        self.inc_btn = tk.Button(
            self.arrow_frame, text="INC  ▶▶",
            bg=ACCENT_DIM, fg=ACCENT,
            activebackground=ACCENT, activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=("Courier New", 10, "bold"),
            command=self._increment,
            padx=10, pady=4,
        )
        self.inc_btn.pack(side="left")

        # ── Col 1 Row 2: setpoint row (hidden until ON) ──────────────────
        self.sp_frame = tk.Frame(self, bg=PANEL_BG)
        # NOT gridded yet — shown only when ON

        tk.Label(self.sp_frame, text="Setpoint:",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left")

        self.sp_entry = tk.Entry(
            self.sp_frame,
            textvariable=self._sp_str,
            width=12,
            bg=DARK_BG, fg=ACCENT,
            insertbackground=ACCENT,
            relief="flat",
            font=FONT_MONO,
        )
        self.sp_entry.pack(side="left", padx=(6, 4))

        tk.Label(self.sp_frame, text=self.unit,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left")

    # ------------------------------------------------------------------
    def _on_toggle(self):
        self.enabled.set(not self.enabled.get())
        enabled = self.enabled.get()

        if enabled:
            self.toggle_btn.config(text="ON", bg=SUCCESS, fg=DARK_BG,
                                   activebackground=DANGER, activeforeground=TEXT_PRIMARY)
            self.val_display.config(fg=ACCENT)
            # Reveal arrow and setpoint rows
            self.arrow_frame.grid(row=1, column=1, sticky="w", pady=(0, 4))
            self.sp_frame.grid(row=2, column=1, sticky="w", pady=(0, 10))
        else:
            self.toggle_btn.config(text="OFF", bg=DANGER, fg=TEXT_PRIMARY,
                                   activebackground=SUCCESS, activeforeground=DARK_BG)
            self.val_display.config(fg=TEXT_PRIMARY)
            # Hide arrow and setpoint rows
            self.arrow_frame.grid_remove()
            self.sp_frame.grid_remove()

        # TODO: call your hardware enable/disable function here
        # e.g., hardware.set_control_enabled(self.label, enabled)

    def _increment(self):
        new = min(self.value.get() + self.step, self.max_val)
        self.value.set(new)
        self._display_str.set(self._fmt(new))
        self._sp_str.set(self._fmt(new))
        # TODO: send updated value to hardware

    def _decrement(self):
        new = max(self.value.get() - self.step, self.min_val)
        self.value.set(new)
        self._display_str.set(self._fmt(new))
        self._sp_str.set(self._fmt(new))
        # TODO: send updated value to hardware

    # ------------------------------------------------------------------
    def update_measured_value(self, val):
        """Call this from your real-time data loop to update the display."""
        self.value.set(val)
        self._display_str.set(self._fmt(val))


# ---------------------------------------------------------------------------
# Camera Panel placeholder
# ---------------------------------------------------------------------------

class CameraPanel(tk.Frame):
    """
    Live camera feed panel backed by camera.PlasmaCamera.

    Layout (from top to bottom):
      ┌─────────────────────────────────────┐
      │         live image / status         │  ← expands to fill
      │─────────────────────────────────────│
      │  ▶ Play   ■ Stop      status label  │  ← control bar
      └─────────────────────────────────────┘

    The camera is NOT started automatically — the user must press Play.
    If initialisation fails an error message is shown in the panel.
    """

    POLL_MS = 30    # display refresh interval in milliseconds

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=DARK_BG,
                         highlightbackground=BORDER_COLOR,
                         highlightthickness=1, **kwargs)
        self._cam      = PlasmaCamera()
        self._running  = False
        self._after_id = None
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        # ── Image / status area ─────────────────────────────────────────
        self._img_label = tk.Label(self, bg=DARK_BG, cursor="crosshair")
        self._img_label.grid(row=0, column=0, sticky="nsew")

        # Overlay shown when camera is not running
        self._overlay = tk.Frame(self, bg=DARK_BG)
        self._overlay.place(relx=0.5, rely=0.5, anchor="center")

        self._overlay_icon = tk.Label(
            self._overlay, text="◉",
            bg=DARK_BG, fg=BORDER_COLOR,
            font=("Courier New", 42))
        self._overlay_icon.pack()

        self._overlay_msg = tk.Label(
            self._overlay, text="Press  ▶ Play  to start camera",
            bg=DARK_BG, fg=TEXT_MUTED,
            font=FONT_HEADER)
        self._overlay_msg.pack(pady=(4, 0))

        self._error_msg = tk.Label(
            self._overlay, text="",
            bg=DARK_BG, fg=DANGER,
            font=FONT_SMALL,
            wraplength=340, justify="center")
        self._error_msg.pack(pady=(6, 0))

        # ── Control bar ─────────────────────────────────────────────────
        bar = tk.Frame(self, bg=PANEL_BG,
                       highlightbackground=BORDER_COLOR,
                       highlightthickness=1)
        bar.grid(row=1, column=0, sticky="ew")

        self._play_btn = tk.Button(
            bar, text="▶  Play",
            bg=SUCCESS, fg=DARK_BG,
            activebackground="#2ea043", activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=("Courier New", 10, "bold"),
            padx=14, pady=5,
            command=self._on_play,
        )
        self._play_btn.pack(side="left", padx=(8, 4), pady=6)

        self._stop_btn = tk.Button(
            bar, text="■  Stop",
            bg=ACCENT_DIM, fg=TEXT_MUTED,
            activebackground=DANGER, activeforeground=TEXT_PRIMARY,
            relief="flat", bd=0, cursor="hand2",
            font=("Courier New", 10, "bold"),
            padx=14, pady=5,
            state="disabled",
            command=self._on_stop,
        )
        self._stop_btn.pack(side="left", padx=(0, 12), pady=6)

        self._status_lbl = tk.Label(
            bar, text="● Stopped",
            bg=PANEL_BG, fg=TEXT_MUTED,
            font=FONT_LABEL)
        self._status_lbl.pack(side="left")

        # Resolution label on the right
        self._res_lbl = tk.Label(
            bar, text="",
            bg=PANEL_BG, fg=TEXT_MUTED,
            font=FONT_SMALL)
        self._res_lbl.pack(side="right", padx=8)

    # ------------------------------------------------------------------
    def _on_play(self):
        """Start the camera."""
        self._set_status("Starting…", WARNING)
        self._play_btn.config(state="disabled")
        # Run camera init in a thread so the GUI stays responsive
        import threading
        threading.Thread(target=self._start_camera, daemon=True).start()

    def _start_camera(self):
        """Background thread: initialise camera then hand back to GUI."""
        ok, err = self._cam.start()
        # Schedule GUI update back on main thread
        self.after(0, lambda: self._on_camera_ready(ok, err))

    def _on_camera_ready(self, ok, err):
        if ok:
            self._running = True
            self._overlay.place_forget()    # hide the idle overlay
            self._play_btn.config(state="disabled")
            self._stop_btn.config(
                state="normal", bg=DANGER, fg=TEXT_PRIMARY,
                activebackground="#c0392b")
            self._set_status("● Live", SUCCESS)
            self._res_lbl.config(text=f"{self._cam.width}×{self._cam.height}")
            self._poll()                    # start display loop
        else:
            self._play_btn.config(state="normal")
            self._set_status("● Error", DANGER)
            self._overlay_icon.config(fg=DANGER, text="⚠")
            self._overlay_msg.config(
                text="Camera failed to start", fg=DANGER)
            self._error_msg.config(text=err or "Unknown error")

    def _on_stop(self):
        """Stop the camera and return to idle state."""
        self._running = False
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        self._cam.stop()

        # Clear the image and restore overlay
        self._img_label.config(image="")
        self._img_label.image = None
        self._overlay_icon.config(fg=BORDER_COLOR, text="◉")
        self._overlay_msg.config(
            text="Press  ▶ Play  to start camera", fg=TEXT_MUTED)
        self._error_msg.config(text="")
        self._overlay.place(relx=0.5, rely=0.5, anchor="center")

        self._play_btn.config(state="normal")
        self._stop_btn.config(state="disabled",
                              bg=ACCENT_DIM, fg=TEXT_MUTED)
        self._set_status("● Stopped", TEXT_MUTED)
        self._res_lbl.config(text="")

    # ------------------------------------------------------------------
    def _poll(self):
        """Tkinter polling loop — fetches latest frame and updates display."""
        if not self._running:
            return

        frame = self._cam.get_frame()
        if frame is not None:
            # Scale to current widget size, preserving aspect ratio
            w = self._img_label.winfo_width()
            h = self._img_label.winfo_height()
            if w > 1 and h > 1:
                frame = self._scale_to_fit(frame, w, h)
            photo = ImageTk.PhotoImage(image=frame)
            self._img_label.config(image=photo)
            self._img_label.image = photo   # prevent GC

        self._after_id = self.after(self.POLL_MS, self._poll)

    @staticmethod
    def _scale_to_fit(img, target_w, target_h):
        """Resize PIL Image to fit inside (target_w, target_h), keep aspect."""
        src_w, src_h = img.size
        scale = min(target_w / src_w, target_h / src_h)
        new_w = max(1, int(src_w * scale))
        new_h = max(1, int(src_h * scale))
        return img.resize((new_w, new_h), Image.NEAREST)

    # ------------------------------------------------------------------
    def _set_status(self, text, color=TEXT_MUTED):
        self._status_lbl.config(text=text, fg=color)

    def destroy(self):
        """Clean up camera on widget destruction."""
        self._running = False
        if self._after_id:
            self.after_cancel(self._after_id)
        self._cam.stop()
        super().destroy()


# ---------------------------------------------------------------------------
# Plot Panel placeholder
# ---------------------------------------------------------------------------

class PlotPanel(tk.Frame):
    """
    Placeholder for a matplotlib / custom live plot.
    Replace _build_placeholder() body with your real Figure embed.
    """

    def __init__(self, parent, title, color=ACCENT, **kwargs):
        super().__init__(parent, bg=DARK_BG,
                         highlightbackground=color,
                         highlightthickness=1, **kwargs)
        self.title = title
        self.color = color
        self._build_placeholder()

    def _build_placeholder(self):
        tk.Label(self, text=self.title,
                 bg=DARK_BG, fg=self.color,
                 font=FONT_HEADER, anchor="w").pack(
                     fill="x", padx=8, pady=(6, 0))

        canvas_area = tk.Frame(self, bg=PLOT_BG)
        canvas_area.pack(fill="both", expand=True, padx=6, pady=6)

        tk.Label(canvas_area,
                 text="[ Plot placeholder — embed matplotlib Figure here ]",
                 bg=PLOT_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).place(relx=0.5, rely=0.5, anchor="center")

        # TODO: embed your figure, e.g.:
        # self.fig, self.ax = plt.subplots(facecolor=PLOT_BG)
        # self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_area)
        # self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def refresh(self, data):
        """
        TODO: update the plot with new data.
        Called from your real-time polling loop.
        """
        pass


# ---------------------------------------------------------------------------
# Readout Widget (measured sensor value, read-only)
# ---------------------------------------------------------------------------

class SensorReadout(tk.Frame):
    """
    Large read-only display for a measured sensor value.

    Optional features:
      warn_above   — float: show a warning banner above this value
      warn_text    — str:   text for the warning banner
      auto_unit    — dict:  automatic unit switching based on value threshold
                            e.g. {"above": 1.0, "unit_hi": "Torr", "unit_lo": "mTorr",
                                  "scale_lo": 1000}
                            When value < threshold, display value*scale_lo in unit_lo.
      on_refresh   — callable: if provided, a ↺ Refresh button is added that calls it
    """

    def __init__(self, parent, label, unit, color=ACCENT,
                 warn_above=None, warn_text="WARNING",
                 auto_unit=None, on_refresh=None, **kwargs):
        super().__init__(parent, bg=PANEL_BG,
                         highlightbackground=color,
                         highlightthickness=1, **kwargs)
        self.var        = tk.StringVar(value="---")
        self._unit_var  = tk.StringVar(value=unit)
        self.color      = color
        self.warn_above = warn_above
        self.warn_text  = warn_text
        self.auto_unit  = auto_unit     # dict or None
        self.on_refresh = on_refresh
        self._build(label)

    def _build(self, label):
        tk.Label(self, text=label.upper(),
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(pady=(6, 0))

        # Warning banner — hidden until threshold exceeded
        self.warn_lbl = tk.Label(self, text=self.warn_text,
                                 bg=WARNING, fg=DARK_BG,
                                 font=("Courier New", 8, "bold"),
                                 padx=6, pady=1)
        # not packed yet — shown dynamically in update()

        val_row = tk.Frame(self, bg=PANEL_BG)
        val_row.pack(pady=2)

        tk.Label(val_row, textvariable=self.var,
                 bg=PANEL_BG, fg=self.color,
                 font=("Courier New", 22, "bold")).pack(side="left")

        tk.Label(val_row, textvariable=self._unit_var,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left", anchor="s", pady=(0, 4),
                                       padx=(2, 0))

        bottom_row = tk.Frame(self, bg=PANEL_BG)
        bottom_row.pack(pady=(0, 6))

        self.indicator = tk.Canvas(bottom_row, width=8, height=8,
                                   bg=PANEL_BG, highlightthickness=0)
        self.indicator.create_oval(0, 0, 8, 8, fill=TEXT_MUTED, tags="dot")
        self.indicator.pack(side="left", padx=(0, 6))

        # Optional refresh button
        if self.on_refresh is not None:
            tk.Button(
                bottom_row, text="↺ Refresh",
                bg=ACCENT_DIM, fg=ACCENT,
                activebackground=ACCENT, activeforeground=DARK_BG,
                relief="flat", bd=0, cursor="hand2",
                font=("Courier New", 8, "bold"),
                padx=6, pady=1,
                command=self.on_refresh,
            ).pack(side="left")

    def update(self, value):
        """Call from your data loop with the raw value (in base unit)."""
        # Auto unit switching
        if self.auto_unit is not None:
            threshold = self.auto_unit["above"]
            if value < threshold:
                display_val  = value * self.auto_unit.get("scale_lo", 1)
                display_unit = self.auto_unit["unit_lo"]
            else:
                display_val  = value
                display_unit = self.auto_unit["unit_hi"]
            self._unit_var.set(f" {display_unit}")
            self.var.set(f"{display_val:.4g}")
        else:
            self.var.set(f"{value:.4g}")

        self.indicator.itemconfig("dot", fill=self.color)

        # Show / hide the atmosphere warning banner
        if self.warn_above is not None and value > self.warn_above:
            self.warn_lbl.pack(after=self.pack_slaves()[0], pady=(0, 2))
            self.config(highlightbackground=WARNING)
        else:
            self.warn_lbl.pack_forget()
            self.config(highlightbackground=self.color)

    def set_alarm(self, active: bool):
        """Highlight in danger color when out-of-range / no signal."""
        color = DANGER if active else self.color
        self.indicator.itemconfig("dot", fill=color)


# ---------------------------------------------------------------------------
# Control Tab
# ---------------------------------------------------------------------------

def build_control_tab(parent):
    """Construct all widgets for the Control tab."""

    # ── Root grid: left column | right column ───────────────────────────
    parent.columnconfigure(0, weight=3, minsize=520)   # left  (camera + controls)
    parent.columnconfigure(1, weight=2, minsize=380)   # right (plots + readouts)
    parent.rowconfigure(0, weight=1)

    # ════════════════════════════════════════════════════════════════════
    # LEFT COLUMN
    # ════════════════════════════════════════════════════════════════════
    left_col = tk.Frame(parent, bg=DARK_BG)
    left_col.grid(row=0, column=0, sticky="nsew", padx=(8, 4), pady=8)
    left_col.rowconfigure(0, weight=7)   # camera area — taller, roughly square
    left_col.rowconfigure(1, weight=3)   # controls
    left_col.columnconfigure(0, weight=1)

    # ── Camera feed ─────────────────────────────────────────────────────
    cam_frame = make_panel(left_col, title="Camera Feed")
    cam_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 4))

    camera_panel = CameraPanel(cam_frame)
    camera_panel.pack(fill="both", expand=True, padx=6, pady=6)

    # ── Parameter Controls ───────────────────────────────────────────────
    ctrl_frame = make_panel(left_col, title="Parameter Controls")
    ctrl_frame.grid(row=1, column=0, sticky="nsew")

    # --- Define controls ---
    # (label, unit, min, max, default, step)
    CONTROL_DEFS = [
        ("Pressure",             "Torr",  0.0,   760.0,   0.05,  0.001, False),
        ("Power",                "W",     0.0,  2000.0, 100.0, 10.0,   False),
        ("Electron Temperature", "eV",    0.0,    50.0,   5.0,  0.1,   False),
        ("Plasma Density",       "m⁻³",   0.0,  1.0e17,  1e16, 1e15,   True),
    ]

    controls = {}
    for i, (label, unit, mn, mx, dflt, step, sci) in enumerate(CONTROL_DEFS):
        ctrl_frame.rowconfigure(i, weight=1)
        ctrl = ParameterControl(ctrl_frame,
                                label=label, unit=unit,
                                min_val=mn, max_val=mx,
                                default_val=dflt, step=step,
                                sci_notation=sci)
        ctrl.grid(row=i, column=0, sticky="nsew", padx=8, pady=4)
        controls[label] = ctrl

    ctrl_frame.columnconfigure(0, weight=1)

    # TODO: wire controls dict to your hardware/controller module
    # e.g., from controller import PlasmaController
    #        pc = PlasmaController()
    #        for name, widget in controls.items():
    #            widget.value.trace_add("write", lambda *a, n=name: pc.set(n, controls[n].value.get()))

    # ════════════════════════════════════════════════════════════════════
    # RIGHT COLUMN
    # ════════════════════════════════════════════════════════════════════
    right_col = tk.Frame(parent, bg=DARK_BG)
    right_col.grid(row=0, column=1, sticky="nsew", padx=(4, 8), pady=8)
    right_col.columnconfigure(0, weight=1)
    right_col.rowconfigure(0, weight=1)   # Te plot
    right_col.rowconfigure(1, weight=1)   # ne plot
    right_col.rowconfigure(2, weight=1)   # EEDF plot
    right_col.rowconfigure(3, weight=0)   # sensor readouts (fixed height)

    # ── Plot 1: Electron Temperature ────────────────────────────────────
    te_plot = PlotPanel(right_col,
                        title="Electron Temperature  (Tₑ)",
                        color="#58a6ff")
    te_plot.grid(row=0, column=0, sticky="nsew", pady=(0, 4))

    # ── Plot 2: Plasma Density ──────────────────────────────────────────
    ne_plot = PlotPanel(right_col,
                        title="Plasma Density  (nₑ)",
                        color="#3fb950")
    ne_plot.grid(row=1, column=0, sticky="nsew", pady=(0, 4))

    # ── Plot 3: EEDF ────────────────────────────────────────────────────
    eedf_plot = PlotPanel(right_col,
                          title="Electron Energy Distribution Function  (EEDF)",
                          color="#d29922")
    eedf_plot.grid(row=2, column=0, sticky="nsew", pady=(0, 4))

    # ── Sensor Readouts: Measured Power | Measured Pressure ─────────────
    readout_frame = tk.Frame(right_col, bg=DARK_BG)
    readout_frame.grid(row=3, column=0, sticky="ew", pady=(0, 0))
    readout_frame.columnconfigure(0, weight=1)
    readout_frame.columnconfigure(1, weight=1)

    power_readout = SensorReadout(readout_frame,
                                  label="Measured Power",
                                  unit="W",
                                  color=ACCENT)
    power_readout.grid(row=0, column=0, sticky="ew", padx=(0, 4), ipady=4)

    # ── Pressure reader ──────────────────────────────────────────────────
    pressure_reader = PressureReader()   # default: /dev/ttyUSB0 @ 115200
    ok, err = pressure_reader.start()

    if not ok:
        print(f"[Pressure] Serial error: {err}")

    def _reconnect_pressure():
        """Stop the current reader and start a fresh one (called by Refresh button)."""
        pressure_reader.stop()
        pressure_readout.var.set("---")
        pressure_readout.set_alarm(False)
        pressure_readout._unit_var.set(" Torr")
        pressure_readout.warn_lbl.pack_forget()
        pressure_readout.config(highlightbackground=SUCCESS)
        ok2, err2 = pressure_reader.start()
        if not ok2:
            pressure_readout.var.set("ERR")
            pressure_readout.set_alarm(True)
            print(f"[Pressure] Reconnect failed: {err2}")

    pressure_readout = SensorReadout(
        readout_frame,
        label="Measured Pressure",
        unit="Torr",
        color=SUCCESS,
        warn_above=25,
        warn_text="⚠ ATMOSPHERE",
        auto_unit={"above": 1.0, "unit_hi": "Torr",
                   "unit_lo": "mTorr", "scale_lo": 1000},
        on_refresh=_reconnect_pressure,
    )
    pressure_readout.grid(row=0, column=1, sticky="ew", padx=(4, 0), ipady=4)

    if not ok:
        pressure_readout.var.set("ERR")
        pressure_readout.set_alarm(True)

    def _poll_pressure():
        status, _ = pressure_reader.get_status()
        val       = pressure_reader.get_pressure()

        if status == "ok" and val is not None:
            pressure_readout.update(val)
            pressure_readout.set_alarm(False)
            controls["Pressure"].update_measured_value(val)
        elif status == "error":
            pressure_readout.var.set("---")
            pressure_readout.set_alarm(True)
        # "waiting" — leave display as-is until first data arrives

        parent.after(250, _poll_pressure)

    parent.after(250, _poll_pressure)

    return {
        "camera":            camera_panel,
        "controls":          controls,
        "te_plot":           te_plot,
        "ne_plot":           ne_plot,
        "eedf_plot":         eedf_plot,
        "power_readout":     power_readout,
        "pressure_readout":  pressure_readout,
        "pressure_reader":   pressure_reader,
    }


# ---------------------------------------------------------------------------
# Train Tab (reserved)
# ---------------------------------------------------------------------------

def build_train_tab(parent):
    """
    Train tab — reserved for future implementation.
    Replace the placeholder below with your training UI.
    """
    holder = tk.Frame(parent, bg=DARK_BG)
    holder.pack(fill="both", expand=True)

    tk.Label(holder,
             text="TRAIN",
             bg=DARK_BG, fg=BORDER_COLOR,
             font=("Courier New", 48, "bold")).place(relx=0.5, rely=0.45, anchor="center")

    tk.Label(holder,
             text="[ Reserved for training / ML interface ]",
             bg=DARK_BG, fg=TEXT_MUTED,
             font=FONT_LABEL).place(relx=0.5, rely=0.54, anchor="center")


# ---------------------------------------------------------------------------
# Application Root
# ---------------------------------------------------------------------------

class PlasmaGUI(tk.Tk):
    """Main application window."""

    def __init__(self):
        super().__init__()

        self.title("Plasma Control System")
        self.geometry("1280x820")
        self.minsize(900, 600)
        self.configure(bg=DARK_BG)

        self._apply_style()
        self._build_header()
        self._build_tabs()

    # ------------------------------------------------------------------
    def _apply_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure("TNotebook",
                        background=DARK_BG,
                        borderwidth=0)
        style.configure("TNotebook.Tab",
                        background=PANEL_BG,
                        foreground=TEXT_MUTED,
                        padding=[16, 6],
                        font=FONT_HEADER,
                        borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", ACCENT_DIM)],
                  foreground=[("selected", ACCENT)])

    def _build_header(self):
        header = tk.Frame(self, bg=PANEL_BG,
                          highlightbackground=BORDER_COLOR,
                          highlightthickness=1)
        header.pack(fill="x", side="top")

        tk.Label(header,
                 text="⚡  PLASMA CONTROL SYSTEM",
                 bg=PANEL_BG, fg=ACCENT,
                 font=FONT_TITLE,
                 pady=8).pack(side="left", padx=16)

        self.sys_status = tk.Label(header,
                                   text="● IDLE",
                                   bg=PANEL_BG, fg=WARNING,
                                   font=FONT_HEADER)
        self.sys_status.pack(side="right", padx=16)

        # TODO: update sys_status from your system state machine
        # e.g., "● RUNNING" (SUCCESS), "● FAULT" (DANGER), "● IDLE" (WARNING)

    def _build_tabs(self):
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True,
                           padx=8, pady=(4, 8))

        # ── Control tab ─────────────────────────────────────────────────
        control_tab = tk.Frame(self.notebook, bg=DARK_BG)
        self.notebook.add(control_tab, text="  Control  ")
        self.widgets = build_control_tab(control_tab)

        # ── Train tab ────────────────────────────────────────────────────
        train_tab = tk.Frame(self.notebook, bg=DARK_BG)
        self.notebook.add(train_tab, text="  Train  ")
        build_train_tab(train_tab)

        # Clean up hardware on window close
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        """Shut down hardware readers before destroying the window."""
        try:
            self.widgets["pressure_reader"].stop()
        except Exception:
            pass
        self.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = PlasmaGUI()
    app.mainloop()