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
    A single adjustable parameter row with:
      - Enable/disable toggle
      - Label
      - Decrement button  |  current value display  |  Increment button
      - Unit label
      - Setpoint entry
    """

    def __init__(self, parent, label, unit, min_val, max_val,
                 default_val, step=1.0, **kwargs):
        super().__init__(parent, bg=PANEL_BG, **kwargs)

        self.label     = label
        self.unit      = unit
        self.min_val   = min_val
        self.max_val   = max_val
        self.step      = step
        self.enabled   = tk.BooleanVar(value=False)
        self.value     = tk.DoubleVar(value=default_val)
        self.setpoint  = tk.DoubleVar(value=default_val)

        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        # ── Row 0: toggle + parameter name ──────────────────────────────
        header = tk.Frame(self, bg=PANEL_BG)
        header.grid(row=0, column=0, columnspan=7, sticky="w", pady=(4, 2))

        self.toggle_btn = tk.Checkbutton(
            header,
            variable=self.enabled,
            command=self._on_toggle,
            bg=PANEL_BG,
            activebackground=PANEL_BG,
            selectcolor=ACCENT_DIM,
            fg=SUCCESS,
            activeforeground=SUCCESS,
            font=FONT_LABEL,
        )
        self.toggle_btn.pack(side="left")

        tk.Label(header, text=self.label,
                 bg=PANEL_BG, fg=TEXT_PRIMARY,
                 font=FONT_HEADER).pack(side="left", padx=(2, 0))

        self.status_lbl = tk.Label(header, text="OFF",
                                   bg=PANEL_BG, fg=DANGER,
                                   font=FONT_SMALL)
        self.status_lbl.pack(side="left", padx=(8, 0))

        # ── Row 1: decrement | value | increment | unit ─────────────────
        ctrl = tk.Frame(self, bg=PANEL_BG)
        ctrl.grid(row=1, column=0, columnspan=7, sticky="w", padx=4, pady=2)

        self.dec_btn = tk.Button(
            ctrl, text="◀", width=2,
            bg=ACCENT_DIM, fg=ACCENT,
            activebackground=ACCENT, activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=FONT_LABEL,
            command=self._decrement,
            state="disabled",
        )
        self.dec_btn.pack(side="left", padx=(0, 2))

        self.val_display = tk.Label(
            ctrl,
            textvariable=self.value,
            width=8,
            bg=DARK_BG, fg=TEXT_PRIMARY,
            font=FONT_VALUE,
            relief="flat",
            anchor="center",
        )
        self.val_display.pack(side="left")

        self.inc_btn = tk.Button(
            ctrl, text="▶", width=2,
            bg=ACCENT_DIM, fg=ACCENT,
            activebackground=ACCENT, activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=FONT_LABEL,
            command=self._increment,
            state="disabled",
        )
        self.inc_btn.pack(side="left", padx=(2, 6))

        tk.Label(ctrl, text=self.unit,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left")

        # ── Row 2: setpoint entry ────────────────────────────────────────
        sp_frame = tk.Frame(self, bg=PANEL_BG)
        sp_frame.grid(row=2, column=0, columnspan=7, sticky="w", padx=4, pady=(0, 6))

        tk.Label(sp_frame, text="Setpoint:",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(side="left")

        self.sp_entry = tk.Entry(
            sp_frame,
            textvariable=self.setpoint,
            width=8,
            bg=DARK_BG, fg=ACCENT,
            insertbackground=ACCENT,
            relief="flat",
            font=FONT_MONO,
            state="disabled",
        )
        self.sp_entry.pack(side="left", padx=(4, 2))

        tk.Label(sp_frame, text=self.unit,
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(side="left")

        tk.Button(
            sp_frame, text="SET",
            bg=ACCENT_DIM, fg=ACCENT,
            activebackground=ACCENT, activeforeground=DARK_BG,
            relief="flat", bd=0, cursor="hand2",
            font=FONT_SMALL,
            command=self._apply_setpoint,
            state="disabled",
            name="set_btn",
        ).pack(side="left", padx=(4, 0))
        self.set_btn = sp_frame.children["set_btn"]

        # Separator
        sep = tk.Frame(self, bg=BORDER_COLOR, height=1)
        sep.grid(row=3, column=0, columnspan=7, sticky="ew", pady=(0, 2))

    # ------------------------------------------------------------------
    def _on_toggle(self):
        enabled = self.enabled.get()
        state   = "normal" if enabled else "disabled"
        color   = SUCCESS if enabled else DANGER
        label   = "ON" if enabled else "OFF"

        self.status_lbl.config(text=label, fg=color)
        self.dec_btn.config(state=state)
        self.inc_btn.config(state=state)
        self.sp_entry.config(state=state)
        self.set_btn.config(state=state)

        # TODO: call your hardware enable/disable function here
        # e.g., hardware.set_control_enabled(self.label, enabled)

    def _increment(self):
        new = min(self.value.get() + self.step, self.max_val)
        self.value.set(round(new, 4))
        # TODO: send updated value to hardware
        # e.g., hardware.set_parameter(self.label, self.value.get())

    def _decrement(self):
        new = max(self.value.get() - self.step, self.min_val)
        self.value.set(round(new, 4))
        # TODO: send updated value to hardware

    def _apply_setpoint(self):
        clamped = max(self.min_val, min(self.max_val, self.setpoint.get()))
        self.value.set(round(clamped, 4))
        self.setpoint.set(round(clamped, 4))
        # TODO: send setpoint to controller
        # e.g., controller.set_setpoint(self.label, clamped)

    # ------------------------------------------------------------------
    def update_measured_value(self, val):
        """Call this from your real-time data loop to update the display."""
        self.value.set(round(val, 4))


# ---------------------------------------------------------------------------
# Camera Panel placeholder
# ---------------------------------------------------------------------------

class CameraPanel(tk.Frame):
    """
    Placeholder for live camera feed.
    Replace the interior with your CameraPanel implementation.
    """

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=DARK_BG,
                         highlightbackground=BORDER_COLOR,
                         highlightthickness=1, **kwargs)
        self._build_placeholder()

    def _build_placeholder(self):
        inner = tk.Frame(self, bg=DARK_BG)
        inner.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(inner, text="⬛", font=("Courier New", 48),
                 bg=DARK_BG, fg=BORDER_COLOR).pack()
        tk.Label(inner, text="CAMERA FEED",
                 bg=DARK_BG, fg=TEXT_MUTED,
                 font=FONT_HEADER).pack()
        tk.Label(inner, text="[ Replace with CameraPanel implementation ]",
                 bg=DARK_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(pady=(4, 0))

    def update_frame(self, image):
        """
        TODO: implement live frame updates.
        Expected signature: image is a PIL Image or numpy array.
        e.g.:
            photo = ImageTk.PhotoImage(image)
            self.image_label.config(image=photo)
            self.image_label.image = photo
        """
        pass


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
    """

    def __init__(self, parent, label, unit, color=ACCENT, **kwargs):
        super().__init__(parent, bg=PANEL_BG,
                         highlightbackground=color,
                         highlightthickness=1, **kwargs)
        self.var   = tk.StringVar(value="---")
        self.color = color
        self._build(label, unit)

    def _build(self, label, unit):
        tk.Label(self, text=label.upper(),
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(pady=(6, 0))

        val_row = tk.Frame(self, bg=PANEL_BG)
        val_row.pack(pady=2)

        tk.Label(val_row, textvariable=self.var,
                 bg=PANEL_BG, fg=self.color,
                 font=("Courier New", 22, "bold")).pack(side="left")

        tk.Label(val_row, text=f" {unit}",
                 bg=PANEL_BG, fg=TEXT_MUTED,
                 font=FONT_LABEL).pack(side="left", anchor="s", pady=(0, 4))

        self.indicator = tk.Canvas(self, width=8, height=8,
                                   bg=PANEL_BG, highlightthickness=0)
        self.indicator.create_oval(0, 0, 8, 8, fill=TEXT_MUTED, tags="dot")
        self.indicator.pack(pady=(0, 6))

    def update(self, value):
        """Call from your data loop: readout.update(measured_value)"""
        self.var.set(f"{value:.2f}")
        self.indicator.itemconfig("dot", fill=self.color)

    def set_alarm(self, active: bool):
        """Highlight in warning color when out-of-range."""
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
    left_col.rowconfigure(0, weight=5)   # camera area gets most vertical space
    left_col.rowconfigure(1, weight=2)   # controls
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
        ("Pressure",             "mTorr", 0.0,   500.0,  50.0,  1.0),
        ("Power",                "W",     0.0,  2000.0, 100.0, 10.0),
        ("Electron Temperature", "eV",    0.0,    50.0,   5.0,  0.1),
        ("Plasma Density",       "m⁻³",   0.0,  1.0e17,  1e16, 1e15),
    ]

    controls = {}
    for (label, unit, mn, mx, dflt, step) in CONTROL_DEFS:
        ctrl = ParameterControl(ctrl_frame,
                                label=label, unit=unit,
                                min_val=mn, max_val=mx,
                                default_val=dflt, step=step)
        ctrl.pack(fill="x", padx=8, pady=(4, 0))
        controls[label] = ctrl

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

    power_readout    = SensorReadout(readout_frame,
                                     label="Measured Power",
                                     unit="W",
                                     color=ACCENT)
    power_readout.grid(row=0, column=0, sticky="ew", padx=(0, 4), ipady=4)

    pressure_readout = SensorReadout(readout_frame,
                                     label="Measured Pressure",
                                     unit="mTorr",
                                     color=SUCCESS)
    pressure_readout.grid(row=0, column=1, sticky="ew", padx=(4, 0), ipady=4)

    # TODO: start your real-time data polling loop here, e.g.:
    # def poll():
    #     data = hardware.read_sensors()
    #     power_readout.update(data["power"])
    #     pressure_readout.update(data["pressure"])
    #     te_plot.refresh(data["te_series"])
    #     ne_plot.refresh(data["ne_series"])
    #     eedf_plot.refresh(data["eedf"])
    #     controls["Electron Temperature"].update_measured_value(data["te"])
    #     controls["Plasma Density"].update_measured_value(data["ne"])
    #     parent.after(200, poll)   # refresh every 200 ms
    # parent.after(200, poll)

    return {
        "camera":        camera_panel,
        "controls":      controls,
        "te_plot":       te_plot,
        "ne_plot":       ne_plot,
        "eedf_plot":     eedf_plot,
        "power_readout": power_readout,
        "pressure_readout": pressure_readout,
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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = PlasmaGUI()
    app.mainloop()