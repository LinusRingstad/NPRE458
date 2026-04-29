"""
plasma_model.py
===============
PyTorch model loader and inference engine for the plasma neural network.

Features:  ['radius', 'rat1', 'rat2', 'rat3', 'rat4', 'rat5']  (6 inputs)
Targets:   ['ne', 'te']                                          (2 outputs)

IMPORTANT: The model was trained on StandardScaler-normalised inputs and
outputs. Both scalers must be loaded alongside the model weights:
    scaler_x_ratonly.pkl  — input scaler  (6 features)
    scaler_y_ratonly.pkl  — output scaler (ne, te)

Without inverse-transforming the output the predictions are in normalised
space (small numbers, often negative) which is not physically meaningful.

Usage:
    predictor = PlasmaPredictor()
    ok, err = predictor.load()
    profile  = predictor.predict_radial_profile(ratios)
    # { "r": ndarray(m), "ne": ndarray(m⁻³), "te": ndarray(eV) }
"""

import numpy as np
from pathlib import Path

R_CATHODE = 46.29e-3 / 2   # m — cathode sphere radius
R_WALL    = 280e-3   / 2   # m — chamber wall radius
N_RADIAL  = 60             # radial evaluation points

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import joblib
    JOBLIB_AVAILABLE = True
except ImportError:
    JOBLIB_AVAILABLE = False


# ---------------------------------------------------------------------------
# Network architecture (must match training script exactly)
# ---------------------------------------------------------------------------

class PlasmaMLP(nn.Module):
    """
    input:  radius, rat1, rat2, rat3, rat4, rat5  (6 features)
    output: ne, Te                                 (2 features)
    """
    def __init__(self, input_size=6, hidden_size=128, output_size=2):
        super(PlasmaMLP, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size // 2, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size // 2),
            nn.ReLU(),
            nn.Linear(hidden_size // 2, output_size)
        )

    def forward(self, x):
        return self.network(x)


# ---------------------------------------------------------------------------
# Predictor
# ---------------------------------------------------------------------------

class PlasmaPredictor:
    """
    Wraps the saved PyTorch plasma model + sklearn StandardScalers.

    Files needed (all in the same directory as this script):
        plasma_model_ratonly.pth    — model state dict
        scaler_x_ratonly.pkl        — input scaler
        scaler_y_ratonly.pkl        — output scaler
    """

    def __init__(self,
                 model_path:    str = "plasma_model_ratonly.pth",
                 scaler_x_path: str = "scaler_x_ratonly.pkl",
                 scaler_y_path: str = "scaler_y_ratonly.pkl",
                 n_radial:      int = N_RADIAL):

        self._base      = Path(__file__).resolve().parent
        self.model_path    = self._resolve(model_path)
        self.scaler_x_path = self._resolve(scaler_x_path)
        self.scaler_y_path = self._resolve(scaler_y_path)
        self.n_radial      = n_radial

        self._model    = None
        self._scaler_x = None
        self._scaler_y = None
        self._loaded   = False

        self.r_points = np.linspace(R_WALL, R_CATHODE, n_radial)

    def _resolve(self, p):
        path = Path(p)
        return path if path.is_absolute() else self._base / path

    # ------------------------------------------------------------------
    def load(self):
        """
        Load model weights and both scalers from disk.
        Returns (True, None) on success or (False, error_string) on failure.
        """
        if not TORCH_AVAILABLE:
            return False, ("PyTorch is not installed.\n"
                           "Install with:  pip install torch --index-url "
                           "https://download.pytorch.org/whl/cpu")

        if not JOBLIB_AVAILABLE:
            return False, ("joblib is not installed.\n"
                           "Install with:  pip install joblib")

        # --- Load model weights ---
        if not self.model_path.exists():
            return False, f"Model file not found: {self.model_path}"

        try:
            checkpoint = torch.load(self.model_path,
                                     map_location=torch.device("cpu"),
                                     weights_only=False)
        except Exception as exc:
            return False, f"torch.load failed: {exc}"

        if isinstance(checkpoint, nn.Module):
            self._model = checkpoint
        elif isinstance(checkpoint, dict):
            net = PlasmaMLP()
            try:
                net.load_state_dict(checkpoint)
            except RuntimeError as exc:
                return False, f"State dict load failed: {exc}"
            self._model = net
        else:
            return False, f"Unrecognised checkpoint type: {type(checkpoint)}"

        self._model.eval()

        # --- Load scalers ---
        for path, attr, name in [
            (self.scaler_x_path, "_scaler_x", "scaler_x"),
            (self.scaler_y_path, "_scaler_y", "scaler_y"),
        ]:
            if not path.exists():
                return False, (f"{name} file not found: {path}\n"
                               "Make sure scaler_x_ratonly.pkl and "
                               "scaler_y_ratonly.pkl are in the same folder.")
            try:
                setattr(self, attr, joblib.load(path))
            except Exception as exc:
                return False, f"Failed to load {name}: {exc}"

        self._loaded = True
        return True, None

    # ------------------------------------------------------------------
    def predict_radial_profile(self, ratios: dict):
        """
        Run inference over the full radial sweep.

        Parameters
        ----------
        ratios : dict  Keys 'rat1'…'rat5' from peak_ratios.extract_ratios().

        Returns
        -------
        dict  { 'r': ndarray(m), 'ne': ndarray(m⁻³), 'te': ndarray(eV) }
        None  if model not loaded or inference fails
        """
        if not self._loaded:
            return None

        try:
            rat_row = [ratios[f"rat{i}"] for i in range(1, 6)]

            # Build raw feature array: shape (N_RADIAL, 6)
            rows = np.array([[r] + rat_row for r in self.r_points],
                            dtype=np.float64)

            # Scale inputs the same way the training script did
            rows_scaled = self._scaler_x.transform(rows)

            X = torch.tensor(rows_scaled, dtype=torch.float32)

            with torch.no_grad():
                preds_scaled = self._model(X).numpy()   # (N_RADIAL, 2)

            # Inverse-transform outputs back to physical units
            # scaler_y was fit on [ne, te] columns in that order
            preds_physical = self._scaler_y.inverse_transform(preds_scaled)

            return {
                "r":  self.r_points.copy(),
                "ne": preds_physical[:, 0],   # m⁻³
                "te": preds_physical[:, 1],   # eV
            }

        except Exception as exc:
            print(f"[PlasmaModel] Inference error: {exc}")
            return None