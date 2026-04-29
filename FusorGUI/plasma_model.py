"""
plasma_model.py
===============
PyTorch model loader and inference engine for the plasma neural network.

The model takes features  ['radius', 'rat1', 'rat2', 'rat3', 'rat4', 'rat5']
and returns targets       ['ne', 'te'].

Typical usage — called once every 20 seconds by the spectrometer aggregation
loop:

    from plasma_model import PlasmaPredictor
    from peak_ratios  import extract_ratios

    predictor = PlasmaPredictor("plasma_model_ratonly.pth")
    ok, err = predictor.load()

    ratios = extract_ratios(wavelengths, intensity)   # dict rat1..rat5
    if ratios:
        profile = predictor.predict_radial_profile(ratios)
        # profile is a dict:
        # {
        #   "r":   np.ndarray   radial positions (m), wall → cathode
        #   "ne":  np.ndarray   electron density (m⁻³)
        #   "te":  np.ndarray   electron temperature (eV)
        # }
"""

import numpy as np
from pathlib import Path

# Physical geometry
R_CATHODE = 46.29e-3 / 2   # m — cathode sphere radius
R_WALL    = 280e-3   / 2   # m — chamber wall radius

# Number of radial points to evaluate across the profile
N_RADIAL  = 60

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class PlasmaPredictor:
    """
    Wraps the saved PyTorch plasma model.

    Call load() once at startup, then predict_radial_profile() as often
    as needed. All inference runs on CPU so no GPU is required on the Pi.
    """

    def __init__(self, model_path: str = "plasma_model_ratonly.pth",
                 n_radial: int = N_RADIAL):
        self.model_path = Path(model_path)
        self.n_radial   = n_radial
        self._model     = None
        self._loaded    = False

        # Radial sweep from wall inward to cathode (wall first for plotting)
        self.r_points = np.linspace(R_WALL, R_CATHODE, n_radial)

    # ------------------------------------------------------------------
    def load(self):
        """
        Load the model weights from disk.
        Returns (True, None) on success or (False, error_string) on failure.
        """
        if not TORCH_AVAILABLE:
            return False, ("PyTorch is not installed.\n"
                           "Install with:  pip install torch --index-url "
                           "https://download.pytorch.org/whl/cpu")

        if not self.model_path.exists():
            # Try relative to this file's directory
            alt = Path(__file__).resolve().parent / self.model_path
            if alt.exists():
                self.model_path = alt
            else:
                return False, f"Model file not found: {self.model_path}"

        try:
            # torch.load with weights_only=True is safe for our own models
            self._model = torch.load(
                self.model_path,
                map_location=torch.device("cpu"),
                weights_only=False,
            )
            self._model.eval()
            self._loaded = True
            return True, None
        except Exception as exc:
            return False, str(exc)

    # ------------------------------------------------------------------
    def predict_radial_profile(self, ratios: dict) -> dict | None:
        """
        Run inference over the full radial sweep for a given set of
        peak ratios.

        Parameters
        ----------
        ratios : dict
            Keys 'rat1' … 'rat5' as returned by peak_ratios.extract_ratios().

        Returns
        -------
        dict  { 'r': ndarray(m), 'ne': ndarray, 'te': ndarray }
        None  if model not loaded or inference fails
        """
        if not self._loaded:
            return None

        try:
            # Build input tensor: one row per radial position
            # Feature order: ['radius', 'rat1', 'rat2', 'rat3', 'rat4', 'rat5']
            rat_row = [ratios[f"rat{i}"] for i in range(1, 6)]
            rows = []
            for r in self.r_points:
                rows.append([r] + rat_row)

            X = torch.tensor(rows, dtype=torch.float32)

            with torch.no_grad():
                preds = self._model(X).numpy()   # shape (N_RADIAL, 2)

            return {
                "r":  self.r_points.copy(),
                "ne": preds[:, 0],   # electron density
                "te": preds[:, 1],   # electron temperature
            }

        except Exception as exc:
            print(f"[PlasmaModel] Inference error: {exc}")
            return None
