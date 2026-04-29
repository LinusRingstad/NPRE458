"""
plasma_model.py
===============
PyTorch model loader and inference engine for the plasma neural network.

Features:  ['radius', 'rat1', 'rat2', 'rat3', 'rat4', 'rat5']  (6 inputs)
Targets:   ['ne', 'te']                                          (2 outputs)

The loader handles two common save formats:
  A) torch.save(model, path)          → full model object (preferred)
  B) torch.save(model.state_dict(), path) → OrderedDict of weights only
     → requires the PlasmaNet architecture defined below to be correct

If you saved with option B, fill in the hidden layer sizes in PlasmaNet
to match your training script exactly, then this will work automatically.

Usage:
    predictor = PlasmaPredictor("plasma_model_ratonly.pth")
    ok, err = predictor.load()
    profile  = predictor.predict_radial_profile(ratios)
    # profile: { "r": ndarray(m), "ne": ndarray, "te": ndarray }
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


# ---------------------------------------------------------------------------
# Network architecture
# ---------------------------------------------------------------------------
# *** IMPORTANT ***
# If your .pth file was saved with torch.save(model.state_dict(), ...),
# this architecture MUST match your training script exactly.
# Update the hidden layer sizes below to match yours.
# If you saved with torch.save(model, ...) this class is not used.
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


class PlasmaPredictor:
    """
    Wraps the saved PyTorch plasma model.
    Handles both full-object saves and state-dict saves.
    """

    def __init__(self, model_path: str = "plasma_model_ratonly.pth",
                 n_radial: int = N_RADIAL):
        self.model_path = Path(model_path)
        self.n_radial   = n_radial
        self._model     = None
        self._loaded    = False

        self.r_points = np.linspace(R_WALL, R_CATHODE, n_radial)

    # ------------------------------------------------------------------
    def load(self):
        """
        Load model weights from disk.
        Returns (True, None) on success or (False, error_string) on failure.
        """
        if not TORCH_AVAILABLE:
            return False, ("PyTorch is not installed.\n"
                           "Install with:  pip install torch --index-url "
                           "https://download.pytorch.org/whl/cpu")

        # Resolve path
        path = self.model_path
        if not path.exists():
            alt = Path(__file__).resolve().parent / path
            if alt.exists():
                path = alt
            else:
                return False, f"Model file not found: {path}"

        try:
            checkpoint = torch.load(path,
                                     map_location=torch.device("cpu"),
                                     weights_only=False)
        except Exception as exc:
            return False, f"torch.load failed: {exc}"

        # --- Determine save format ---
        if isinstance(checkpoint, nn.Module):
            # Format A: full model object saved directly
            self._model = checkpoint

        elif isinstance(checkpoint, dict):
            # Format B: state dict — instantiate architecture then load weights
            net = PlasmaMLP()
            try:
                net.load_state_dict(checkpoint)
            except RuntimeError as exc:
                return False, (
                    f"State dict load failed: {exc}\n"
                    "Check that PlasmaMLP in plasma_model.py matches "
                    "your training script exactly."
                )
            self._model = net

        else:
            return False, (f"Unrecognised checkpoint type: {type(checkpoint)}. "
                           "Expected nn.Module or state dict.")

        self._model.eval()
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
        dict  { 'r': ndarray(m), 'ne': ndarray, 'te': ndarray }
        None  if model not loaded or inference fails
        """
        if not self._loaded:
            return None

        try:
            rat_row = [ratios[f"rat{i}"] for i in range(1, 6)]
            rows = [[r] + rat_row for r in self.r_points]

            X = torch.tensor(rows, dtype=torch.float32)

            with torch.no_grad():
                preds = self._model(X).numpy()   # (N_RADIAL, 2)

            return {
                "r":  self.r_points.copy(),
                "ne": preds[:, 0],
                "te": preds[:, 1],
            }

        except Exception as exc:
            print(f"[PlasmaModel] Inference error: {exc}")
            return None