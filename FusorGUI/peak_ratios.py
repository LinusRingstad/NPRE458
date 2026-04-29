"""
peak_ratios.py
==============
Extracts the 5 spectral peak ratios used as features for the plasma neural
network model.

Peak definitions (wavelength in nm):
    rat1 = 770 / 380
    rat2 = 880 / 481
    rat3 = 481 / 380
    rat4 = 770 / 725
    rat5 = 950 / 630

Each "peak" value is the maximum intensity in a ±WINDOW_NM band around the
target wavelength.

Usage:
    from peak_ratios import extract_ratios

    ratios, reason = extract_ratios(wavelengths, intensity)
    if ratios is None:
        print("Skipping:", reason)
    else:
        # use ratios["rat1"] … ratios["rat5"]
"""

import numpy as np

# Half-width of the search window around each target wavelength (nm)
WINDOW_NM = 8.0

# Minimum signal threshold — if the overall spectrum max is below this
# fraction of 255, the plasma is not yet lit / spectrometer is warming up
MIN_SIGNAL_FRACTION = 0.05   # 5% of full scale

# Peak pair definitions: (numerator_nm, denominator_nm)
PEAK_PAIRS = [
    (770, 380),   # rat1
    (880, 481),   # rat2
    (481, 380),   # rat3
    (770, 725),   # rat4
    (950, 630),   # rat5
]


def _peak_value(wavelengths: np.ndarray, intensity: np.ndarray,
                target_nm: float, window: float = WINDOW_NM):
    """
    Return the maximum intensity within [target_nm ± window] nm.
    Returns (value, True) on success or (None, False) if out of range.
    """
    mask = (wavelengths >= target_nm - window) & (wavelengths <= target_nm + window)
    if not np.any(mask):
        return None, False
    return float(np.max(intensity[mask])), True


def extract_ratios(wavelengths: np.ndarray,
                   intensity: np.ndarray):
    """
    Compute all 5 peak ratios from a calibrated spectrum.

    Parameters
    ----------
    wavelengths : np.ndarray  Wavelength axis in nm.
    intensity   : np.ndarray  Normalised intensity 0–255.

    Returns
    -------
    (dict, None)   — success: dict with keys 'rat1' … 'rat5'
    (None, str)    — failure: human-readable reason string
    """
    # --- 1. Minimum signal guard (plasma not lit / still warming up) ---
    max_signal = float(np.max(intensity))
    if max_signal < MIN_SIGNAL_FRACTION * 255:
        return None, (f"Signal too weak (max={max_signal:.1f}/255). "
                      "Plasma may not be lit or spectrometer still warming up.")

    # --- 2. Collect all unique target wavelengths ---
    all_targets = set()
    for num_nm, den_nm in PEAK_PAIRS:
        all_targets.add(num_nm)
        all_targets.add(den_nm)

    peak_cache = {}
    for nm in sorted(all_targets):
        val, found = _peak_value(wavelengths, intensity, nm)
        if not found:
            wl_min = float(wavelengths.min())
            wl_max = float(wavelengths.max())
            return None, (f"{nm} nm is outside spectrometer range "
                          f"({wl_min:.0f}–{wl_max:.0f} nm).")
        peak_cache[nm] = val

    # --- 3. Compute ratios ---
    ratios = {}
    for i, (num_nm, den_nm) in enumerate(PEAK_PAIRS, start=1):
        den = peak_cache[den_nm]
        if den < 1e-6:
            return None, (f"Denominator peak at {den_nm} nm is zero "
                          f"(rat{i}). Spectrum may still be calibrating.")
        ratios[f"rat{i}"] = peak_cache[num_nm] / den

    return ratios, None