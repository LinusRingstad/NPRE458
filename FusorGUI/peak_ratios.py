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
target wavelength. This is robust to small calibration shifts.

Usage:
    from peak_ratios import extract_ratios

    ratios = extract_ratios(wavelengths, intensity)
    # returns dict with keys: rat1 … rat5, or None if a peak is out of range
"""

import numpy as np

# Half-width of the search window around each target wavelength (nm)
WINDOW_NM = 5.0

# Peak pair definitions: (numerator_nm, denominator_nm)
PEAK_PAIRS = [
    (770, 380),   # rat1
    (880, 481),   # rat2
    (481, 380),   # rat3
    (770, 725),   # rat4
    (950, 630),   # rat5
]


def _peak_value(wavelengths: np.ndarray, intensity: np.ndarray,
                target_nm: float, window: float = WINDOW_NM) -> float | None:
    """
    Return the maximum intensity within [target_nm - window, target_nm + window].
    Returns None if the target wavelength is outside the array range.
    """
    mask = (wavelengths >= target_nm - window) & (wavelengths <= target_nm + window)
    if not np.any(mask):
        return None
    return float(np.max(intensity[mask]))


def extract_ratios(wavelengths: np.ndarray,
                   intensity: np.ndarray) -> dict | None:
    """
    Compute all 5 peak ratios from a calibrated spectrum.

    Parameters
    ----------
    wavelengths : np.ndarray
        Wavelength axis in nm, same length as intensity.
    intensity : np.ndarray
        Intensity profile (arbitrary units, 0–255 normalised is fine).

    Returns
    -------
    dict with keys 'rat1' … 'rat5'   — all ratios computed successfully
    None                              — one or more peaks not in range
                                        or denominator is zero
    """
    peaks = {}
    all_targets = set()
    for num_nm, den_nm in PEAK_PAIRS:
        all_targets.add(num_nm)
        all_targets.add(den_nm)

    # Pre-compute all needed peak values
    peak_cache = {}
    for nm in all_targets:
        val = _peak_value(wavelengths, intensity, nm)
        if val is None:
            return None     # wavelength out of range entirely
        peak_cache[nm] = val

    ratios = {}
    for i, (num_nm, den_nm) in enumerate(PEAK_PAIRS, start=1):
        den = peak_cache[den_nm]
        if den == 0:
            return None     # avoid divide-by-zero
        ratios[f"rat{i}"] = peak_cache[num_nm] / den

    return ratios
