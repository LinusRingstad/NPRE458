# Setup Model Functions

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv, os, warnings
#from IPython.display import display, HTML
import json

warnings.filterwarnings("ignore", category=RuntimeWarning)

plt.rcParams.update({
    "figure.dpi": 120,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

# ── Physical constants ─────────────────────────────────────────────────────
e_charge = 1.602e-19      # C
eps0     = 8.854e-12      # F/m
kb       = 1.381e-23      # J/K
Tgas     = 300.0          # K  (neutral gas temperature)

# ── Fixed geometry ─────────────────────────────────────────────────────────
R_CATHODE = 46.29e-3 / 2   # m  — cathode sphere radius
R_WALL    = 280e-3   / 2   # m  — chamber wall radius


# ╔══════════════════════════════════════════════╗
# ║        SET YOUR EXPERIMENT CONDITIONS        ║
# ╠══════════════════════════════════════════════╣

CURRENT_mA = 20.0     # discharge current  [mA]
VOLTAGE_V  = 550.0    # cathode voltage magnitude  [V]  (sign applied automatically)
PRESSURE_TORR = 40E-3   # gas pressure  [Torr]

# ── Solver resolution ──────────────────────────
NR        = 400    # radial grid points (increase for finer resolution)
N_OUTER   = 800    # outer iterations   (increase if not converged)

# ╚══════════════════════════════════════════════╝

# Derived
I_A = CURRENT_mA * 1e-3
V0  = -abs(VOLTAGE_V)    # cathode is always negative

# -- Air plasma transport and rate coefficients ---------------------------------

MU_I0         = 1.5e-4   # m^2/(V*s) ion mobility at 1 Torr
MU_E_REF      = 0.10     # m^2/(V*s) electron mobility at 1 Torr
SEC_ELEC_COEF = 0.01     # secondary electron emission coefficient gamma
K_REC         = 2.0e-16  # m^3/s effective e-i recombination

# Numerical controls (robust baseline)
RELAX_N       = 0.020
RELAX_TE      = 0.050
ALPHA_PHI     = 0.28     # stronger coupling to corrected potential

# Physical / numerical safeguards
TE_MIN_EV      = 1.0
TE_MAX_EV      = 4.0
CHARGE_CLIP_M3 = 2.0e15
NE_MIN_M3      = 1e10
NE_MAX_M3      = 2e18
NI_MIN_M3      = 1e10
NI_MAX_M3      = 2e18


def gas_density(p_torr):
    """Neutral number density [m^-3] from pressure in Torr."""
    return (p_torr * 133.322) / (kb * Tgas)


def ionization_rate(Te_eV):
    """Effective ionisation rate coefficient k_iz [m^3/s] for air."""
    Te   = np.maximum(Te_eV, 0.3)
    k_N2 = 1.8e-14 * np.exp(-15.58 / Te) * (Te / 15.58)**0.5
    k_O2 = 2.3e-14 * np.exp(-12.07 / Te) * (Te / 12.07)**0.5
    return 0.80 * k_N2 + 0.20 * k_O2


def attachment_rate(Te_eV, N):
    """3-body electron attachment rate for O2-rich air [m^3/s]."""
    Te = np.maximum(Te_eV, 0.1)
    return 2.0e-43 * N * np.exp(-0.5 * Te)


def electron_mobility(E_field, p_torr):
    """Pressure- and field-dependent electron mobility [m^2/(V*s)]."""
    mu0  = MU_E_REF / p_torr
    N    = gas_density(p_torr)
    norm = np.abs(E_field) / N
    return mu0 / (1.0 + norm / 5e-20)


def ion_mobility(p_torr):
    """Pressure-scaled ion mobility [m^2/(V*s)]."""
    return MU_I0 / p_torr


def smooth_1d(arr, n_pass=1):
    """Small conservative smoother for grid-scale noise."""
    out = arr.copy()
    for _ in range(n_pass):
        out[1:-1] = 0.2 * out[:-2] + 0.6 * out[1:-1] + 0.2 * out[2:]
    return out


def solve_tridiagonal(a, b, c, d):
    """Thomas algorithm for tridiagonal systems."""
    n = len(b)
    cp = np.zeros(n - 1)
    dp = np.zeros(n)

    cp[0] = c[0] / b[0]
    dp[0] = d[0] / b[0]

    for i in range(1, n - 1):
        den = b[i] - a[i - 1] * cp[i - 1]
        cp[i] = c[i] / den
        dp[i] = (d[i] - a[i - 1] * dp[i - 1]) / den

    den_last = b[-1] - a[-1] * cp[-1]
    dp[-1] = (d[-1] - a[-1] * dp[-2]) / den_last

    x = np.zeros(n)
    x[-1] = dp[-1]
    for i in range(n - 2, -1, -1):
        x[i] = dp[i] - cp[i] * x[i + 1]

    return x


def solve_poisson_correction(r, dr, ne, ni, V0):
    """Tridiagonal spherical Poisson solve with clipped space charge."""
    Nr = len(r)
    n = Nr - 2
    A = 1.0 / dr**2
    ri = r[1:-1]
    Bv = 1.0 / (ri * dr)

    lower = -(A - Bv[1:])
    diag  = np.full(n, 2 * A)
    upper = -(A + Bv[:-1])

    rho = e_charge * np.clip(ni[1:-1] - ne[1:-1], -CHARGE_CLIP_M3, CHARGE_CLIP_M3)
    rhs = rho / eps0

    rhs[0]  -= (-(A - Bv[0])) * V0

    phi_i = solve_tridiagonal(lower, diag, upper, rhs)

    phi = np.zeros(Nr)
    phi[0] = V0
    phi[-1] = 0.0
    phi[1:-1] = np.clip(phi_i, V0, 0.0)
    return smooth_1d(phi, n_pass=1)


def solve_plasma(I_A, V0, p_torr, Nr=NR, n_outer=N_OUTER, verbose=True):
    """
    Robust 1-D radial plasma baseline.
    Prioritizes smooth, physical-looking profiles over aggressive sheath sharpness.
    """
    rc = R_CATHODE
    rw = R_WALL
    dr = (rw - rc) / (Nr - 1)
    r  = np.linspace(rc, rw, Nr)
    s  = (r - rc) / (rw - rc)

    N_gas = gas_density(p_torr)
    mu_i  = ion_mobility(p_torr)

    # Initial conditions
    phi = V0 * (1.0 - s)
    Te  = 1.1 + 0.6 * np.exp(-((s - 0.20)**2) / 0.06)

    E_est = abs(V0) / (rw - rc)
    n0    = np.clip(abs(I_A) / (4*np.pi*rc**2 * e_charge * mu_i * E_est + 1e-30),
                    1e12, 5e16)
    ne    = np.clip(n0 * (1.0 - 0.55 * s**0.8), NE_MIN_M3, NE_MAX_M3)
    ni    = np.clip(ne * 1.01, NI_MIN_M3, NI_MAX_M3)

    for outer in range(n_outer):

        # 1) Potential: mostly-Laplace + controlled Poisson correction.
        phi_lap = V0 * (1.0 - s)
        phi_corr = solve_poisson_correction(r, dr, ne, ni, V0)
        phi_target = 0.70 * phi_lap + 0.30 * phi_corr
        phi = (1.0 - ALPHA_PHI) * phi + ALPHA_PHI * phi_target
        phi[0] = V0
        phi[-1] = 0.0
        phi = np.clip(smooth_1d(phi, n_pass=1), V0, 0.0)
        phi[0] = V0
        phi[-1] = 0.0

        # 2) Electric field
        E_field = np.zeros(Nr)
        E_field[1:-1] = -(phi[2:] - phi[:-2]) / (2 * dr)
        E_field[0]    = E_field[1]
        E_field[-1]   = E_field[-2]

        # 3) Electron temperature from E/N plus cathode-heating envelope.
        N = gas_density(p_torr)
        EN = np.clip(np.abs(E_field) / N * 1e21, 0.3, 220.0)
        Te_env = 0.35 * np.exp(-4.0 * s)
        Te_new = np.clip(1.0 + 0.012 * EN**0.55 + Te_env, TE_MIN_EV, TE_MAX_EV)
        Te_new = smooth_1d(Te_new, n_pass=2)
        Te = np.clip(Te + RELAX_TE * (Te_new - Te), TE_MIN_EV, TE_MAX_EV)

        # 4) Transport / chemistry
        mu_e = electron_mobility(E_field, p_torr)
        De   = mu_e * np.maximum(Te, 0.1)

        S_ion = ionization_rate(Te)        * N_gas * ne
        S_att = attachment_rate(Te, N_gas) * N_gas * ne
        S_rec = K_REC * ne * ni

        ne_new = ne.copy()
        ni_new = ni.copy()

        # 5) Explicit update with local pseudo-time-step
        for i in range(1, Nr - 1):
            Ge_ip = -mu_e[i] * E_field[i] * 0.5 * (ne[i] + ne[i+1]) - De[i] * (ne[i+1] - ne[i]) / dr
            Ge_im = -mu_e[i] * E_field[i] * 0.5 * (ne[i] + ne[i-1]) - De[i] * (ne[i] - ne[i-1]) / dr
            div_Ge = (r[i+1]**2 * Ge_ip - r[i-1]**2 * Ge_im) / (2 * dr * r[i]**2)

            Gi_ip = mu_i * E_field[i] * 0.5 * (ni[i] + ni[i+1])
            Gi_im = mu_i * E_field[i] * 0.5 * (ni[i] + ni[i-1])
            div_Gi = (r[i+1]**2 * Gi_ip - r[i-1]**2 * Gi_im) / (2 * dr * r[i]**2)

            dne = -div_Ge + S_ion[i] - S_att[i] - S_rec[i]
            dni = -div_Gi + S_ion[i] - S_rec[i]

            v_e = abs(mu_e[i] * E_field[i]) + De[i] / (dr + 1e-30)
            v_i = abs(mu_i * E_field[i])
            dt_e = 0.10 * dr / (v_e + 1e-12)
            dt_i = 0.10 * dr / (v_i + 1e-12)

            d_ne = np.clip(dne * dt_e, -0.02 * ne[i], 0.02 * ne[i])
            d_ni = np.clip(dni * dt_i, -0.02 * ni[i], 0.02 * ni[i])

            ne_new[i] = np.clip(ne[i] + d_ne, NE_MIN_M3, NE_MAX_M3)
            ni_new[i] = np.clip(ni[i] + d_ni, NI_MIN_M3, NI_MAX_M3)

        # 6) Soft boundary conditions with stronger wall sink
        ne_new[0]  = np.clip(1.05 * ne_new[1] * (1 + SEC_ELEC_COEF), NE_MIN_M3, NE_MAX_M3)
        ni_new[0]  = np.clip(1.03 * ni_new[1], NI_MIN_M3, NI_MAX_M3)
        ne_new[-1] = np.clip(0.70 * ne_new[-2], NE_MIN_M3, NE_MAX_M3)
        ni_new[-1] = np.clip(0.75 * ni_new[-2], NI_MIN_M3, NI_MAX_M3)

        # 7) Smooth and re-couple ions toward quasi-neutral baseline
        ne_new = smooth_1d(ne_new, n_pass=1)
        ni_new = smooth_1d(ni_new, n_pass=1)
        ni_new = 0.94 * ni_new + 0.06 * ne_new

        ne = np.clip(ne_new, NE_MIN_M3, NE_MAX_M3)
        ni = np.clip(ni_new, NI_MIN_M3, NI_MAX_M3)

        # 8) Gentle current normalization
        if outer % 30 == 0 and outer > 0:
            sl = slice(Nr // 4, 3 * Nr // 4)
            Ji_prof = e_charge * mu_i      * np.abs(E_field[sl]) * ni[sl]
            Je_prof = e_charge * mu_e[sl]  * np.abs(E_field[sl]) * ne[sl]
            I_prof  = 4 * np.pi * r[sl]**2 * (Ji_prof + Je_prof)
            I_calc  = np.median(I_prof)
            if I_calc > 1e-30:
                scale = np.clip((abs(I_A) / I_calc)**0.06, 0.97, 1.03)
                ne = np.clip(ne * scale, NE_MIN_M3, NE_MAX_M3)
                ni = np.clip(ni * scale, NI_MIN_M3, NI_MAX_M3)

        if verbose and outer % 100 == 0:
            print(f"  iter {outer:4d}  |  Te_peak = {Te.max():.2f} eV  |  ne_peak = {ne.max():.2e} m^-3")

    # Post-process
    E_field = np.zeros(Nr)
    E_field[1:-1] = -(phi[2:] - phi[:-2]) / (2 * dr)
    E_field[0]    = E_field[1]
    E_field[-1]   = E_field[-2]

    mu_e   = electron_mobility(E_field, p_torr)
    Je     = e_charge * mu_e * np.abs(E_field) * ne
    Ji_arr = e_charge * mu_i * np.abs(E_field) * ni
    source = ionization_rate(Te) * N_gas * ne

    return dict(r=r, phi=phi, ne=ne, ni=ni, Te=Te, E=E_field,
                Je=Je, Ji=Ji_arr, source=source)


with open("../spec/experiment_summary_norm.csv", "r") as f:
    df = pd.read_csv(f)


training_data = {}
number = 0

for index, row in df.iterrows():
    filename = row['File']
    print("Reading:", filename)
    
    current = row['Current_A']
    voltage = row['Voltage_kV']
    pressure = row['Pressure_mTorr']
    ratios = [row[f'rat{i}'] for i in range(1, 7)]

    results = solve_plasma(float(current)*1e-3, float(voltage), float(pressure)*1e-3, verbose=False)

    training_data[number] = {'Current': float(current), 'Voltage': float(voltage), 'Pressure': float(pressure), 'Peak_Ratios': ratios, 'Electron_Temperatures': list(results['Te']), 'Electron_Densities': list(results['ne']), 'Ion_Densities': list(results['ni'])}
    number += 1


with open("training_data.json", "w") as f:
    json.dump(training_data, f, indent=4)
