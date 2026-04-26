"""
DC Glow Discharge Radial Plasma Solver
=======================================
Spherical geometry: central cathode (r_c = 23.145 mm) in a grounded
spherical chamber (r_w = 140 mm). Working gas: Air (N2/O2 mixture).

Physics:
  - Poisson's equation for electric potential phi(r)
  - Drift-diffusion continuity for electrons and ions
  - Electron energy equation for Te(r)
  - Ionization/attachment source terms for air chemistry

Usage:
  python dc_glow_plasma_solver.py [--current mA] [--voltage V] [--pressure Torr]
                                   [--export] [--output filename.csv] [--plot]

Examples:
  python dc_glow_plasma_solver.py --current 10 --voltage 500 --pressure 1.0 --plot
  python dc_glow_plasma_solver.py --current 20 --voltage 400 --pressure 2.0 --export
  python dc_glow_plasma_solver.py --current 5  --voltage 600 --pressure 0.5 --export --output my_plasma.csv
"""

import argparse
import sys
import warnings
import numpy as np
import csv
import os

# ── Optional matplotlib ───────────────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("TkAgg")           # change to "Qt5Agg" or "Agg" if needed
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ═══════════════════════════════════════════════════════════════════════════════
# Physical constants
# ═══════════════════════════════════════════════════════════════════════════════
e_charge = 1.602e-19      # C
eps0     = 8.854e-12      # F/m
me       = 9.109e-31      # kg
kb       = 1.381e-23      # J/K
Tgas     = 300.0          # K  (neutral gas temperature)

# ═══════════════════════════════════════════════════════════════════════════════
# Geometry (fixed)
# ═══════════════════════════════════════════════════════════════════════════════
R_CATHODE = 46.29e-3 / 2          # m  (cathode sphere radius)
R_WALL    = 280e-3   / 2          # m  (chamber wall radius)

# ═══════════════════════════════════════════════════════════════════════════════
# Air plasma parameters
# ═══════════════════════════════════════════════════════════════════════════════
# Air is treated as ~80 % N2 / 20 % O2.
# Ionization energies:
#   N2: 15.58 eV,  O2: 12.07 eV  → effective mixture ~ 14.5 eV
# Electron attachment (O2) reduces net ionisation.
# Ion transport: average of N2+ and O2+ mobilities.

E_ION_AIR    = 14.5          # eV  effective ionisation threshold
E_ATT_O2     = 0.0           # eV  O2 attachment threshold (3-body, threshold~0)
MU_I0        = 2.0e-4        # m²/(V·s)  ion mobility at 1 Torr (N-scaled below)
MU_E_REF     = 0.10          # m²/(V·s)  electron mobility at 1 Torr
DE_OVER_MUE  = 1.0           # Te (eV) — Einstein relation: De = mu_e * Te(eV)*e/e = mu_e*Te_eV
SEC_ELEC_COEF = 0.01         # secondary electron emission coefficient γ

# ═══════════════════════════════════════════════════════════════════════════════
# Solver settings
# ═══════════════════════════════════════════════════════════════════════════════
NR          = 400            # number of radial grid points
N_OUTER     = 800            # outer iteration count
N_POISSON   = 10             # Poisson sub-iterations per outer step
SOR_OMEGA   = 1.5            # successive over-relaxation factor
RELAX_N     = 0.03           # density relaxation factor
RELAX_TE    = 0.04           # temperature relaxation factor


# ═══════════════════════════════════════════════════════════════════════════════
# Helper: gas number density at given pressure
# ═══════════════════════════════════════════════════════════════════════════════
def gas_density(p_torr: float) -> float:
    """Return neutral number density [m^-3] from pressure in Torr."""
    return (p_torr * 133.322) / (kb * Tgas)


# ═══════════════════════════════════════════════════════════════════════════════
# Electron transport & rate coefficients (Boltzmann-type fits for air)
# ═══════════════════════════════════════════════════════════════════════════════
def ionization_rate(Te_eV: np.ndarray, p_torr: float) -> np.ndarray:
    """
    Ionization rate coefficient k_iz [m^3/s] for air.
    Uses a two-term Arrhenius fit calibrated to BOLSIG+ data for air.
    """
    Te = np.maximum(Te_eV, 0.3)
    # N2 component (80%)
    k_N2 = 1.8e-14 * np.exp(-15.58 / Te) * (Te / 15.58)**0.5
    # O2 component (20%), lower threshold
    k_O2 = 2.3e-14 * np.exp(-12.07 / Te) * (Te / 12.07)**0.5
    return 0.80 * k_N2 + 0.20 * k_O2


def attachment_rate(Te_eV: np.ndarray, N: float) -> np.ndarray:
    """
    3-body electron attachment rate [m^3/s] for O2 in air.
    k_att ~ 1e-43 * N [s^-1 per electron], simplified fit.
    """
    # Attachment is significant at low Te; decreases above ~2 eV
    Te = np.maximum(Te_eV, 0.1)
    k_att = 2.0e-43 * N * np.exp(-0.5 * Te)   # m^3/s effective
    return k_att


def electron_mobility(E_field: np.ndarray, p_torr: float) -> np.ndarray:
    """
    Electron mobility [m^2/(V·s)], pressure-reduced.
    Simple two-term Phelps-type fit: mu_e = mu_e0 / (1 + |E/N| * scale)
    """
    p_ref = 1.0
    mu0 = MU_E_REF * (p_ref / p_torr)
    # Field-dependent saturation
    norm = np.abs(E_field) / (p_torr * 133.322 / (kb * Tgas))  # E/N in V·m^2
    mu_e = mu0 / (1.0 + norm / 5e-20)
    return mu_e


def ion_mobility(p_torr: float) -> float:
    """Ion mobility [m^2/(V·s)] at pressure p_torr (N-scaled)."""
    return MU_I0 / p_torr


def electron_diffusion(mu_e: np.ndarray, Te_eV: np.ndarray) -> np.ndarray:
    """Electron diffusion coefficient via Einstein relation: De = mu_e * Te[eV]."""
    return mu_e * np.maximum(Te_eV, 0.1)


def electron_temperature_model(
    r: np.ndarray,
    phi: np.ndarray,
    dr: float,
    p_torr: float,
) -> np.ndarray:
    """
    Estimate Tₑ(r) from a simplified electron energy balance.

    In a glow discharge Te is elevated where electrons are accelerated
    by the field (sheath) and falls to 1–3 eV in the bulk.

    Energy input:  Joule heating P = e * |E| * Gamma_e / n_e ~ mu_e * E^2
    Energy loss:   inelastic collisions ~ nu_in * (Te - Tg)
    Steady state:  Te ~ Tg + alpha * (E/N)^2
    """
    Nr = len(r)
    Te = np.full(Nr, 2.0)   # seed

    N = gas_density(p_torr)
    for i in range(1, Nr - 1):
        Ei = abs(phi[i+1] - phi[i-1]) / (2 * dr)   # |E| in V/m
        EN = Ei / N * 1e21                            # E/N in Td (10^-21 V·m²)
        # Simple polynomial fit to BOLSIG+ mean energy vs E/N for air
        # Te_eV ≈ 0.5 + 0.035 * EN^0.65 (valid ~1–300 Td)
        Te[i] = max(0.5, min(15.0, 0.5 + 0.035 * EN**0.65))

    Te[0]    = Te[1]
    Te[Nr-1] = max(0.3, Te[Nr-2] * 0.8)   # cooler near grounded wall
    return Te


# ═══════════════════════════════════════════════════════════════════════════════
# Core solver
# ═══════════════════════════════════════════════════════════════════════════════
def solve_plasma(
    I_A:    float,
    V0:     float,
    p_torr: float,
    verbose: bool = True,
) -> dict:
    """
    Solve the 1D radial (spherical) drift-diffusion plasma model.

    Parameters
    ----------
    I_A    : discharge current [A]
    V0     : cathode voltage [V] (negative)
    p_torr : gas pressure [Torr]
    verbose: print iteration progress

    Returns
    -------
    dict with keys: r, phi, ne, ni, Te, E, Je, Ji, source
    All quantities in SI except r (m), ne/ni (m^-3), Te (eV).
    """
    if V0 > 0:
        V0 = -abs(V0)    # enforce negative cathode

    rc = R_CATHODE
    rw = R_WALL
    Nr = NR
    dr = (rw - rc) / (Nr - 1)

    # Grid
    r = np.linspace(rc, rw, Nr)

    N_gas = gas_density(p_torr)
    mu_i  = ion_mobility(p_torr)

    # ── Initial conditions ────────────────────────────────────────────────────
    s = (r - rc) / (rw - rc)   # normalised position 0→1

    phi = V0 * (1.0 - s)**2
    Te  = 2.0 + 4.0 * np.exp(-((s - 0.15)**2) / 0.03)

    # Estimate initial density from rough current balance
    # Γ_i = n_i * mu_i * |E| at cathode surface; I = 4π r_c^2 * e * Γ_i
    E_est = abs(V0) / (rw - rc)
    n0    = abs(I_A) / (4 * np.pi * rc**2 * e_charge * mu_i * E_est + 1e-30)
    n0    = np.clip(n0, 1e10, 1e20)

    ne = n0 * (0.1 + 0.9 * np.sin(np.pi * s + 0.1 * np.pi))
    ne = np.maximum(ne, 1e8)
    ni = ne * 1.01
    ni = np.maximum(ni, 1e8)

    # ── Iteration ─────────────────────────────────────────────────────────────
    for outer in range(N_OUTER):

        # 1. Update electron temperature from local E/N
        Te_new = electron_temperature_model(r, phi, dr, p_torr)
        Te     = Te + RELAX_TE * (Te_new - Te)
        Te     = np.clip(Te, 0.3, 20.0)

        # 2. Poisson: (1/r²) d/dr(r² dphi/dr) = -e(ni - ne)/eps0
        for _ in range(N_POISSON):
            for i in range(1, Nr - 1):
                rhs   = e_charge * (ni[i] - ne[i]) / eps0
                # Spherical: phi''  + (2/r) phi' = rhs
                # FD:  (phi_{i+1} - 2*phi_i + phi_{i-1})/dr^2
                #       + (2/r_i)*(phi_{i+1}-phi_{i-1})/(2*dr) = rhs
                A = 1.0 / dr**2
                B = 1.0 / (r[i] * dr)
                phi_new = (
                    rhs
                    + (A + B) * phi[i+1]
                    + (A - B) * phi[i-1]
                ) / (2 * A)
                phi[i] = phi[i] + SOR_OMEGA * (phi_new - phi[i])
            phi[0]    = V0
            phi[Nr-1] = 0.0

        # 3. Electric field  (central difference; one-sided at boundaries)
        E_field = np.zeros(Nr)
        E_field[1:-1] = -(phi[2:] - phi[:-2]) / (2 * dr)
        E_field[0]    = -(phi[1]  - phi[0])   / dr
        E_field[-1]   = -(phi[-1] - phi[-2])  / dr

        # 4. Transport coefficients
        mu_e = electron_mobility(E_field, p_torr)
        De   = electron_diffusion(mu_e, Te)

        # 5. Source terms
        k_iz  = ionization_rate(Te, p_torr)
        k_att = attachment_rate(Te, N_gas)
        S_ion = k_iz  * N_gas * ne          # ionisation [m^-3 s^-1]
        S_att = k_att * N_gas * ne          # attachment loss

        # 6. Density update (explicit, relaxed)
        #    Continuity: (1/r²) d/dr(r² Γ) = S
        #    Drift-diffusion fluxes (upwind)
        ne_new = ne.copy()
        ni_new = ni.copy()

        for i in range(1, Nr - 1):
            # Electron flux components
            drift_e = -mu_e[i] * E_field[i] * ne[i]    # drift (electrons move opposite to E)
            diff_e_fwd = -De[i] * (ne[i+1] - ne[i]) / dr
            diff_e_bwd = -De[i] * (ne[i]   - ne[i-1]) / dr

            Ge_i   = drift_e - De[i] * (ne[i+1] - ne[i-1]) / (2 * dr)
            Ge_ip  = -mu_e[i] * E_field[i] * 0.5*(ne[i]+ne[i+1]) - De[i]*(ne[i+1]-ne[i])/dr
            Ge_im  = -mu_e[i] * E_field[i] * 0.5*(ne[i]+ne[i-1]) - De[i]*(ne[i]-ne[i-1])/dr

            div_Ge = (r[i+1]**2 * Ge_ip - r[i-1]**2 * Ge_im) / (2 * dr * r[i]**2)

            # Ion flux (drift only, no diffusion for simplicity)
            Gi_ip = mu_i * E_field[i] * 0.5 * (ni[i] + ni[i+1])   # ions move with E
            Gi_im = mu_i * E_field[i] * 0.5 * (ni[i] + ni[i-1])
            div_Gi = (r[i+1]**2 * Gi_ip - r[i-1]**2 * Gi_im) / (2 * dr * r[i]**2)

            dne = -div_Ge + S_ion[i] - S_att[i]
            dni = -div_Gi + S_ion[i]

            ne_new[i] = max(1e8, ne[i] + RELAX_N * np.sign(dne) * min(abs(dne) * dr / max(abs(ne[i]), 1), ne[i]))
            ni_new[i] = max(1e8, ni[i] + RELAX_N * np.sign(dni) * min(abs(dni) * dr / max(abs(ni[i]), 1), ni[i]))

        # Boundary conditions
        # Cathode (r=rc): ion flux in, electron flux out (secondary emission)
        ne_new[0]    = ne_new[1] * (1 + SEC_ELEC_COEF)
        ni_new[0]    = ni_new[1]
        # Wall (r=rw): grounded, particles absorbed
        ne_new[-1]   = ne_new[-2] * 0.5
        ni_new[-1]   = ni_new[-2] * 0.5

        ne = ne_new
        ni = ni_new

        # 7. Current normalisation: scale density so discharge current matches target
        if outer % 30 == 0 and outer > 0:
            # Compute current density at mid-point
            imid = Nr // 2
            Ji_mid = e_charge * mu_i * abs(E_field[imid]) * ni[imid]
            I_calc = 4 * np.pi * r[imid]**2 * Ji_mid
            if I_calc > 1e-30:
                scale = (abs(I_A) / I_calc) ** 0.3
                scale = np.clip(scale, 0.5, 2.0)
                ne *= scale
                ni *= scale

        if verbose and outer % 100 == 0:
            Je_cat = e_charge * mu_e[0] * abs(E_field[0]) * ne[0]
            I_est  = 4 * np.pi * rc**2 * Je_cat
            print(f"  iter {outer:4d}  |  Te_peak = {Te.max():.2f} eV  "
                  f"|  ne_peak = {ne.max():.2e} m⁻³  "
                  f"|  I_est = {I_est*1e3:.2f} mA")

    # ── Post-process ──────────────────────────────────────────────────────────
    E_field[1:-1] = -(phi[2:] - phi[:-2]) / (2 * dr)
    E_field[0]    = -(phi[1]  - phi[0])   / dr
    E_field[-1]   = -(phi[-1] - phi[-2])  / dr

    mu_e = electron_mobility(E_field, p_torr)
    Je   = e_charge * (mu_e * abs(E_field) * ne)
    Ji   = e_charge * mu_i  * abs(E_field) * ni
    source = ionization_rate(Te, p_torr) * N_gas * ne

    return {
        "r":      r,
        "phi":    phi,
        "ne":     ne,
        "ni":     ni,
        "Te":     Te,
        "E":      E_field,
        "Je":     Je,
        "Ji":     Ji,
        "source": source,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# CSV export
# ═══════════════════════════════════════════════════════════════════════════════
def export_csv(result: dict, filename: str, I_A: float, V0: float, p_torr: float):
    """Write solver results to a CSV file."""
    r   = result["r"]
    ne  = result["ne"]
    ni  = result["ni"]
    Te  = result["Te"]
    phi = result["phi"]
    E   = result["E"]

    header = [
        "# DC Glow Discharge Radial Solver — Air Plasma",
        f"# Cathode radius  : {R_CATHODE*1e3:.3f} mm",
        f"# Chamber radius  : {R_WALL*1e3:.1f} mm",
        f"# Current         : {I_A*1e3:.2f} mA",
        f"# Voltage (cathode): {V0:.1f} V",
        f"# Pressure        : {p_torr:.4f} Torr",
        f"# Gas             : Air (80% N2 / 20% O2)",
        "#",
        "# Columns:",
        "#   r_m        — radial position [m]",
        "#   r_mm       — radial position [mm]",
        "#   ne_m3      — electron density [m^-3]",
        "#   ne_cm3     — electron density [cm^-3]",
        "#   ni_m3      — ion density [m^-3]",
        "#   ni_cm3     — ion density [cm^-3]",
        "#   Te_eV      — electron temperature [eV]",
        "#   phi_V      — electric potential [V]",
        "#   E_Vm       — radial electric field [V/m]",
    ]

    rows = []
    for i in range(len(r)):
        rows.append([
            f"{r[i]:.6e}",
            f"{r[i]*1e3:.4f}",
            f"{ne[i]:.6e}",
            f"{ne[i]/1e6:.6e}",
            f"{ni[i]:.6e}",
            f"{ni[i]/1e6:.6e}",
            f"{Te[i]:.4f}",
            f"{phi[i]:.4f}",
            f"{E[i]:.4f}",
        ])

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        for line in header:
            f.write(line + "\n")
        writer.writerow(["r_m","r_mm","ne_m3","ne_cm3","ni_m3","ni_cm3","Te_eV","phi_V","E_Vm"])
        writer.writerows(rows)

    print(f"\n  CSV saved → {os.path.abspath(filename)}")


# ═══════════════════════════════════════════════════════════════════════════════
# Plotting
# ═══════════════════════════════════════════════════════════════════════════════
def plot_results(result: dict, I_mA: float, V0: float, p_torr: float):
    if not HAVE_MPL:
        print("  matplotlib not installed — skipping plot. (pip install matplotlib)")
        return

    r   = result["r"] * 1e3       # mm
    ne  = result["ne"] / 1e6      # cm^-3
    ni  = result["ni"] / 1e6
    Te  = result["Te"]
    phi = result["phi"]
    E   = np.abs(result["E"])

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle(
        f"Air DC Glow Discharge — I = {I_mA:.1f} mA, "
        f"V$_{{cathode}}$ = {V0:.0f} V, p = {p_torr:.3f} Torr",
        fontsize=13, y=0.98
    )
    gs = gridspec.GridSpec(2, 2, hspace=0.38, wspace=0.35)

    # Panel 1 — Density
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.semilogy(r, ne, color="#185FA5", lw=1.8, label="$n_e$")
    ax1.semilogy(r, ni, color="#993C1D", lw=1.8, ls="--", label="$n_i$")
    ax1.set_xlabel("r (mm)")
    ax1.set_ylabel("Density (cm$^{-3}$)")
    ax1.set_title("Plasma density")
    ax1.legend(fontsize=9)
    ax1.axvline(R_CATHODE*1e3, color="gray", lw=0.8, ls=":")
    ax1.axvline(R_WALL  *1e3, color="gray", lw=0.8, ls=":")
    ax1.grid(True, which="both", alpha=0.25)

    # Panel 2 — Electron temperature
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(r, Te, color="#D4537E", lw=1.8)
    ax2.set_xlabel("r (mm)")
    ax2.set_ylabel("$T_e$ (eV)")
    ax2.set_title("Electron temperature")
    ax2.axvline(R_CATHODE*1e3, color="gray", lw=0.8, ls=":")
    ax2.axvline(R_WALL  *1e3, color="gray", lw=0.8, ls=":")
    ax2.grid(True, alpha=0.25)

    # Panel 3 — Potential
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(r, phi, color="#1D9E75", lw=1.8)
    ax3.axhline(0, color="black", lw=0.5, ls="--")
    ax3.set_xlabel("r (mm)")
    ax3.set_ylabel("$\\phi$ (V)")
    ax3.set_title("Electric potential")
    ax3.axvline(R_CATHODE*1e3, color="gray", lw=0.8, ls=":")
    ax3.axvline(R_WALL  *1e3, color="gray", lw=0.8, ls=":")
    ax3.grid(True, alpha=0.25)

    # Panel 4 — Electric field
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(r, E, color="#BA7517", lw=1.8)
    ax4.set_xlabel("r (mm)")
    ax4.set_ylabel("|E| (V/m)")
    ax4.set_title("Electric field magnitude")
    ax4.axvline(R_CATHODE*1e3, color="gray", lw=0.8, ls=":")
    ax4.axvline(R_WALL  *1e3, color="gray", lw=0.8, ls=":")
    ax4.grid(True, alpha=0.25)

    # Shared annotation
    for ax in [ax1, ax2, ax3, ax4]:
        ax.set_xlim(r[0], r[-1])

    plt.figtext(0.5, 0.01,
                f"Cathode @ r = {R_CATHODE*1e3:.2f} mm  |  "
                f"Wall @ r = {R_WALL*1e3:.0f} mm  |  "
                "dotted lines = electrode boundaries",
                ha="center", fontsize=8, color="gray")
    plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
# Summary printer
# ═══════════════════════════════════════════════════════════════════════════════
def print_summary(result: dict, I_mA: float, V0: float, p_torr: float):
    ne   = result["ne"]
    Te   = result["Te"]
    E    = result["E"]
    phi  = result["phi"]
    r    = result["r"]

    ne_peak_idx = np.argmax(ne)
    Te_peak_idx = np.argmax(Te)

    print("\n" + "="*58)
    print("  SIMULATION SUMMARY")
    print("="*58)
    print(f"  Gas              : Air (80% N₂ / 20% O₂)")
    print(f"  Cathode radius   : {R_CATHODE*1e3:.3f} mm")
    print(f"  Chamber radius   : {R_WALL*1e3:.1f} mm")
    print(f"  Current          : {I_mA:.2f} mA")
    print(f"  Cathode voltage  : {V0:.1f} V")
    print(f"  Pressure         : {p_torr:.4f} Torr")
    print("-"*58)
    print(f"  Peak nₑ          : {ne.max()/1e6:.3e} cm⁻³"
          f"  @ r = {r[ne_peak_idx]*1e3:.1f} mm")
    print(f"  Peak Tₑ          : {Te.max():.2f} eV"
          f"  @ r = {r[Te_peak_idx]*1e3:.1f} mm")
    print(f"  Peak |E|         : {np.abs(E).max():.1f} V/m"
          f"  @ r = {r[np.argmax(np.abs(E))]*1e3:.1f} mm")
    print(f"  Plasma potential : {phi.max():.1f} V")
    print(f"  Bulk Tₑ (mid)    : {Te[NR//2]:.2f} eV")
    print(f"  Bulk nₑ (mid)    : {ne[NR//2]/1e6:.3e} cm⁻³")
    print("="*58)


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════
def parse_args():
    p = argparse.ArgumentParser(
        description="DC glow discharge radial solver — spherical air plasma",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--current",  type=float, default=10.0,
                   help="Discharge current in mA")
    p.add_argument("--voltage",  type=float, default=500.0,
                   help="Cathode voltage magnitude in V (sign applied automatically)")
    p.add_argument("--pressure", type=float, default=1.0,
                   help="Gas pressure in Torr")
    p.add_argument("--export",   action="store_true",
                   help="Export results to CSV")
    p.add_argument("--output",   type=str,   default="",
                   help="CSV output filename (default: auto-generated)")
    p.add_argument("--plot",     action="store_true",
                   help="Show matplotlib plots (requires matplotlib)")
    p.add_argument("--quiet",    action="store_true",
                   help="Suppress iteration output")
    return p.parse_args()


def auto_filename(I_mA, V0, p_torr):
    return f"plasma_I{I_mA:.0f}mA_V{abs(V0):.0f}V_p{p_torr:.3f}Torr.csv"


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    args = parse_args()

    I_mA   = args.current
    I_A    = I_mA * 1e-3
    V0     = -abs(args.voltage)     # cathode is always negative
    p_torr = args.pressure
    verbose = not args.quiet

    print("\n" + "="*58)
    print("  DC Glow Discharge Radial Solver — Air Plasma")
    print("="*58)
    print(f"  I = {I_mA:.2f} mA  |  V = {V0:.1f} V  |  p = {p_torr:.4f} Torr")
    print(f"  Grid: {NR} points from {R_CATHODE*1e3:.2f} mm to {R_WALL*1e3:.0f} mm")
    print(f"  Iterations: {N_OUTER}  (SOR ω = {SOR_OMEGA})")
    print("="*58)
    print("  Solving...\n")

    result = solve_plasma(I_A, V0, p_torr, verbose=verbose)

    print_summary(result, I_mA, V0, p_torr)

    if args.export:
        fname = args.output if args.output else auto_filename(I_mA, V0, p_torr)
        export_csv(result, fname, I_A, V0, p_torr)

    if args.plot:
        plot_results(result, I_mA, V0, p_torr)

    return result


if __name__ == "__main__":
    main()
