#!/usr/bin/env python3
"""
Rich sysid analysis — STM32F103 firmware output.
CSV: step, t_us, cmd_mv, ccr, adc, ang, st, mode

Phases encoded in step // 100:
    0 = REST (brake→coast baseline)
    1 = Fine FWD staircase (14 levels)
    2 = Fine REV staircase (14 levels)
    3 = Dynamic step (0.5V / 0.8V / -0.6V with brakes)
    4 = Log-stair + coast (2 passes × 10 levels)
    5 = Chirp 0.1 → 15 Hz
    6 = Doublet (5 reps)
    7 = Coast-down friction
    8 = Final brake

Usage:
    python analyze_rich.py sysid_rich.csv

Outputs:
    sysid_summary.txt
    sysid_analysis.png
    sysid_phase{0..7}.csv
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal as sig
from scipy.optimize import curve_fit
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

# ============================================================ constants
COUNTS_PER_REV = 4096
VDDA_MV        = 3300
DIV_R1_K       = 20
DIV_R2_K       = 10
TWO_PI         = 2 * np.pi
I2C_ERR_ST     = 0xFF
I2C_ERR_ANG    = 0xFFFF
MOTION_THRESH  = 0.30    # rad/s — below this we call it "not moving"
STEADY_FRAC    = 0.50    # last fraction of each step = steady state

# Motor modes as encoded in firmware
MODE_BRAKE     = 0
MODE_COAST     = 1
MODE_DRIVE_FWD = 2
MODE_DRIVE_REV = 3


# ============================================================ helpers
def adc_to_mv(adc):
    return adc * VDDA_MV * (DIV_R1_K + DIV_R2_K) / (DIV_R2_K * 4095)


def unwrap_angle(ang_raw, period=COUNTS_PER_REV):
    a = np.asarray(ang_raw, dtype=float).copy()
    a[a >= I2C_ERR_ANG - 1] = np.nan
    out = np.full_like(a, np.nan)
    prev = None
    for i, x in enumerate(a):
        if np.isnan(x):
            continue
        if prev is None:
            out[i] = x
            prev = x
            continue
        d = (x - (prev % period) + period / 2) % period - period / 2
        prev += d
        out[i] = prev
    return out


def velocity(ang_u, t_us):
    """Central difference; NaN-tolerant; rad/s."""
    a = np.asarray(ang_u, dtype=float)
    t = np.asarray(t_us, dtype=float) * 1e-6
    n = len(a)
    v = np.full(n, np.nan)
    if n < 2:
        return v
    for i in range(1, n - 1):
        if np.isnan(a[i - 1]) or np.isnan(a[i + 1]):
            continue
        dt = t[i + 1] - t[i - 1]
        if dt > 0:
            v[i] = (a[i + 1] - a[i - 1]) / dt * (TWO_PI / COUNTS_PER_REV)
    if not np.isnan(a[0]) and not np.isnan(a[1]) and t[1] > t[0]:
        v[0] = (a[1] - a[0]) / (t[1] - t[0]) * (TWO_PI / COUNTS_PER_REV)
    if not np.isnan(a[-1]) and not np.isnan(a[-2]) and t[-1] > t[-2]:
        v[-1] = (a[-1] - a[-2]) / (t[-1] - t[-2]) * (TWO_PI / COUNTS_PER_REV)
    return v


def fit_line(x, y):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    mask = np.isfinite(x) & np.isfinite(y)
    x, y = x[mask], y[mask]
    if len(x) < 2:
        return np.nan, np.nan, np.nan
    A = np.vstack([x, np.ones_like(x)]).T
    (s, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ np.array([s, b])
    ss_res = np.sum((y - yhat) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else np.nan
    return s, b, r2


def steady_omega(g, frac=STEADY_FRAC, skip_first=0.10):
    """Mean omega over the last `frac` of the group, skipping transients."""
    n = len(g)
    if n < 4:
        return np.nan
    k_start = int(n * skip_first)
    k0 = int(n * (1 - frac))
    k0 = max(k0, k_start)
    w = g["omega"].iloc[k0:].values
    w = w[np.isfinite(w)]
    return float(np.mean(w)) if len(w) else np.nan


def step_response(g, thresh_lo=0.10, thresh_hi=0.90):
    """
    Rise time 10–90 % and final value for a single driven step.
    Group `g` must contain columns omega, t_us, cmd_mv.
    """
    w = g["omega"].values
    t = g["t_us"].values * 1e-6
    mask = np.isfinite(w)
    if mask.sum() < 5:
        return None
    w = w[mask]
    t = t[mask]
    # Steady value from last 30 %
    k0 = int(len(w) * 0.70)
    w_final = np.mean(w[k0:])
    if abs(w_final) < MOTION_THRESH:
        return None
    sgn = np.sign(w_final)
    wn = w * sgn
    if wn[0] < 0:
        # started backwards; not a clean step
        return None
    wf = abs(w_final)
    t_lo = t_hi = None
    for i in range(len(wn)):
        if t_lo is None and wn[i] >= thresh_lo * wf:
            t_lo = t[i]
        if t_hi is None and wn[i] >= thresh_hi * wf:
            t_hi = t[i]
            break
    tr = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else np.nan
    return dict(w_final=w_final, t_rise=tr)


# ============================================================ analysis phases
def analyze_phase0(df0, out):
    """REST baseline noise."""
    out.append("\n=== Phase 0: REST baseline ===")
    brake = df0[df0["mode"] == MODE_BRAKE]
    coast = df0[df0["mode"] == MODE_COAST]
    for tag, g in [("brake", brake), ("coast", coast)]:
        if len(g) < 3:
            continue
        w = g["omega"].dropna().values
        if len(w) < 2:
            continue
        out.append(f"  {tag}: n={len(g):4d} ω_mean={np.mean(w):+.3f} "
                   f"ω_std={np.std(w):.3f} rad/s (noise floor)")


def analyze_phase12(df, out):
    """Fine staircase: deadzone, Kv, symmetry."""
    out.append("\n=== Phases 1+2: Fine staircase (deadzone + Kv) ===")
    rows = []
    for ph in (1, 2):
        g = df[df["phase"] == ph]
        for sub, gg in g.groupby("sub"):
            w_ss = steady_omega(gg)
            vb = gg["vbat_v"].mean()
            cmd = gg["cmd_mv"].iloc[0]
            rows.append(dict(phase=ph, level=int(sub), cmd_mv=int(cmd),
                             omega_ss=w_ss, vbat_v=float(vb), n=len(gg)))
    tbl = pd.DataFrame(rows)
    if tbl.empty:
        out.append("  (no data)")
        return tbl, None, None

    out.append(f"  {'ph':>3} {'lvl':>3} {'cmd_mV':>7} {'ω_ss':>9} {'Vbat':>6}")
    # ↓↓↓  itertuples() به‌جای iterrows() — dtype per-column حفظ می‌شود
    for r in tbl.itertuples(index=False):
        out.append(f"  {r.phase:>3d} {r.level:>3d} {r.cmd_mv:>7d} "
                   f"{r.omega_ss:>+9.3f} {r.vbat_v:>6.2f}")

    moving = tbl[tbl["omega_ss"].abs() > MOTION_THRESH]
    slope = intercept = r2 = deadzone = None
    if len(moving) >= 3:
        x = moving["cmd_mv"].abs().values / 1000.0
        y = moving["omega_ss"].abs().values
        slope, intercept, r2 = fit_line(x, y)
        if slope:
            deadzone = -intercept / slope
        out.append(f"\n  |ω| = {slope:+.3f}·|V| {intercept:+.3f}  R²={r2:.4f}")
        out.append(f"  Kv       ≈ {slope:+.2f} rad/s/V  "
                   f"(~{slope*60/TWO_PI:.1f} RPM/V)")
        out.append(f"  deadzone ≈ {deadzone*1000:.0f} mV")

    for tag, ph in [("+V", 1), ("−V", 2)]:
        m = moving[moving["phase"] == ph].sort_values("cmd_mv", key=abs)
        if len(m):
            r = m.iloc[0]
            out.append(f"  first motion {tag}: level={int(r['level'])} "
                       f"cmd={int(r['cmd_mv'])} mV → ω={r['omega_ss']:+.2f} rad/s")

    pos = tbl[tbl["phase"] == 1].set_index("level")["omega_ss"]
    neg = tbl[tbl["phase"] == 2].set_index("level")["omega_ss"]
    common = pos.index.intersection(neg.index)
    if len(common) >= 3:
        asym = (pos[common].abs() - neg[common].abs())
        out.append(f"  asymmetry (FWD − |REV|) across {len(common)} levels: "
                   f"mean={asym.mean():+.3f} rad/s, max={asym.abs().max():.3f}")
    return tbl, slope, deadzone


def analyze_phase6(df, out):
    """Doublet — measure initial dω/dt after drive onset → J estimation."""
    out.append("\n=== Phase 6: Doublet — initial acceleration ===")
    g6 = df[df["phase"] == 6].copy()
    if g6.empty:
        out.append("  (no data)")
        return
    g6["rep"] = g6["sub"] // 2
    g6["parity"] = g6["sub"] % 2

    accels = []
    for (rep, par), g in g6[g6["mode"].isin([MODE_DRIVE_FWD])].groupby(
            ["rep", "parity"]):
        w = g["omega"].values
        t = (g["t_us"].values - g["t_us"].values[0]) * 1e-6
        mask = np.isfinite(w)
        if mask.sum() < 3:
            continue
        k = mask.sum()
        n_fit = max(3, min(k, 5))
        p = np.polyfit(t[mask][:n_fit], w[mask][:n_fit], 1)
        dwdt = p[0]
        accels.append(dwdt)
        cmd = int(g["cmd_mv"].iloc[0])           # ← cast به int
        out.append(f"  rep={int(rep)} pulse={int(par)}: cmd={cmd:+5d} mV, "
                   f"dω/dt={dwdt:+.1f} rad/s²")
    if accels:
        out.append(f"  median dω/dt = {np.median(accels):+.1f} rad/s²")
        out.append("  (if Kt known: J ≈ Kt·I / dω/dt, with I = cmd/R_motor)")

def analyze_phase3(df, out):
    """Dynamic steps → rise time, tau estimate."""
    out.append("\n=== Phase 3: Dynamic step response ===")
    g3 = df[df["phase"] == 3]
    if g3.empty:
        out.append("  (no data)")
        return
    # sub 0=0.5V, 1=brake, 2=0.8V, 3=brake, 4=-0.6V, 5=brake
    labels = {0: "+0.5V", 1: "brake", 2: "+0.8V", 3: "brake",
              4: "−0.6V", 5: "brake"}
    for sub, g in g3.groupby("sub"):
        lab = labels.get(int(sub), f"sub{int(sub)}")
        if lab == "brake":
            continue
        m = step_response(g)
        if m is None:
            out.append(f"  {lab}: no motion")
            continue
        wf = m["w_final"]
        tr = m["t_rise"]
        if np.isfinite(tr):
            tau_est = tr / np.log(9.0)  # 10→90 % of first-order
            out.append(f"  {lab:6s}: ω_ss={wf:+7.3f} rad/s, "
                       f"t_rise(10-90%)={tr*1000:5.0f} ms, "
                       f"τ≈{tau_est*1000:.0f} ms")
        else:
            out.append(f"  {lab:6s}: ω_ss={wf:+7.3f} rad/s, "
                       f"t_rise=— (incomplete)")


def _fit_coast(omega_seg, t_seg):
    """
    Fit ω(t) = (ω0 + τf/b)·exp(−b·t/J) − τf/b   (viscous + Coulomb)
    Practical parameterisation:
        ω(t) = ω0·exp(−t/τb) − v_drift·t     (small Coulomb)
    or   ω(t) = ω0·exp(−t/τb) − c0        (constant)
    Try both; return best by R².
    """
    w = np.asarray(omega_seg, float)
    t = np.asarray(t_seg, float)
    mask = np.isfinite(w)
    t, w = t[mask], w[mask]
    if len(t) < 6 or abs(w[0]) < MOTION_THRESH:
        return None
    t = t - t[0]
    w0 = w[0]
    sgn = np.sign(w0)
    wn = w * sgn  # positive

    # Model A: exponential + linear drift
    def mA(t, a, tau, drift):
        return a * np.exp(-t / tau) - drift * t

    # Model B: exponential + constant (Coulomb plateau)
    def mB(t, a, tau, c):
        return a * np.exp(-t / tau) - c

    out = []
    try:
        p0 = (wn[0], 0.15, 0.5)
        poptA, _ = curve_fit(mA, t, wn, p0=p0, maxfev=8000,
                             bounds=([0, 0.005, 0], [10 * wn[0] + 1, 5.0, 20.0]))
        rA = mA(t, *poptA)
        r2A = 1 - np.sum((wn - rA) ** 2) / (np.sum((wn - wn.mean()) ** 2) + 1e-20)
        out.append(("linear-drift", r2A, poptA))
    except Exception:
        pass
    try:
        p0 = (wn[0], 0.15, 0.3)
        poptB, _ = curve_fit(mB, t, wn, p0=p0, maxfev=8000,
                             bounds=([0, 0.005, 0], [10 * wn[0] + 1, 5.0, 20.0]))
        rB = mB(t, *poptB)
        r2B = 1 - np.sum((wn - rB) ** 2) / (np.sum((wn - wn.mean()) ** 2) + 1e-20)
        out.append(("plateau", r2B, poptB))
    except Exception:
        pass

    if not out:
        return None
    out.sort(key=lambda x: -x[1])
    kind, r2, popt = out[0]
    if kind == "linear-drift":
        a, tau, drift = popt
        return dict(tau_b=tau, drift=drift, r2=r2, sign=sgn, kind=kind)
    else:
        a, tau, c = popt
        return dict(tau_b=tau, plateau=c, r2=r2, sign=sgn, kind=kind)


def analyze_phase4(df, out):
    """
    Log staircase with coast.
    Phase 4 sub encoding: pass*20 + level*2 + coast_bit.
        pass=0 ascending (+), pass=1 descending (−)
    Drive segment = first 800 ms of each level
    Coast segment = next 400 ms
    """
    out.append("\n=== Phase 4: Log-stair with coast (friction) ===")
    g4 = df[df["phase"] == 4].copy()
    if g4.empty:
        out.append("  (no data)")
        return None, None
    g4["pass"] = g4["sub"] // 20
    g4["within"] = g4["sub"] % 20
    g4["level"] = g4["within"] // 2
    g4["coast"] = g4["within"] % 2

    # --- steady (end of drive) for Kv refinement ---
    out.append("  ---- steady-state at end of drive ----")
    kv_pts = []
    for (p, lvl), g in g4[g4["coast"] == 0].groupby(["pass", "level"]):
        w_ss = steady_omega(g, frac=0.40, skip_first=0.50)  # last part of drive
        cmd = g["cmd_mv"].iloc[0]
        if np.isfinite(w_ss) and abs(w_ss) > MOTION_THRESH:
            kv_pts.append((abs(cmd) / 1000.0, abs(w_ss)))
        out.append(f"  pass={p} lvl={lvl:2d} cmd={cmd:+6d} mV  ω={w_ss:+7.3f}")
    if len(kv_pts) >= 3:
        x, y = zip(*kv_pts)
        s, b, r2 = fit_line(x, y)
        out.append(f"  log-stair fit: |ω|={s:+.3f}·|V|{b:+.3f} R²={r2:.4f}")

    # --- friction from coast segments ---
    out.append("\n  ---- coast decay fits (viscous + Coulomb) ----")
    coast_fits = []
    for (p, lvl), g in g4[g4["coast"] == 1].groupby(["pass", "level"]):
        w = g["omega"].values
        t = g["t_us"].values * 1e-6
        fit = _fit_coast(w, t)
        if fit is None:
            continue
        coast_fits.append(fit)
        out.append(f"  pass={p} lvl={lvl:2d}: τb={fit['tau_b']*1000:5.0f} ms "
                   f"sign={fit['sign']:+.0f} R²={fit['r2']:.3f}")

    if coast_fits:
        taus = [f["tau_b"] for f in coast_fits]
        out.append(f"  median τb over {len(coast_fits)} coasts: "
                   f"{np.median(taus)*1000:.0f} ms  "
                   f"(range {min(taus)*1000:.0f}–{max(taus)*1000:.0f})")
    return g4, coast_fits


def analyze_phase5(df, out):
    """
    Chirp → empirical transfer function.
    Input: cmd_mv / 1000 (command voltage)
    Output: omega (rad/s)
    """
    out.append("\n=== Phase 5: Chirp — frequency response ===")
    g5 = df[(df["phase"] == 5) & (df["sub"] == 0)].copy()
    if len(g5) < 200:
        out.append("  (not enough chirp samples)")
        return None
    # Resample to uniform dt using t_us (as the sample rate might not be exact)
    t = (g5["t_us"].values - g5["t_us"].values[0]) * 1e-6
    dt_med = np.median(np.diff(t))
    fs = 1.0 / dt_med
    out.append(f"  effective fs = {fs:.1f} Hz, n = {len(g5)}")
    if fs < 40:
        out.append(f"  ⚠ sampling < 40 Hz: chirp up to 15 Hz will alias")

    # Linear interpolation to uniform grid
    t_u = np.arange(0, t[-1], dt_med)
    v_cmd = np.interp(t_u, t, g5["cmd_mv"].values / 1000.0)
    w = np.interp(t_u, t, g5["omega"].values)

    # Detrend (remove DC — motor won't hold any DC in chirp, but there could be bias)
    v_cmd = v_cmd - v_cmd.mean()
    # omega can have offset if the wheel drifted
    w_off = w.mean()
    w = w - w_off

    nper = min(1024, len(v_cmd) // 4)
    if nper < 64:
        out.append("  ⚠ chirp too short for reliable ETF")
        return None

    f, Pvv = sig.welch(v_cmd, fs=fs, nperseg=nper)
    f, Pww = sig.welch(w,     fs=fs, nperseg=nper)
    f, Pvw = sig.csd(v_cmd, w, fs=fs, nperseg=nper)
    H = Pvw / (Pvv + 1e-20)
    coh = np.abs(Pvw) ** 2 / (Pvv * Pww + 1e-20)

    # Only trust where coherence > 0.5 and within chirp band (0.1..15 Hz)
    valid = (coh > 0.5) & (f >= 0.1) & (f <= 15.0) & (f > 0)
    if valid.sum() < 5:
        out.append("  ⚠ coherence < 0.5 over most of the band")
        return f, H, coh

    mag = np.abs(H[valid])
    phase_deg = np.degrees(np.angle(H[valid]))
    fv = f[valid]

    # Estimate DC gain & bandwidth from magnitude
    mag_db = 20 * np.log10(mag + 1e-20)
    kv_from_chirp = np.median(mag[fv < 0.5]) if (fv < 0.5).any() else np.nan
    if np.isfinite(kv_from_chirp):
        out.append(f"  Kv (low-f plateau) ≈ {kv_from_chirp:.2f} rad/s/V")
        # −3 dB point
        db_max = 20 * np.log10(kv_from_chirp + 1e-20)
        threshold = db_max - 3.0
        below = np.where(mag_db < threshold)[0]
        if len(below):
            f_3db = fv[below[0]]
            out.append(f"  f_-3dB ≈ {f_3db:.2f} Hz → τ_eff ≈ "
                       f"{1/(TWO_PI*f_3db)*1000:.0f} ms")

    # Low-freq phase slope → τ
    low_mask = fv < 1.0
    if low_mask.sum() >= 2:
        out.append(f"  phase at 0.5 Hz ≈ "
                   f"{np.interp(0.5, fv, phase_deg):.1f}°")

    out.append(f"  coherence > 0.8 over {np.sum(coh[valid]>0.8)} bins "
               f"of {valid.sum()} valid")

    return f, H, coh



def analyze_phase7(df, out):
    """Independent coast-down check."""
    out.append("\n=== Phase 7: Coast-down friction ===")
    g7 = df[df["phase"] == 7].copy()
    if g7.empty:
        out.append("  (no data)")
        return
    for sub, g in g7[g7["mode"] == MODE_COAST].groupby("sub"):
        w = g["omega"].values
        t = g["t_us"].values * 1e-6
        fit = _fit_coast(w, t)
        if fit is None:
            out.append(f"  sub={int(sub)}: not enough motion")
            continue
        out.append(f"  sub={int(sub)}: τb={fit['tau_b']*1000:.0f} ms, "
                   f"R²={fit['r2']:.3f}, kind={fit['kind']}, "
                   f"sign={fit['sign']:+.0f}")


# ============================================================ plotting
def make_plots(df, tbl12, g4, chirp_data, out_png):
    fig, ax = plt.subplots(4, 2, figsize=(15, 16))
    t_all = df["t_us"].values * 1e-6

    # 1 — Command staircase with phase shading
    for ph, col in zip(range(9), plt.cm.tab10(np.linspace(0, 1, 9))):
        mask = df["phase"] == ph
        if mask.any():
            ax[0, 0].axvspan(t_all[mask][0], t_all[mask][-1],
                             color=col, alpha=0.10)
    ax[0, 0].plot(t_all, df["cmd_mv"] / 1000, ".", ms=1, color="k")
    ax[0, 0].set(xlabel="t [s]", ylabel="cmd [V]", title="Command by phase")
    ax[0, 0].grid(True, alpha=0.3)

    # 2 — Battery
    ax[0, 1].plot(t_all, df["vbat_v"], ".", ms=1)
    ax[0, 1].set(xlabel="t [s]", ylabel="VBAT [V]", title="Battery")
    ax[0, 1].grid(True, alpha=0.3)

    # 3 — Unwrapped angle
    ax[1, 0].plot(t_all, df["ang_u"] * 360 / COUNTS_PER_REV, ".", ms=1)
    ax[1, 0].set(xlabel="t [s]", ylabel="angle [deg]", title="Unwrapped angle")
    ax[1, 0].grid(True, alpha=0.3)

    # 4 — Omega
    ax[1, 1].plot(t_all, df["omega"], ".", ms=1)
    ax[1, 1].axhline(0, color="k", lw=0.4)
    ax[1, 1].set(xlabel="t [s]", ylabel="ω [rad/s]", title="Angular velocity")
    ax[1, 1].grid(True, alpha=0.3)

    # 5 — Fine staircase curve
    if tbl12 is not None and not tbl12.empty:
        for ph, col, tag in [(1, "C0", "+V"), (2, "C1", "−V")]:
            sub = tbl12[tbl12["phase"] == ph]
            ax[2, 0].plot(sub["cmd_mv"] / 1000, sub["omega_ss"],
                          "o-", color=col, label=tag, ms=4)
        ax[2, 0].axhline(0, color="k", lw=0.4)
        ax[2, 0].axvline(0, color="k", lw=0.4)
        ax[2, 0].legend()
    ax[2, 0].set(xlabel="V_cmd [V]", ylabel="ω_ss [rad/s]",
                 title="Fine staircase (deadzone + Kv)")
    ax[2, 0].grid(True, alpha=0.3)

    # 6 — Log stair coast fits (first pass, few levels)
    if g4 is not None and not g4.empty:
        for lvl, col in [(0, "C0"), (3, "C1"), (6, "C2"), (9, "C3")]:
            g = g4[(g4["pass"] == 0) & (g4["level"] == lvl) &
                   (g4["coast"] == 1)]
            if len(g) > 3:
                t = (g["t_us"].values - g["t_us"].values[0]) * 1e-6
                ax[2, 1].plot(t * 1000, g["omega"].values,
                              ".", ms=2, color=col, label=f"L{lvl}")
        ax[2, 1].legend(fontsize=8)
    ax[2, 1].set(xlabel="t since coast [ms]", ylabel="ω [rad/s]",
                 title="Coast decays (Phase 4)")
    ax[2, 1].grid(True, alpha=0.3)

    # 7 — Chirp ETF magnitude
    if chirp_data is not None:
        f, H, coh = chirp_data
        mag_db = 20 * np.log10(np.abs(H) + 1e-20)
        valid = coh > 0.5
        ax[3, 0].semilogx(f[valid], mag_db[valid], "o", ms=2)
        ax[3, 0].set(xlabel="f [Hz]", ylabel="|H| [dB]",
                     title="ETF magnitude (coh>0.5)")
        ax[3, 0].grid(True, which="both", alpha=0.3)
        ax2 = ax[3, 0].twinx()
        ax2.semilogx(f, coh, "-", color="gray", alpha=0.5, lw=0.8)
        ax2.set_ylabel("coherence", color="gray")

        # Phase
        phase_deg = np.degrees(np.angle(H[valid]))
        ax[3, 1].semilogx(f[valid], phase_deg, "o", ms=2, color="C1")
        ax[3, 1].set(xlabel="f [Hz]", ylabel="∠H [deg]",
                     title="ETF phase (coh>0.5)")
        ax[3, 1].grid(True, which="both", alpha=0.3)
        ax[3, 1].axhline(-45, color="k", lw=0.4, ls="--")
    else:
        ax[3, 0].text(0.5, 0.5, "no chirp data", ha="center", va="center",
                      transform=ax[3, 0].transAxes)
        ax[3, 1].axis("off")

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"wrote {out_png}")


# ============================================================ main
def main():
    csv = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sysid_rich.csv")
    if not csv.exists():
        sys.exit(f"not found: {csv}")

    # ---- 1. Load ----
    df = pd.read_csv(csv, comment="#", skipinitialspace=True)
    df.columns = [c.strip() for c in df.columns]
    required = ["step", "t_us", "cmd_mv", "ccr", "adc", "ang", "st"]
    for c in required:
        if c not in df.columns:
            sys.exit(f"missing column: {c}")
    if "mode" not in df.columns:
        df["mode"] = -1

    print(f"Loaded {len(df)} samples, {df['step'].nunique()} distinct steps")

    # ---- 2. Derived ----
    df["phase"] = (df["step"] // 100).astype(int)
    df["sub"] = (df["step"] % 100).astype(int)
    df["ang_u"] = unwrap_angle(df["ang"].values)
    df["omega"] = velocity(df["ang_u"].values, df["t_us"].values)
    df["vbat_v"] = adc_to_mv(df["adc"].values) / 1000

    # ---- 3. Basic diagnostics ----
    out = []
    n_i2c = int((df["st"] == I2C_ERR_ST).sum())
    n_ang = int((df["ang"] >= I2C_ERR_ANG - 1).sum())
    out.append(f"=== Diagnostics ===")
    out.append(f"  rows: {len(df)}   phases present: "
               f"{sorted(df['phase'].unique().tolist())}")
    out.append(f"  I2C errors (st=0xFF): {n_i2c}  ({n_i2c/len(df)*100:.2f} %)")
    out.append(f"  angle errors (0xFFFF): {n_ang}  "
               f"({n_ang/len(df)*100:.2f} %)")

    # Effective sample rate (filter inter-phase gaps)
    dt_all = np.diff(df["t_us"].values) * 1e-6
    dt_sane = dt_all[(dt_all > 1e-3) & (dt_all < 0.5)]
    if len(dt_sane):
        out.append(f"  median dt = {np.median(dt_sane)*1000:.1f} ms "
                   f"(fs ≈ {1/np.median(dt_sane):.1f} Hz)")
        gaps = np.where(dt_all > 0.5)[0]
        out.append(f"  inter-phase gaps: {len(gaps)}")

    # ---- 4. Per-phase analysis ----
    tbl12 = None
    g4 = None
    chirp = None
    for ph in sorted(df["phase"].unique()):
        g = df[df["phase"] == ph]
        # save per-phase CSV
        g.to_csv(f"sysid_phase{ph}.csv", index=False)
        if ph == 0:
            analyze_phase0(g, out)
        elif ph in (1, 2):
            pass  # done together below
        elif ph == 3:
            analyze_phase3(df, out)
        elif ph == 4:
            g4, coast_fits = analyze_phase4(df, out)
        elif ph == 5:
            chirp = analyze_phase5(df, out)
        elif ph == 6:
            analyze_phase6(df, out)
        elif ph == 7:
            analyze_phase7(df, out)
        elif ph == 8:
            out.append("\n=== Phase 8: Final brake (safety) ===")

    # Phase 1+2 done together for asymmetry
    tbl12, kv, dz = analyze_phase12(df, out)

    # ---- 5. Global summary ----
    out.append("\n=== Global summary ===")
    if kv is not None:
        out.append(f"  Kv    ≈ {kv:+.2f} rad/s/V")
    if dz is not None:
        out.append(f"  deadzone ≈ {dz*1000:.0f} mV")
    if len(dt_sane) and n_i2c / len(df) > 0.01:
        out.append(f"  ⚠ I2C error rate {n_i2c/len(df)*100:.1f}% — check wiring")
    if chirp is not None:
        f, H, coh = chirp
        if (coh > 0.5).sum() < 10:
            out.append("  ⚠ chirp coherence low — increase chirp amplitude "
                       "or reduce sample period")
    out.append("\n  → for the model equation ω = Kv·V − b, feed Kv and\n"
               "    deadzone into the controller's feed-forward path.\n"
               "    Feed τ from phase 3/7 into the observer time constant.")

    # ---- 6. Write summary + plots ----
    txt = "\n".join(out)
    Path("sysid_summary.txt").write_text(txt)
    print(txt)
    print("\nwrote sysid_summary.txt")

    make_plots(df, tbl12, g4, chirp, "sysid_analysis.png")

    # Save tabular summaries
    if tbl12 is not None and not tbl12.empty:
        tbl12.to_csv("sysid_fine_stair.csv", index=False)
        print("wrote sysid_fine_stair.csv")


if __name__ == "__main__":
    main()
