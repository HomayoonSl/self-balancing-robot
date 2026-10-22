#!/usr/bin/env python3
"""
تحلیل خروجی staircase identification برای DRV8833 + AS5600.

استفاده:
    python analyze_motor.py motor_data.csv

خروجی‌ها:
    motor_steps_summary.csv   – خلاصه‌ی هر پله
    motor_analysis.png        – نمودارها
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---- پارامترها (هم‌راستا با فریمور) ---------------------------------
COUNTS_PER_REV   = 4096          # AS5600
VDDA_MV          = 3300
DIV_R1_K         = 20
DIV_R2_K         = 10
SAMPLE_PERIOD_US = 5000
STEADY_FRAC      = 0.50          # نیمه‌ی آخر هر پله = حالت پایدار
MOTION_THRESH_RS = 0.30          # آستانه‌ی «حرکت» [rad/s]
I2C_ERR_ST       = 0xFF
I2C_ERR_ANG      = 0xFFFF


def adc_to_mv(adc):
    return adc * VDDA_MV * (DIV_R1_K + DIV_R2_K) / (DIV_R2_K * 4095)


def unwrap_angle(ang_raw, period=COUNTS_PER_REV):
    """باز کردن wrap زاویه‌ی ۱۲بیتی؛ مقادیر خطای I2C → NaN."""
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


def velocity(ang_unwrapped, t_us):
    """مشتق مرکزی زاویه (rad/s)."""
    a = np.asarray(ang_unwrapped, dtype=float)
    t = np.asarray(t_us, dtype=float) * 1e-6
    v = np.full_like(a, np.nan)
    for i in range(1, len(a) - 1):
        if np.isnan(a[i - 1]) or np.isnan(a[i + 1]):
            continue
        dt = t[i + 1] - t[i - 1]
        if dt > 0:
            v[i] = (a[i + 1] - a[i - 1]) / dt * (2 * np.pi / COUNTS_PER_REV)
    return v


def per_step(df):
    rows = []
    for step, g in df.groupby("step", sort=True):
        g = g.reset_index(drop=True)
        omega = velocity(unwrap_angle(g["ang"].values), g["t_us"].values)
        n = len(g)
        k0 = int(n * (1 - STEADY_FRAC))
        omega_ss = float(np.nanmean(omega[k0:])) if k0 < n else np.nan
        vbat_v   = float(adc_to_mv(g["adc"].values).mean() / 1000)
        dt       = np.diff(g["t_us"].values)
        rows.append(dict(
            step=int(step),
            cmd_mv=int(g["cmd_mv"].iloc[0]),
            ccr=int(g["ccr"].iloc[0]),
            vbat_v=vbat_v,
            omega_ss=omega_ss,
            n=n,
            dt_us_med=float(np.median(dt)) if len(dt) else np.nan,
            i2c_bad=int((g["st"].values == I2C_ERR_ST).sum()),
        ))
    return pd.DataFrame(rows)


def fit_line(x, y):
    A = np.vstack([x, np.ones_like(x)]).T
    (s, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ np.array([s, b])
    ss_res = np.sum((y - yhat) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return s, b, r2


def main():
    csv = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("motor_data.csv")
    if not csv.exists():
        sys.exit(f"not found: {csv}")

    df = pd.read_csv(csv, comment="#", skipinitialspace=True)
    print(f"Loaded {len(df)} rows, {df['step'].nunique()} steps from {csv.name}")

    # ---------- عیب‌یابی‌های کلی ----------
    i2c_err = int((df["st"] == I2C_ERR_ST).sum())
    ang_err = int((df["ang"] == I2C_ERR_ANG).sum())
    print(f"I2C/encoder errors: st=0xFF ×{i2c_err},  ang=0xFFFF ×{ang_err}")

    stats = per_step(df)

    # ---- نمونه‌های غیرمنتظره (کمتر از حد مورد انتظار) ----
    exp_n = int(round(stats["dt_us_med"].median() and
                      600_000 / SAMPLE_PERIOD_US))
    bad_n = stats[stats["n"] < exp_n * 0.9]
    if len(bad_n):
        print(f"⚠ {len(bad_n)} step(s) with fewer samples than expected "
              f"({exp_n}):")
        print(bad_n[["step", "n", "i2c_bad"]].to_string(index=False))

    # ---- جدول خلاصه ----
    print("\n=== Per-step summary ===")
    print(stats.to_string(index=False,
                          float_format=lambda v: f"{v:8.3f}"))

    # ---------- برازش منحنی موتور ----------
    ok = stats.dropna(subset=["omega_ss"]).copy()
    moving = ok[ok["omega_ss"].abs() > MOTION_THRESH_RS]

    print("\n=== Motor model ===")
    if len(moving) < 3:
        print("Not enough moving points for a fit.")
    else:
        # آیا علامت ω با علامت فرمان هم‌راستاست؟
        c = float(np.corrcoef(moving["cmd_mv"], moving["omega_ss"])[0, 1])
        print(f"sign-corr(cmd, omega) = {c:+.3f}"
              + ("   (encoder phase inverted vs command)"
                 if c < 0 else ""))

        # برازش متقارن روی |ω| در برابر |V| — بهترین تخمین Kv
        x = moving["cmd_mv"].abs().values / 1000.0     # [V]
        y = moving["omega_ss"].abs().values             # [rad/s]
        slope, intercept, r2 = fit_line(x, y)
        deadzone_v = -intercept / slope if slope else float("nan")
        kv_rpm_v   = slope * 60.0 / (2 * np.pi)

        print(f"|omega| = {slope:+.3f}·|V| {intercept:+.3f}   "
              f"R²={r2:.4f}")
        print(f"  Kv         ≈ {slope:+.2f} rad/s/V  "
              f"(~{kv_rpm_v:.1f} RPM/V)")
        print(f"  Kt (SI)    ≈ {1.0/abs(slope):.5f} N·m/A  (اگر مقاومت "
              f"سیم‌پیچ را بدانی)")
        print(f"  deadzone   ≈ {deadzone_v*1000:.0f} mV "
              f"(≈ {deadzone_v:.3f} V) — اصطکاک ایستا")

        # برازش جدا برای هر جهت
        for name, m in [("+", x > 0),
                        ("-", np.zeros_like(x, dtype=bool))]:
            # m برای "-" یعنی همان |V| ولی می‌خواهیم هر جهت مستقلاً
            pass
        for tag, mask in [("+V", moving["cmd_mv"] > 0),
                          ("-V", moving["cmd_mv"] < 0)]:
            sub = moving[mask]
            if len(sub) >= 2:
                xs = sub["cmd_mv"].abs().values / 1000.0
                ys = sub["omega_ss"].abs().values
                s2, b2, r22 = fit_line(xs, ys)
                print(f"  {tag}: slope={s2:+.3f} rad/s/V, "
                      f"deadzone={-b2/s2*1000:+.0f} mV, R²={r22:.4f}")

    # ---------- تشخیص شروع حرکت (دِدزون تجربی) ----------
    pos = moving[moving["cmd_mv"] > 0].sort_values("cmd_mv")
    neg = moving[moving["cmd_mv"] < 0].sort_values("cmd_mv", ascending=False)
    if len(pos):
        print(f"Smallest +cmd with motion: {pos['cmd_mv'].iloc[0]} mV "
              f"(ω={pos['omega_ss'].iloc[0]:+.2f} rad/s)")
    if len(neg):
        print(f"Smallest -cmd with motion: {neg['cmd_mv'].iloc[0]} mV "
              f"(ω={neg['omega_ss'].iloc[0]:+.2f} rad/s)")

    # ---------- ذخیره ----------
    stats.to_csv("motor_steps_summary.csv", index=False)

    # ---------- نمودارها ----------
    fig, ax = plt.subplots(3, 2, figsize=(14, 10))
    t = df["t_us"].values / 1e6
    ang_u = unwrap_angle(df["ang"].values)
    omega = velocity(ang_u, df["t_us"].values)

    ax[0, 0].plot(t, df["cmd_mv"] / 1000, ".", ms=2)
    ax[0, 0].set(xlabel="t [s]", ylabel="cmd [V]",
                 title="Command staircase"); ax[0, 0].grid(True)

    ax[0, 1].plot(t, adc_to_mv(df["adc"].values) / 1000, ".", ms=2)
    ax[0, 1].set(xlabel="t [s]", ylabel="VBAT [V]",
                 title="Battery voltage"); ax[0, 1].grid(True)

    ax[1, 0].plot(t, ang_u * 360 / COUNTS_PER_REV, ".", ms=1)
    ax[1, 0].set(xlabel="t [s]", ylabel="angle [deg]",
                 title="Unwrapped angle"); ax[1, 0].grid(True)

    ax[1, 1].plot(t, omega, ".", ms=1)
    ax[1, 1].axhline(0, color="k", lw=0.5)
    ax[1, 1].set(xlabel="t [s]", ylabel="ω [rad/s]",
                 title="Angular velocity"); ax[1, 1].grid(True)

    m = stats.dropna(subset=["omega_ss"])
    ax[2, 0].plot(m["cmd_mv"] / 1000, m["omega_ss"], "o")
    ax[2, 0].axhline(0, color="k", lw=0.5)
    ax[2, 0].axvline(0, color="k", lw=0.5)
    if len(moving) >= 3 and slope:
        xs = np.linspace((m["cmd_mv"] / 1000).min(),
                         (m["cmd_mv"] / 1000).max(), 50)
        # fit signed: از همان پارامترهای |ω| ولی با علامت
        ax[2, 0].plot(xs, np.sign(xs) * (slope * np.abs(xs) + intercept),
                      "r-", lw=1,
                      label=f"Kv≈{slope:.2f} rad/s/V")
        ax[2, 0].legend()
    ax[2, 0].set(xlabel="V_cmd [V]", ylabel="ω_ss [rad/s]",
                 title="Steady-state motor curve"); ax[2, 0].grid(True)

    for tag, mask, col in [("+V", m["cmd_mv"] > 0, "C0"),
                           ("−V", m["cmd_mv"] < 0, "C1")]:
        sel = m[mask]
        ax[2, 1].plot(np.abs(sel["cmd_mv"]) / 1000,
                      np.abs(sel["omega_ss"]), "o", color=col, label=tag)
    ax[2, 1].set(xlabel="|V_cmd| [V]", ylabel="|ω_ss| [rad/s]",
                 title="Symmetry check"); ax[2, 1].grid(True)
    ax[2, 1].legend()

    fig.tight_layout()
    fig.savefig("motor_analysis.png", dpi=120)
    print("\nWrote motor_steps_summary.csv and motor_analysis.png")


if __name__ == "__main__":
    main()
