#!/usr/bin/env python3
import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==============================================================================
# Configuration & Physical Constants
# ==============================================================================
DT = 0.005              # Loop period: 5 ms (200 Hz)
WHEEL_RADIUS_M = 0.02   # Wheel radius: 20 mm
V_LIMIT_V = 5.0         # Actuator saturation limit
RUNAWAY_RATE = 40.0     # Runaway velocity threshold (rad/s)
FALL_RAD = 0.50         # Fall limit: 0.50 rad (~28.6 deg)
FALL_DEG = np.degrees(FALL_RAD)

DEFAULT_PATH = "data/03_first_modular/LOG.CSV"

def load_and_preprocess(filepath: str) -> pd.DataFrame:
    if not os.path.exists(filepath):
        print(f"Error: File not found: {filepath}")
        sys.exit(1)
        
    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]

    # Handle slight naming variations between versions
    wheel_col = 'wheel_crad' if 'wheel_crad' in df.columns else 'wheel_cr'
    wheel_rate_col = 'wheel_rate_crad_s' if 'wheel_rate_crad_s' in df.columns else 'wheel_rate_cr'

    # Time vector
    df['time_s'] = df['row'] * DT

    # Scaling raw integer records to SI and engineering units
    df['pitch_deg'] = df['pitch_cdeg'] / 100.0
    df['pitch_rad'] = np.radians(df['pitch_deg'])

    df['rate_dps'] = df['rate_ddps'] / 10.0
    df['rate_rad_s'] = np.radians(df['rate_dps'])

    # Scaled using detected column names
    df['wheel_rad'] = df[wheel_col] / 100.0
    df['wheel_pos_m'] = df['wheel_rad'] * WHEEL_RADIUS_M

    df['wheel_rate_rad_s'] = df[wheel_rate_col] / 100.0
    df['wheel_vel_m_s'] = df['wheel_rate_rad_s'] * WHEEL_RADIUS_M

    # v_mv: millivolts -> volts
    df['v_cmd'] = df['v_mv'] / 1000.0

    return df

def print_performance_metrics(df: pd.DataFrame):
    duration = df['time_s'].iloc[-1] - df['time_s'].iloc[0]
    total_samples = len(df)
    
    # Statistical analysis of pitch angle error
    pitch_rms = np.sqrt(np.mean(df['pitch_deg']**2))
    pitch_max = df['pitch_deg'].max()
    pitch_min = df['pitch_deg'].min()
    
    # Actuator saturation statistics
    saturated_samples = np.sum(np.abs(df['v_cmd']) >= (V_LIMIT_V - 0.05))
    saturation_ratio = (saturated_samples / total_samples) * 100.0
    
    # Wheel motion metrics
    max_wheel_speed = np.max(np.abs(df['wheel_rate_rad_s']))
    total_travel_m = df['wheel_pos_m'].iloc[-1] - df['wheel_pos_m'].iloc[0]

    print("=" * 65)
    print("           SELF-BALANCING ROBOT - RUN PERFORMANCE METRICS        ")
    print("=" * 65)
    print(f"Total Duration        : {duration:.3f} s ({total_samples} samples)")
    print(f"Sampling Frequency    : {1.0 / DT:.1f} Hz (Period = {DT*1000:.1f} ms)")
    print("-" * 65)
    print(f"Pitch Angle RMS       : {pitch_rms:.2f} deg")
    print(f"Pitch Angle Range     : [{pitch_min:.2f}, {pitch_max:.2f}] deg")
    print(f"Peak Angular Rate     : {np.max(np.abs(df['rate_dps'])):.2f} deg/s")
    print("-" * 65)
    print(f"Peak Wheel Speed      : {max_wheel_speed:.2f} rad/s ({max_wheel_speed * WHEEL_RADIUS_M:.2f} m/s)")
    print(f"Net Wheel Travel      : {total_travel_m:.3f} m")
    print("-" * 65)
    print(f"Voltage Saturation    : {saturation_ratio:.1f}% of total run time (|V| >= {V_LIMIT_V:.1f}V)")
    print(f"Mean Control Effort   : {np.mean(df['v_cmd']**2):.3f} V^2")
    print("=" * 65)

def plot_dynamics(df: pd.DataFrame, output_image: str = "analysis_plot.png"):
    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=False)
    t = df['time_s']

    # 1. Pitch Angle History
    axs[0, 0].plot(t, df['pitch_deg'], label=r"$\theta$ (Pitch)", color="tab:blue", lw=1.5)
    axs[0, 0].axhline(0, color="gray", linestyle="--", alpha=0.7)
    axs[0, 0].axhline(FALL_DEG, color="tab:red", linestyle=":", label="Fall Limit")
    axs[0, 0].axhline(-FALL_DEG, color="tab:red", linestyle=":")
    axs[0, 0].set_ylabel("Pitch Angle [deg]")
    axs[0, 0].set_title("Pitch Angle Tracking")
    axs[0, 0].grid(True, alpha=0.3)
    axs[0, 0].legend(loc="upper right")

    # 2. Pitch Angular Velocity
    axs[0, 1].plot(t, df['rate_dps'], label=r"$\dot{\theta}$ (Rate)", color="tab:orange", lw=1.2)
    axs[0, 1].axhline(0, color="gray", linestyle="--", alpha=0.7)
    axs[0, 1].set_ylabel("Pitch Rate [deg/s]")
    axs[0, 1].set_title("Pitch Angular Velocity")
    axs[0, 1].grid(True, alpha=0.3)
    axs[0, 1].legend(loc="upper right")

    # 3. Wheel Position
    axs[1, 0].plot(t, df['wheel_rad'], label=r"$\phi_w$ (Angle)", color="tab:green", lw=1.5)
    axs[1, 0].set_ylabel("Wheel Angle [rad]")
    axs[1, 0].set_title("Wheel Displacement")
    axs[1, 0].grid(True, alpha=0.3)
    axs[1, 0].legend(loc="upper right")

    # 4. Wheel Velocity
    axs[1, 1].plot(t, df['wheel_rate_rad_s'], label=r"$\dot{\phi}_w$", color="tab:purple", lw=1.2)
    axs[1, 1].axhline(RUNAWAY_RATE, color="tab:red", linestyle=":", label="Runaway Limit")
    axs[1, 1].axhline(-RUNAWAY_RATE, color="tab:red", linestyle=":")
    axs[1, 1].set_ylabel("Wheel Speed [rad/s]")
    axs[1, 1].set_title("Wheel Speed")
    axs[1, 1].grid(True, alpha=0.3)
    axs[1, 1].legend(loc="upper right")

    # 5. Control Voltage Command
    axs[2, 0].plot(t, df['v_cmd'], label=r"$V_{cmd}$", color="tab:red", lw=1.2)
    axs[2, 0].axhline(V_LIMIT_V, color="black", linestyle="--", label="Limits")
    axs[2, 0].axhline(-V_LIMIT_V, color="black", linestyle="--")
    axs[2, 0].axhspan(-0.4, 0.4, color="yellow", alpha=0.2, label="Deadband (+/-0.4V)")
    axs[2, 0].set_xlabel("Time [s]")
    axs[2, 0].set_ylabel("Motor Command [V]")
    axs[2, 0].set_title("Actuator Control Voltage")
    axs[2, 0].grid(True, alpha=0.3)
    axs[2, 0].legend(loc="upper right")

    # 6. Phase Portrait: Pitch Rate vs Pitch Angle
    axs[2, 1].plot(df['pitch_deg'], df['rate_dps'], color="tab:brown", lw=0.8, alpha=0.8)
    axs[2, 1].scatter(df['pitch_deg'].iloc[0], df['rate_dps'].iloc[0], color="green", s=40, label="Start", zorder=5)
    axs[2, 1].scatter(df['pitch_deg'].iloc[-1], df['rate_dps'].iloc[-1], color="red", s=40, label="End", zorder=5)
    axs[2, 1].axvline(0, color="gray", linestyle="--", alpha=0.5)
    axs[2, 1].axhline(0, color="gray", linestyle="--", alpha=0.5)
    axs[2, 1].set_xlabel("Pitch Angle [deg]")
    axs[2, 1].set_ylabel("Pitch Rate [deg/s]")
    axs[2, 1].set_title(r"Phase Plane ($\theta$ vs $\dot{\theta}$)")
    axs[2, 1].grid(True, alpha=0.3)
    axs[2, 1].legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(output_image, dpi=200)
    print(f"Plot saved successfully to: {output_image}")
    plt.show()

def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    df = load_and_preprocess(filepath)
    print_performance_metrics(df)
    plot_dynamics(df)

if __name__ == "__main__":
    main()
