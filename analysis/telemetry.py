"""Functions for reading and processing telemetry data."""

from pathlib import Path

import pandas as pd
from scipy import integrate

hebi_names = ["steer", "bogie"]


def read_telem(path: Path):
    df = pd.read_csv(path)
    for i in range(4):
        df[f"wheel{i+1}_pos"] -= df[f"wheel{i+1}_pos"].iloc[0]
    return df


def calculate_energy_consumption(df: pd.DataFrame) -> float:
    """given a dataframe representing a transition, calculates the total energy consumption over the interval
    the dataframe covers"""

    # Calculate total energy across all 4 wheels
    total_wheel_energy_J = 0
    for i in range(1, 5):
        wheel_power_W = (
            (df[f"wheel{i}_cur"] / 1000)
            .multiply(df[f"wheel{i}_vel"])
            .apply(lambda x: x * 1.45)
            .abs()
        )
        wheel_energy_J = integrate.trapz(wheel_power_W, df["time"])
        total_wheel_energy_J += wheel_energy_J

    # Calculate total energy across both hebis
    total_hebi_energy_J = 0
    for name in hebi_names:
        hebi_power = df[f"hebi_{name}_eff"].multiply(df[f"hebi_{name}_vel"]).abs()
        hebi_energy_J = integrate.trapz(hebi_power, df["time"])
        total_hebi_energy_J += hebi_energy_J

    return total_wheel_energy_J + total_hebi_energy_J


def calculate_joint_work(df: pd.DataFrame) -> float:
    """given a dataframe representing a transition, calculates the articulation joint work"""

    joint_work = 0

    for name in hebi_names:
        # multiply joint work by angular displacement
        angular_displacement = df[f"hebi_{name}_pos"].diff()
        joint_work += df[f"hebi_{name}_eff"].multiply(angular_displacement).sum()

    return joint_work
