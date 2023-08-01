import math
from pathlib import Path
from typing import List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from transforms3d.euler import mat2euler
from transforms3d.quaternions import quat2mat

import vicon

hebi_names = ["steer", "bogie"]
vicon_body_name = "asterix_body"


def read_telem(path: Path):
    df = pd.read_csv(path)
    for i in range(4):
        df[f"wheel{i+1}_pos"] -= df[f"wheel{i+1}_pos"].iloc[0]
    return df


def get_transitions(df: pd.DataFrame) -> List[pd.DataFrame]:
    """Returns a list of the dataframes of transitions"""

    # Identify the start and end indices of consecutive runs
    starts = df.index[(df["in_transition"] == 1) & (df["in_transition"].shift(1) == 0)]
    ends = df.index[(df["in_transition"] == 0) & (df["in_transition"].shift(1) == 1)]

    # Handle the case when the last run is ongoing
    if len(ends) < len(starts):
        ends = ends.append(pd.Index([df.index[-1]]))

    runs = [df.iloc[start:end] for start, end in zip(starts, ends)]
    return runs


def calculate_power_consumption(df: pd.DataFrame) -> float:
    """given a dataframe representing a transition, calculates the total power consumption over the interval the dataframe covers"""

    # calculate wheel power
    wheel_power = 0
    for i in range(1, 5):
        wheel_power += df[f"wheel{i}_vol"].multiply(df[f"wheel{i}_cur"]).abs().sum()/100

    # calculate hebi power
    hebi_power = 0
    for name in hebi_names:
        hebi_power += df[f"hebi_{name}_vol"].multiply(df[f"hebi_{name}_cur_motor"]).abs().sum()

    return wheel_power + hebi_power


def calculate_joint_work(df: pd.DataFrame) -> float:
    """given a dataframe representing a transition, calculates the articulation joint work"""

    joint_work = 0

    for name in hebi_names:
        # multiply joint work by angular displacement
        angular_displacement = df[f"hebi_{name}_pos"].diff()
        joint_work += df[f"hebi_{name}_eff"].multiply(angular_displacement).sum()

    return joint_work


def calculate_transition_pose_change(df: pd.DataFrame) -> List[float]:
    """given a dataframe representing a transition, calculates the heading change as delta [x, y, z, roll, pitch, yaw]"""

    # calculate 3d displacement
    initial_pos = np.array([
        df[f"{vicon_body_name}_x"].iloc[0]/1000,
        df[f"{vicon_body_name}_y"].iloc[0]/1000,
        df[f"{vicon_body_name}_z"].iloc[0]/1000
    ])

    final_pos = np.array([
        df[f"{vicon_body_name}_x"].iloc[-1]/1000,
        df[f"{vicon_body_name}_y"].iloc[-1]/1000,
        df[f"{vicon_body_name}_z"].iloc[-1]/1000
    ])

    displacement = (final_pos - initial_pos).tolist()

    initial_quaternion = [
        df[f"{vicon_body_name}_q1"].iloc[0],
        df[f"{vicon_body_name}_q2"].iloc[0],
        df[f"{vicon_body_name}_q3"].iloc[0],
        df[f"{vicon_body_name}_q4"].iloc[0],
    ]
    final_quaternion = [
        df[f"{vicon_body_name}_q1"].iloc[-1],
        df[f"{vicon_body_name}_q2"].iloc[-1],
        df[f"{vicon_body_name}_q3"].iloc[-1],
        df[f"{vicon_body_name}_q4"].iloc[-1],
    ]

    # Convert quaternions to rotation matrices
    initial_rotation = quat2mat(initial_quaternion)
    final_rotation = quat2mat(final_quaternion)

    # Compute the change in rotation
    rotation_change = np.dot(final_rotation, np.linalg.inv(initial_rotation))

    # Convert the rotation matrix to roll, pitch, and yaw angles
    rotation_change = list(mat2euler(rotation_change, "sxyz"))

    return displacement + rotation_change


if __name__ == "__main__":
    log_path = Path(__file__).parent.parent / "logs" / "7-23-naive" / "7-23-naive-1.csv"
    vicon_path = "/home/minh/pybind_stm_control/logs/7-23-naive/7-23-naive-1.dat"

    vicon_pd = vicon.read_motion(vicon_path)
    telem_pd = read_telem(log_path)
    print(telem_pd.head())

    combined_pd = vicon.merge_vicon_telemetry(telem_pd, vicon_pd)

    print(combined_pd.head())

    # combined_pd = vicon.transform_origin(combined_pd)

    # vicon.plot_motion(combined_pd, "title")

    # transitions = get_transitions(telem_pd)

    # print(calculate_transition_pose_change(vicon_pd))
