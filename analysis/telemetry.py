import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.figure import Figure
from typing import List

hebi_names = ["steer", "bogie"]

def read_telem(path: Path):
    df = pd.read_csv(path)
    for i in range(4):
        df[f"wheel{i+1}_pos"] -= df[f"wheel{i+1}_pos"].iloc[0]
    return df

def get_transitions(df: pd.DataFrame) -> List[pd.DataFrame]:
    """Returns a list of the dataframes of transitions"""

    # Identify the start and end indices of consecutive runs
    starts = df.index[(df['in_transition'] == 1) & (df['in_transition'].shift(1) ==  0)]
    ends = df.index[(df['in_transition'] == 0) & (df['in_transition'].shift(1) == 1)]


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

    #calculate hebi power
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

if __name__ == "__main__":
    log_path = Path(__file__).parent.parent / "logs" / "7-12" / "7-12-minimarsrun.csv"
    telem_pd = read_telem(log_path)
    transitions = get_transitions(telem_pd)
    
    print(calculate_power_consumption(transitions[0]))
    print(calculate_joint_work(transitions[0]))