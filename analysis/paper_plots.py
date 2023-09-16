from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

REPO_ROOT = Path(__file__).parent.parent
RESULTS_PATH = REPO_ROOT / "results"

transition_names = [
    "bezier-balanced",
    "bezier-heading",
    "bezier-time",
    "bezierway-balanced",
    "bezierway-heading",
    "bezierway-time",
    "linear-balanced",
    "linear-heading",
    "linear-time",
    "naive-balanced",
]


def plot_duration(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "duration_s"
    ax.set_title("Duration of Gait Transitions")
    ax.set_ylabel("Duration (s)")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=transition_names,
        height=means,
        yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.5,
    )

    return fig


def plot_heading(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "heading_change_rad"
    ax.set_title("Heading Change of Gait Transitions")
    ax.set_ylabel("Heading Change (deg)")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=transition_names,
        height=np.degrees(means),
        yerr=np.degrees(stds),
        capsize=5,
        edgecolor="black",
        width=0.5,
    )

    return fig


def plot_cot(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "cost_of_transport"
    ax.set_title("Cost of Transport for Gait Transitions")
    ax.set_ylabel("Cost of Transport [E/mgd]")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=transition_names,
        height=means,
        yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.5,
    )

    return fig


if __name__ == "__main__":
    summary_source_files = {
        transition_name: RESULTS_PATH / f"8-5-{transition_name}_summary.csv"
        for transition_name in transition_names
    }

    transition_data = {
        transition_name: pd.read_csv(source_file, header=0).set_index("name")
        for transition_name, source_file in summary_source_files.items()
    }

    duration_fig = plot_duration(transition_data)
    heading_fig = plot_heading(transition_data)
    power_fig = plot_cot(transition_data)
    plt.show()
