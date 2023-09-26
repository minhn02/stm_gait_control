from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

REPO_ROOT = Path(__file__).parent.parent
RESULTS_PATH = REPO_ROOT / "results"
PLOTS_PATH = REPO_ROOT / "plots"

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

label_names = [
    "bezier-balanced",
    "bezier-heading",
    "bezier-time",
    "spline-balanced",
    "spline-heading",
    "spline-time",
    "linear-balanced",
    "linear-heading",
    "linear-time",
    "naive-balanced",
]

bezier_color = "#003f5c"
spline_color = "#7a5195"
linear_color = "#ef5675"
naive_color = "#ffa600"

bezier_color2 = "#c2e7ff"
spline_color2 = "#f1dbfe"
linear_color2 = "#ffd5da"
naive_color2 = "#fbddbe"

bar_colors = [
    bezier_color,
    bezier_color,
    bezier_color,
    spline_color,
    spline_color,
    spline_color,
    linear_color,
    linear_color,
    linear_color,
    naive_color,
]

secondary_bar_colors = [
    bezier_color2,
    bezier_color2,
    bezier_color2,
    spline_color2,
    spline_color2,
    spline_color2,
    linear_color2,
    linear_color2,
    linear_color2,
    naive_color2,
]


def plot_duration(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "duration_s"
    ax.set_title("Duration of Gait Transitions")
    ax.set_ylabel("Duration (s)")
    ax.yaxis.set_major_formatter("{x:.1f}")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=label_names,
        height=means,
        # yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.5,
        color=bar_colors,
    )

    return fig


def plot_heading(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "heading_change_rad"
    ax.set_title("Heading Change of Gait Transitions")
    ax.set_ylabel("Heading Change (deg)")
    ax.yaxis.set_major_formatter("{x:.1f}")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=label_names,
        height=np.degrees(means),
        # yerr=np.degrees(stds),
        capsize=5,
        edgecolor="black",
        width=0.5,
        color=bar_colors,
    )

    return fig


def plot_cot(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "cost_of_transport"
    ax.set_title("Cost of Transport for Gait Transitions")
    ax.set_ylabel("Cost of Transport [E/mgd]")
    ax.yaxis.set_major_formatter("{x:.1f}")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means = [df.loc[metric_name]["mean"] for df in transition_data.values()]
    stds = [df.loc[metric_name]["std"] for df in transition_data.values()]

    # Create a lower bound on the errors at 0
    stds = [np.minimum(stds, means), stds]

    ax.bar(
        x=label_names,
        height=means,
        # yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.5,
        color=bar_colors,
    )

    return fig


def plot_max_joint_accel(transition_data: dict[str, pd.DataFrame]) -> Figure:
    fig, ax = plt.subplots()

    metric_name1 = "max_accel_steer_radps2"
    metric_name2 = "max_accel_bogie_radps2"
    ax.set_title("Maximum Joint Accelerations")
    ax.set_ylabel("Max Joint Acceleration [rad/s$^2$]")
    ax.yaxis.set_major_formatter("{x:.1f}")
    ax.xaxis.set_tick_params(rotation=45)
    plt.xticks(ha="right")
    plt.subplots_adjust(bottom=0.3)

    means1 = [df.loc[metric_name1]["mean"] for df in transition_data.values()]
    means2 = [df.loc[metric_name2]["mean"] for df in transition_data.values()]

    x_axis = np.arange(len(label_names))

    ax.bar(
        x=x_axis - 0.15,
        height=means1,
        # yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.3,
        color=bar_colors,
        label="Steering Joint",
    )
    ax.bar(
        x=x_axis + 0.15,
        height=means2,
        # yerr=stds,
        capsize=5,
        edgecolor="black",
        width=0.3,
        color=secondary_bar_colors,
        label="Bogie Joint",
    )

    plt.xticks(x_axis, label_names)
    plt.legend()

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
    joint_accel_fig = plot_max_joint_accel(transition_data)
    plt.show()

    # if PLOTS_PATH does not exist, make a folder
    if not PLOTS_PATH.exists():
        PLOTS_PATH.mkdir()

    duration_fig.savefig(PLOTS_PATH / "gaits_duration.png", dpi=300)
    heading_fig.savefig(PLOTS_PATH / "gaits_heading.png", dpi=300)
    power_fig.savefig(PLOTS_PATH / "gaits_cot.png", dpi=300)
    joint_accel_fig.savefig(PLOTS_PATH / "gaits_joint_accel.png", dpi=300)
