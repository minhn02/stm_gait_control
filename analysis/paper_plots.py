from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.figure import Figure

REPO_ROOT = Path(__file__).parent.parent
RESULTS_PATH = REPO_ROOT / "results"

bezier_sources = [
    RESULTS_PATH / "8-5-bezier-balanced",
    RESULTS_PATH / "8-5-bezier-heading",
    RESULTS_PATH / "8-5-bezier-time",
]
bezierway_sources = [
    RESULTS_PATH / "8-5-bezierway-balanced",
    RESULTS_PATH / "8-5-bezierway-heading",
    RESULTS_PATH / "8-5-bezierway-time",
]
linear_sources = [
    RESULTS_PATH / "8-5-linear-balanced",
    RESULTS_PATH / "8-5-linear-heading",
    RESULTS_PATH / "8-5-linear-time",
]
naive_sources = [
    RESULTS_PATH / "8-5-naive-balanced",
]


def combine_dataframes(df_list: List[pd.DataFrame]) -> pd.DataFrame:
    # Combine dataframes in df_list into one dataframe
    return pd.concat(df_list, ignore_index=True)


def average_summary(df_list: List[pd.DataFrame]) -> pd.DataFrame:
    avg_df = df_list[0].copy()

    # The mean column is simply averaged
    avg_df["mean"] = pd.concat([df["mean"] for df in df_list], axis=1).mean(axis=1)

    # Take the extreme min and max across the df_list (not an average)
    avg_df["min"] = pd.concat([df["min"] for df in df_list], axis=1).min(axis=1)
    avg_df["max"] = pd.concat([df["max"] for df in df_list], axis=1).max(axis=1)

    # The average standard deviation is calculated for unequal sample sizes
    # TODO
    return avg_df.set_index("name")


def plot_duration(
    bezier_avg_df: pd.DataFrame,
    bezierway_avg_df: pd.DataFrame,
    linear_avg_df: pd.DataFrame,
    naive_avg_df: pd.DataFrame,
) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "duration_s"

    duration_means = [
        bezier_avg_df.loc[metric_name]["mean"],
        bezierway_avg_df.loc[metric_name]["mean"],
        linear_avg_df.loc[metric_name]["mean"],
        naive_avg_df.loc[metric_name]["mean"],
    ]

    duration_stds = [
        bezier_avg_df.loc[metric_name]["std"],
        bezierway_avg_df.loc[metric_name]["std"],
        linear_avg_df.loc[metric_name]["std"],
        naive_avg_df.loc[metric_name]["std"],
    ]

    duration_mins = [
        bezier_avg_df.loc[metric_name]["min"],
        bezierway_avg_df.loc[metric_name]["min"],
        linear_avg_df.loc[metric_name]["min"],
        naive_avg_df.loc[metric_name]["min"],
    ]

    duration_maxs = [
        bezier_avg_df.loc[metric_name]["max"],
        bezierway_avg_df.loc[metric_name]["max"],
        linear_avg_df.loc[metric_name]["max"],
        naive_avg_df.loc[metric_name]["max"],
    ]

    # max a boxplot with the same data
    ax.bar(
        x=["Bezier", "Bezier Waypoints", "Linear", "Naive"],
        height=duration_means,
        yerr=duration_stds,
        capsize=10,
        # color="white",
        edgecolor="black",
        width=0.5,
    )
    ax.set_ylabel("Duration (s)")
    ax.set_title("Duration of Gait Transitions")
    return fig


def plot_heading(
    bezier_avg_df: pd.DataFrame,
    bezierway_avg_df: pd.DataFrame,
    linear_avg_df: pd.DataFrame,
    naive_avg_df: pd.DataFrame,
) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "heading_change_rad"

    duration_means = [
        bezier_avg_df.loc[metric_name]["mean"],
        bezierway_avg_df.loc[metric_name]["mean"],
        linear_avg_df.loc[metric_name]["mean"],
        naive_avg_df.loc[metric_name]["mean"],
    ]

    duration_stds = [
        bezier_avg_df.loc[metric_name]["std"],
        bezierway_avg_df.loc[metric_name]["std"],
        linear_avg_df.loc[metric_name]["std"],
        naive_avg_df.loc[metric_name]["std"],
    ]

    duration_mins = [
        bezier_avg_df.loc[metric_name]["min"],
        bezierway_avg_df.loc[metric_name]["min"],
        linear_avg_df.loc[metric_name]["min"],
        naive_avg_df.loc[metric_name]["min"],
    ]

    duration_maxs = [
        bezier_avg_df.loc[metric_name]["max"],
        bezierway_avg_df.loc[metric_name]["max"],
        linear_avg_df.loc[metric_name]["max"],
        naive_avg_df.loc[metric_name]["max"],
    ]

    # max a boxplot with the same data
    ax.bar(
        x=["Bezier", "Bezier Waypoints", "Linear", "Naive"],
        height=duration_means,
        yerr=duration_stds,
        capsize=10,
        # color="white",
        edgecolor="black",
        width=0.5,
    )
    ax.set_ylabel("Heading Change (radians)")
    ax.set_title("Change of Heading over Gait Transitions")
    return fig


def plot_power(
    bezier_avg_df: pd.DataFrame,
    bezierway_avg_df: pd.DataFrame,
    linear_avg_df: pd.DataFrame,
    naive_avg_df: pd.DataFrame,
) -> Figure:
    fig, ax = plt.subplots()

    metric_name = "power_wheels_W"

    duration_means = [
        bezier_avg_df.loc[metric_name]["mean"],
        bezierway_avg_df.loc[metric_name]["mean"],
        linear_avg_df.loc[metric_name]["mean"],
        naive_avg_df.loc[metric_name]["mean"],
    ]

    duration_stds = [
        bezier_avg_df.loc[metric_name]["std"],
        bezierway_avg_df.loc[metric_name]["std"],
        linear_avg_df.loc[metric_name]["std"],
        naive_avg_df.loc[metric_name]["std"],
    ]

    duration_mins = [
        bezier_avg_df.loc[metric_name]["min"],
        bezierway_avg_df.loc[metric_name]["min"],
        linear_avg_df.loc[metric_name]["min"],
        naive_avg_df.loc[metric_name]["min"],
    ]

    duration_maxs = [
        bezier_avg_df.loc[metric_name]["max"],
        bezierway_avg_df.loc[metric_name]["max"],
        linear_avg_df.loc[metric_name]["max"],
        naive_avg_df.loc[metric_name]["max"],
    ]

    # max a boxplot with the same data
    ax.bar(
        x=["Bezier", "Bezier Waypoints", "Linear", "Naive"],
        height=duration_means,
        yerr=duration_stds,
        capsize=10,
        # color="white",
        edgecolor="black",
        width=0.5,
    )
    ax.set_ylabel("Average Wheel Power (W)")
    ax.set_title("Cost of Transport for Gait Transitions")
    return fig


if __name__ == "__main__":
    bezier_dfs = [
        pd.read_csv(source.parent / f"{source.name}_summary.csv", header=0)
        for source in bezier_sources
    ]
    bezierway_dfs = [
        pd.read_csv(source.parent / f"{source.name}_summary.csv", header=0)
        for source in bezierway_sources
    ]
    linear_dfs = [
        pd.read_csv(source.parent / f"{source.name}_summary.csv", header=0)
        for source in linear_sources
    ]
    naive_dfs = [
        pd.read_csv(source.parent / f"{source.name}_summary.csv", header=0)
        for source in naive_sources
    ]

    bezier_avg_df = average_summary(bezier_dfs)
    bezierway_avg_df = average_summary(bezierway_dfs)
    linear_avg_df = average_summary(linear_dfs)
    naive_avg_df = average_summary(naive_dfs)

    duration_fig = plot_duration(
        bezier_avg_df, bezierway_avg_df, linear_avg_df, naive_avg_df
    )
    # TODO: Fix to be absolue in the original calcs (before averaging)
    heading_fig = plot_heading(
        bezier_avg_df, bezierway_avg_df, linear_avg_df, naive_avg_df
    )
    power_fig = plot_power(bezier_avg_df, bezierway_avg_df, linear_avg_df, naive_avg_df)
    plt.show()
