import math
import re
import sys
from enum import Enum
from pathlib import Path
from typing import Callable, List, Tuple, Union

import analysis
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import telemetry
import vicon
from rich import print
from rich.console import Console
from rich.table import Table

REPO_ROOT = Path(__file__).parent.parent
RESULTS_PATH = REPO_ROOT / "results"
LOGS_PATH = REPO_ROOT / "logs"

SUMMARY_EXAMPLE_PATH = REPO_ROOT / "logs" / "8-5-bezierway-heading"

LOGPAIR_EXAMPLE_PATH_BASE = (
    REPO_ROOT / "logs" / "8-5-bezierway-heading" / "8-5-bezierway-heading-2"
)
VICON_EXAMPLE_PATH = LOGPAIR_EXAMPLE_PATH_BASE.with_suffix(".dat")
TELEM_EXAMPLE_PATH = LOGPAIR_EXAMPLE_PATH_BASE.with_suffix(".csv")

# Enum for each type of analysis
ANALYSIS_TYPE = Enum("ANALYSIS_TYPE", "LOG_PAIR DIR_SUMMARY ALL_SUMMARIES")

DEFAULT_ANALYSIS = ANALYSIS_TYPE.LOG_PAIR

# offset vicon time in 8-5 data due to improper epoch synching (keys are regex patterns)
VICON_OFFSETS = {
    "^8-5": 1691279982.08448 - 1691280146.5914912,
}


def apply_vicon_time_offset(vicon_path: Path, vicon_df: pd.DataFrame) -> pd.DataFrame:
    """Offset vicon data if vicon path stem matches the regex pattern of a time offset."""

    for key, value in VICON_OFFSETS.items():
        if re.match(key, vicon_path.stem):
            vicon_df = vicon.offset_vicon_data_time(vicon_df, value)
            return vicon_df


def preprocess_data(
    vicon_path: Path, telem_path: Path, plot_merged: bool = False
) -> pd.DataFrame:
    """Read, offset, merge, and transform a pair of vicon and telemetry datafiles.
    Return the transformed dataframe."""

    vicon_df = vicon.read_motion(vicon_path)
    vicon_df = apply_vicon_time_offset(vicon_path, vicon_df)
    telem_df = telemetry.read_telem(telem_path)
    merged_df = analysis.merge_vicon_telemetry(telem_df, vicon_df)
    transformed_df = analysis.flatten_along_x(merged_df)

    if plot_merged:
        analysis.plot_motion(merged_df, "Original Data", show_arrows=True)
        analysis.plot_motion_2d(merged_df, "Original Data")

    return transformed_df


def preprocess_dir(dir_path: Path) -> List[pd.DataFrame]:
    """Find matching pairs of telemetry and vicon datafiles in a directory and preprocess them.
    Return a list of dataframes, one for each file pair."""

    # Find matching pairs of telemetry and vicon datafiles
    csv_files = [f for f in dir_path.iterdir() if f.suffix == ".csv"]
    log_file_pairs = [
        (
            next(
                f
                for f in dir_path.iterdir()
                if f.suffix == ".dat" and f.stem == csv.stem
            ),
            csv,
        )
        for csv in csv_files
    ]
    transformed_dfs = [
        preprocess_data(vicon_path, telem_path)
        for vicon_path, telem_path in log_file_pairs
    ]
    return transformed_dfs


class Metric:
    """A metric to be calculated on a transition dataframe."""

    def __init__(
        self, name: str, fnc: Callable[[pd.DataFrame], Union[float, Tuple[float, ...]]]
    ) -> None:
        self.name = name
        self.fnc = fnc
        self.vals: List[Union[float, Tuple[float, ...]]] = []


def summarise_runs(dir_path: Path, results_dir_path: Path):
    """Summarise the results of a directory of runs. Save results to csv files."""

    transformed_dfs = preprocess_dir(dir_path)

    metrics = [
        Metric("duration_s", analysis.calc_transition_duration),
        Metric("power_wheels_W", analysis.calc_transition_power_wheels),
        Metric("power_hebis_W", analysis.calc_transition_power_hebis),
        Metric("power_W", telemetry.calculate_power_consumption),
        Metric("pos_change_xyz_m", analysis.calc_transition_pos_change_xyz),
        Metric("heading_change_rad", analysis.calc_heading_change),
    ]

    for transformed_df in transformed_dfs:
        transitions, gaits = analysis.get_transitions(transformed_df)
        for transition_df in transitions:
            for metric in metrics:
                metric.vals.append(metric.fnc(transition_df))

    all_results_df = pd.DataFrame()

    summary_results = []
    for metric in metrics:
        all_results_df[metric.name] = metric.vals

        if isinstance(metric.vals[0], tuple):
            vals = np.linalg.norm(metric.vals, axis=1)
            name = f"{metric.name}_norm"
        else:
            vals = metric.vals
            name = metric.name

        summary_results.append(
            {
                "name": name,
                "mean": np.mean(vals),
                "std": np.std(vals),
                "min": np.min(vals),
                "max": np.max(vals),
                "sample_size": np.size(vals),
            }
        )

    summary_results_df = pd.DataFrame(summary_results)

    # Save results to csvs
    all_results_path = results_dir_path / f"{dir_path.name}_all.csv"
    all_results_df.to_csv(all_results_path, index=False)
    summary_results_path = results_dir_path / f"{dir_path.name}_summary.csv"
    summary_results_df.to_csv(summary_results_path, index=False)


def run_directory_summary(dir_path: Path):
    if dir_path.is_dir():
        summarise_runs(dir_path, RESULTS_PATH)
    else:
        raise FileNotFoundError(f"{dir_path} is not a directory.")


def run_all_directory_summaries():
    for dir_path in LOGS_PATH.iterdir():
        if dir_path.is_dir():
            # check dir_path has csv and dat files
            csv_files = [f for f in dir_path.iterdir() if f.suffix == ".csv"]
            dat_files = [f for f in dir_path.iterdir() if f.suffix == ".dat"]
            if dir_path.stem.startswith("8-4"):
                # Error when mergin 8-4-bezierway, so have excluded 8-4 files for now
                continue
            if len(csv_files) > 0 and len(dat_files) > 0:
                print(f"Summarising {dir_path.name}")
                summarise_runs(dir_path, RESULTS_PATH)


def run_log_pair_summary(vicon_path: Path, telem_path: Path):
    if not vicon_path.is_file() or not telem_path.is_file():
        raise FileNotFoundError(f"{vicon_path} or {telem_path} is not a file.")

    transformed_df = preprocess_data(vicon_path, telem_path, plot_merged=True)

    analysis.plot_motion(transformed_df, "Transformed Data", show_arrows=True)
    analysis.plot_motion_2d(transformed_df, "Transformed Data")
    plt.show()

    pose_changes = analysis.summarize_pose_changes(transformed_df)

    table = Table(title=f"Transition Pose Changes - {vicon_path.stem}")
    table.add_column("#", justify="center", style="bold")
    table.add_column("Δ x (cm)", justify="right")
    table.add_column("Δ y (cm)", justify="right")
    table.add_column("Δ z (cm)", justify="right")
    table.add_column("Δ roll (°)", justify="right")
    table.add_column("Δ pitch (°)", justify="right")
    table.add_column("Δ yaw (°)", justify="right")

    i = 1
    for dx, dy, dz, droll, dpitch, dyaw in pose_changes:
        table.add_row(
            str(i),
            f"{dx*100:.2f}",
            f"{dy*100:.2f}",
            f"{dz*100:.2f}",
            f"{math.degrees(droll):.2f}",
            f"{math.degrees(dpitch):.2f}",
            f"{math.degrees(dyaw):.2f}",
        )
        i += 1
    table.add_section()
    table.add_row(
        "Avg |Δ|",
        f"{sum([abs(dx) for dx, _, _, _, _, _ in pose_changes])/len(pose_changes)*100:.2f}",
        f"{sum([abs(dy) for _, dy, _, _, _, _ in pose_changes])/len(pose_changes)*100:.2f}",
        f"{sum([abs(dz) for _, _, dz, _, _, _ in pose_changes])/len(pose_changes)*100:.2f}",
        f"{math.degrees(sum([abs(droll) for _, _, _, droll, _, _ in pose_changes]))/len(pose_changes):.2f}",
        f"{math.degrees(sum([abs(dpitch) for _, _, _, _, dpitch, _ in pose_changes]))/len(pose_changes):.2f}",
        f"{math.degrees(sum([abs(dyaw) for _, _, _, _, _, dyaw in pose_changes]))/len(pose_changes):.2f}",
    )

    print()
    console = Console()
    console.print(table)

    analysis.plot_pose_changes(transformed_df, f"{vicon_path.stem}")
    analysis.plot_hebi_steer_pos(transformed_df)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        run_directory_summary(Path(sys.argv[1]))

    elif len(sys.argv) == 3:
        run_log_pair_summary(Path(sys.argv[1]), Path(sys.argv[2]))

    elif len(sys.argv) == 1:
        if DEFAULT_ANALYSIS == ANALYSIS_TYPE.LOG_PAIR:
            run_log_pair_summary(VICON_EXAMPLE_PATH, TELEM_EXAMPLE_PATH)

        elif DEFAULT_ANALYSIS == ANALYSIS_TYPE.DIR_SUMMARY:
            run_directory_summary(SUMMARY_EXAMPLE_PATH)

        elif DEFAULT_ANALYSIS == ANALYSIS_TYPE.ALL_SUMMARIES:
            run_all_directory_summaries()

        else:
            raise ValueError(f"Invalid default analysis type: {DEFAULT_ANALYSIS}")

    else:
        raise ValueError(
            "Only one directory path argument or two file path arguments should be provided."
        )
