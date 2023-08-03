import re
from pathlib import Path

import color as color
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from scipy.spatial.transform import Rotation as R

BODY_OBJECT_NAME = "asterix_body"
BOGIE_OBJECT_NAME = "asterix_bogie"


def read_motion(path: Path) -> pd.DataFrame:
    """
    Reads motion data based on given path.
    Skips two rows of data, since motion data always have one row of missing headers and one row of 'OCCL'
    (occlusion) datapoints.
    Headers are added manually by hard-coded header definition in function.
    :param path: Path of motion .dat file.
    :return: Pandas Dataframe of motion data.
    """
    with path.open() as f:
        while "header" not in locals() or len(header) < 1:
            header = f.readline().strip("\n")

    number_position_sensors = int((len(header.split(" ")) - 1) / 8)
    header = header.split(" ")

    sensor_positions = []

    sensor_header = [
        "%s_position",
        "%s_x",
        "%s_y",
        "%s_z",
        "%s_q1",
        "%s_q2",
        "%s_q3",
        "%s_q4",
    ]
    labels = ["time"]
    for sensor_nr in range(number_position_sensors):
        sensor_positions.append(header[1 + 8 * sensor_nr])
        for object in sensor_header:
            labels.append(object % header[1 + 8 * sensor_nr])

    data_temp = pd.read_csv(path, skiprows=2, sep=" ").values
    for sensor_nr in range(number_position_sensors - 1, -1, -1):
        data_temp = np.delete(data_temp, np.s_[1 + 8 * sensor_nr], axis=1)
        del labels[1 + 8 * sensor_nr]
    df_motion = pd.DataFrame(data=data_temp, columns=labels)
    df_motion.attrs["sensor_positions"] = sensor_positions
    del header
    df_motion.replace("OCCL", np.nan, inplace=True)
    df_motion = df_motion.astype(float)
    update_body_names(df_motion)
    return df_motion


def transform_origin(df: pd.DataFrame):
    """Takes the combined vicon + telemetry dataframe, df, and sets the initial position to the origin,
    accounting for the steering and bogie angle"""

    initial_angles = {
        BODY_OBJECT_NAME: [0.0, 0.0, df["hebi_steer_pos"].iloc[0]],
        BOGIE_OBJECT_NAME: [df["hebi_bogie_pos"].iloc[0], 0.0, 0.0],
    }

    for body_name in [BODY_OBJECT_NAME, BOGIE_OBJECT_NAME]:
        # get initial position
        initial_x = df[f"{body_name}_x"].iloc[0]
        initial_y = df[f"{body_name}_y"].iloc[0]
        initial_z = df[f"{body_name}_z"].iloc[0]

        # set initial position to origin
        df[f"{body_name}_x"] = df[f"{body_name}_x"] - initial_x
        df[f"{body_name}_y"] = df[f"{body_name}_y"] - initial_y
        df[f"{body_name}_z"] = df[f"{body_name}_z"] - initial_z

        initial_quaternion = [
            df[f"{body_name}_q1"].iloc[0],
            df[f"{body_name}_q2"].iloc[0],
            df[f"{body_name}_q3"].iloc[0],
            df[f"{body_name}_q4"].iloc[0],
        ]

        r = R.from_quat(initial_quaternion)
        R_intitial = r.as_matrix()

        # set initial rover orientation to steering angle
        R_target = R.from_euler("xyz", initial_angles[body_name]).as_matrix()

        # get rotation matrix from initial to target
        R_transform = np.dot(R_target, np.linalg.inv(R_intitial))

        def transform_quaternion(quaternion):
            rotation = R.from_quat(quaternion)
            rotated_quaternion = rotation.apply(R_transform)
            print("rotated quaternion", rotated_quaternion)
            return rotated_quaternion / np.linalg.norm(rotated_quaternion)

        # transform all quaternion data on the body
        quaternion_columns = [
            f"{body_name}_q1",
            f"{body_name}_q2",
            f"{body_name}_q3",
            f"{body_name}_q4",
        ]
        print(df[quaternion_columns].apply(transform_quaternion, axis=1)[0])
        # df[quaternion_columns] = df[quaternion_columns].apply(transform_quaternion, axis=1)

    return df


def update_body_names(df: pd.DataFrame):
    global BODY_OBJECT_NAME, BOGIE_OBJECT_NAME

    body_pattern = re.compile("asterix_body.*_x")
    bogie_pattern = re.compile("asterix_bogie.*_x")

    BODY_OBJECT_NAME = list(filter(body_pattern.match, df.columns))[0][:-2]
    BOGIE_OBJECT_NAME = list(filter(bogie_pattern.match, df.columns))[0][:-2]


def plot_motion(df: pd.DataFrame, title: str) -> Figure:
    # Create a new figure and a 3D axis
    fig = plt.figure()
    if matplotlib.rcParams["text.usetex"]:
        fig.suptitle(r"\Large{\textbf{Motion Capture}}" "\n" r"\small{" + title + "}")
    else:
        fig.suptitle(f"Motion Capture\n({title})")

    ax = fig.add_subplot(111, projection="3d")

    # Plot the three lines
    ax.plot(
        df[f"{BODY_OBJECT_NAME}_x"] / 10,
        df[f"{BODY_OBJECT_NAME}_y"] / 10,
        df[f"{BODY_OBJECT_NAME}_z"] / 10,
        label="Body",
    )
    ax.plot(
        df[f"{BOGIE_OBJECT_NAME}_x"] / 10,
        df[f"{BOGIE_OBJECT_NAME}_y"] / 10,
        df[f"{BOGIE_OBJECT_NAME}_z"] / 10,
        label="Bogie",
        color=color.front_hebi,
    )

    # Add labels and a legend
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    ax.set_aspect("equal")
    fig.legend(loc="lower center", ncol=4)

    return fig


def merge_vicon_telemetry(
    df_telem: pd.DataFrame, df_vicon: pd.DataFrame
) -> pd.DataFrame:
    """Return the df_telem DataFrame extended to include all columns from df_vicon. The number of rows and
    timestamps will match df_telem, data from df_vicon is interpolated to match these timestamps.
    """

    # Extend df_vicon to have rows for all timestamps in both dataframes (new rows are empty)
    time_index = pd.Index.union(pd.Index(df_telem["time"]), pd.Index(df_vicon["time"]))
    df_vicon = df_vicon.set_index("time")
    df_vicon = df_vicon.reindex(time_index)

    # Fill the empty rows by interpolating the data
    df_vicon_interp = df_vicon.interpolate().reset_index()

    # Remove rows with timestamps not in df_telem
    df_vicon_matched = df_vicon_interp[
        df_vicon_interp["time"].isin(df_telem["time"])
    ].reset_index(drop=True)

    merged_df = df_telem.merge(df_vicon_matched)

    return merged_df


def plot_motion_2d(df: pd.DataFrame, title: str) -> Figure:
    # Create a new figure and three subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    if matplotlib.rcParams["text.usetex"]:
        fig.suptitle(
            r"\Large{\textbf{Motion Capture - Body Position}}"
            "\n"
            r"\small{" + title + "}"
        )
    else:
        fig.suptitle(f"Motion Capture - Body Position\n({title})")

    # Plot the data on each subplot
    ax1.plot(df["time"], df[f"{BODY_OBJECT_NAME}_x"] / 10)
    ax2.plot(df["time"], df[f"{BODY_OBJECT_NAME}_y"] / 10)
    ax3.plot(df["time"], df[f"{BODY_OBJECT_NAME}_z"] / 10)

    ax3.set_xlabel("Time (s)")
    ax1.set_ylabel("X (cm)")
    ax2.set_ylabel("Y (cm)")
    ax3.set_ylabel("Z (cm)")

    ax1.set_xlim(df["time"].iloc[0], df["time"].iloc[-1])

    for ax in (ax1, ax2, ax3):
        ax.grid(True, which="both", color="lightgray", linestyle="--", linewidth="0.5")
        ax.minorticks_on()

    return fig


if __name__ == "__main__":
    path = Path("/home/minh/pybind_stm_control/logs/7-23-naive/7-23-naive-1.dat")
    df = read_motion(path)
    print(df.head)
    plot_motion(df, "Sample Data")
    plot_motion_2d(df, "Sample Data")
    plt.show()
