"""Functions for reading and processing merged Vicon and telemetry data."""

from typing import List

import color as color
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from scipy.spatial.transform import Rotation as R
from transforms3d.euler import mat2euler
from transforms3d.quaternions import quat2mat
from vicon import BODY_OBJECT_NAME, BOGIE_OBJECT_NAME


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


def transform_origin(df: pd.DataFrame) -> pd.DataFrame:
    """Take the combined vicon + telemetry dataframe, df, and transforms the co-ordinates so the origin is at
    the initial position of the body object, and the axes are rotated relative to the initial orientation of
    the body object offset by the steering angle."""

    # Define the desired co-ordinate system offset from the body object initial orientation
    initial_body_angle = [0.0, 0.0, df["hebi_steer_pos"].iloc[0] / 2]

    # Extract the initial position and quaternion values
    initial_body_position = (
        df[
            [
                f"{BODY_OBJECT_NAME}_x",
                f"{BODY_OBJECT_NAME}_y",
                f"{BODY_OBJECT_NAME}_z",
            ]
        ]
        .iloc[0]
        .values
    )

    initial_body_quaternion = (
        df[
            [
                f"{BODY_OBJECT_NAME}_q1",
                f"{BODY_OBJECT_NAME}_q2",
                f"{BODY_OBJECT_NAME}_q3",
                f"{BODY_OBJECT_NAME}_q4",
            ]
        ]
        .iloc[0]
        .values
    )

    # Convert initial offset Euler angles to a quaternion representing the offset
    offset_quaternion = R.from_euler("xyz", initial_body_angle).as_quat()

    # Apply the transformation to each row of the dataframe
    def transform_row(row):
        for body_name in [BODY_OBJECT_NAME, BOGIE_OBJECT_NAME]:
            position = np.array(
                [row[f"{body_name}_x"], row[f"{body_name}_y"], row[f"{body_name}_z"]]
            )
            quaternion = np.array(
                [
                    row[f"{body_name}_q1"],
                    row[f"{body_name}_q2"],
                    row[f"{body_name}_q3"],
                    row[f"{body_name}_q4"],
                ]
            )

            # Translate position to the initial position
            position -= initial_body_position

            # Apply the offset to the initial quaternion
            initial_quaternion_with_offset = R.from_quat(
                initial_body_quaternion
            ) * R.from_quat(offset_quaternion)

            # Calculate rotation matrix from the relative quaternion
            relative_quaternion = (
                R.from_quat(quaternion) * initial_quaternion_with_offset.inv()
            )
            rotation_matrix = initial_quaternion_with_offset.as_matrix()

            # Apply the rotation to the position
            transformed_position = np.dot(rotation_matrix, position)

            row[f"{body_name}_x"] = transformed_position[0]
            row[f"{body_name}_y"] = transformed_position[1]
            row[f"{body_name}_z"] = transformed_position[2]
            row[f"{body_name}_q1"] = relative_quaternion.as_quat()[0]
            row[f"{body_name}_q2"] = relative_quaternion.as_quat()[1]
            row[f"{body_name}_q3"] = relative_quaternion.as_quat()[2]
            row[f"{body_name}_q4"] = relative_quaternion.as_quat()[3]

        return row

    df_transformed = df.apply(transform_row, axis=1)

    return df_transformed


def plot_motion(df: pd.DataFrame, title: str, arrows: bool = False) -> Figure:
    """Plot the motion of the body and bogie objects in 3D."""

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

    # Add arrows indicating the orientation
    if arrows:
        for index, row in df.iterrows():
            if index % 50 == 0:
                body_pos = (
                    np.array(
                        [
                            row[f"{BODY_OBJECT_NAME}_x"],
                            row[f"{BODY_OBJECT_NAME}_y"],
                            row[f"{BODY_OBJECT_NAME}_z"],
                        ]
                    )
                    / 10
                )
                bogie_pos = (
                    np.array(
                        [
                            row[f"{BOGIE_OBJECT_NAME}_x"],
                            row[f"{BOGIE_OBJECT_NAME}_y"],
                            row[f"{BOGIE_OBJECT_NAME}_z"],
                        ]
                    )
                    / 10
                )

                body_quat = np.array(
                    [
                        row[f"{BODY_OBJECT_NAME}_q1"],
                        row[f"{BODY_OBJECT_NAME}_q2"],
                        row[f"{BODY_OBJECT_NAME}_q3"],
                        row[f"{BODY_OBJECT_NAME}_q4"],
                    ]
                )
                bogie_quat = np.array(
                    [
                        row[f"{BOGIE_OBJECT_NAME}_q1"],
                        row[f"{BOGIE_OBJECT_NAME}_q2"],
                        row[f"{BOGIE_OBJECT_NAME}_q3"],
                        row[f"{BOGIE_OBJECT_NAME}_q4"],
                    ]
                )

                # Get the orientation vector for each body
                body_orientation = R.from_quat(body_quat).apply([0, 0, 8])
                bogie_orientation = R.from_quat(bogie_quat).apply([0, 0, 8])

                # Add arrows to the plot
                ax.quiver(
                    body_pos[0],
                    body_pos[1],
                    body_pos[2],
                    body_orientation[0],
                    body_orientation[1],
                    body_orientation[2],
                    color="red",
                    arrow_length_ratio=0.3,
                )
                ax.quiver(
                    bogie_pos[0],
                    bogie_pos[1],
                    bogie_pos[2],
                    bogie_orientation[0],
                    bogie_orientation[1],
                    bogie_orientation[2],
                    color="green",
                    arrow_length_ratio=0.3,
                )

    # Add labels and a legend
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    ax.set_aspect("equal")
    fig.legend(loc="lower center", ncol=4)

    return fig


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


def calculate_transition_pose_change(df: pd.DataFrame) -> List[float]:
    """given a dataframe representing a transition, calculates the heading change as delta [x, y, z, roll,
    pitch, yaw]"""

    # calculate 3d displacement
    initial_pos = np.array(
        [
            df[f"{BODY_OBJECT_NAME}_x"].iloc[0] / 1000,
            df[f"{BODY_OBJECT_NAME}_y"].iloc[0] / 1000,
            df[f"{BODY_OBJECT_NAME}_z"].iloc[0] / 1000,
        ]
    )

    final_pos = np.array(
        [
            df[f"{BODY_OBJECT_NAME}_x"].iloc[-1] / 1000,
            df[f"{BODY_OBJECT_NAME}_y"].iloc[-1] / 1000,
            df[f"{BODY_OBJECT_NAME}_z"].iloc[-1] / 1000,
        ]
    )

    displacement = (final_pos - initial_pos).tolist()

    initial_quaternion = [
        df[f"{BODY_OBJECT_NAME}_q1"].iloc[0],
        df[f"{BODY_OBJECT_NAME}_q2"].iloc[0],
        df[f"{BODY_OBJECT_NAME}_q3"].iloc[0],
        df[f"{BODY_OBJECT_NAME}_q4"].iloc[0],
    ]
    final_quaternion = [
        df[f"{BODY_OBJECT_NAME}_q1"].iloc[-1],
        df[f"{BODY_OBJECT_NAME}_q2"].iloc[-1],
        df[f"{BODY_OBJECT_NAME}_q3"].iloc[-1],
        df[f"{BODY_OBJECT_NAME}_q4"].iloc[-1],
    ]

    # Convert quaternions to rotation matrices
    initial_rotation = quat2mat(initial_quaternion)
    final_rotation = quat2mat(final_quaternion)

    # Compute the change in rotation
    rotation_change = np.dot(final_rotation, np.linalg.inv(initial_rotation))

    # Convert the rotation matrix to roll, pitch, and yaw angles
    rotation_change = list(mat2euler(rotation_change, "sxyz"))

    return displacement + rotation_change
