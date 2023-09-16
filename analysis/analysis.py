"""Functions for reading and processing merged Vicon and telemetry data."""

import math
from typing import List, Tuple

import color as color
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import telemetry
from matplotlib.figure import Figure
from scipy import integrate
from scipy.spatial.transform import Rotation as R
from sklearn.linear_model import LinearRegression
from vicon import BODY_OBJECT_NAME, BOGIE_OBJECT_NAME

gait_colors = {
    "WHEEL_WALKING": color.gait_wheel_walking,
    "SQUIRM": color.gait_squirm,
}

HEBI_NAMES = ["hebi_steer", "hebi_bogie"]  # From headings in telemetry data


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
    initial_yaw = -df["hebi_steer_pos"].iloc[0] / 2

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

    # find transformation matrix from vicon origin to desired origin: [x: 0, y: 0, z: 0], [r: 0, p: 0, y: steering_angle/2]
    vicon_R_body0 = R.from_quat(initial_body_quaternion).as_matrix()
    vicon_T_body0 = initial_body_position

    vicon_H_body0 = np.eye(4)
    vicon_H_body0[:3, :3] = vicon_R_body0
    vicon_H_body0[:3, 3] = vicon_T_body0

    desired_R_body0 = R.from_euler("z", initial_yaw).as_matrix()
    desired_H_body0 = np.eye(4)
    desired_H_body0[:3, :3] = desired_R_body0

    desired_H_vicon = desired_H_body0 @ np.linalg.inv(vicon_H_body0)

    # Apply the transformation to each row of the dataframe
    def transform_row(row):
        for body_name in [BODY_OBJECT_NAME, BOGIE_OBJECT_NAME]:
            position = np.array(
                [row[f"{body_name}_x"], row[f"{body_name}_y"], row[f"{body_name}_z"], 1]
            )
            quaternion = np.array(
                [
                    row[f"{body_name}_q1"],
                    row[f"{body_name}_q2"],
                    row[f"{body_name}_q3"],
                    row[f"{body_name}_q4"],
                ]
            )

            vicon_R_bodyt = R.from_quat(quaternion).as_matrix()
            vicon_T_bodyt = position.T

            vicon_H_bodyt = np.eye(4)
            vicon_H_bodyt[:3, :3] = vicon_R_bodyt
            vicon_H_bodyt[:, -1] = vicon_T_bodyt

            desired_H_bodyt = desired_H_vicon @ vicon_H_bodyt
            desired_R_bodyt = desired_H_bodyt[:3, :3]
            transformed_position = desired_H_bodyt[:, -1]
            transformed_quaternion = R.from_matrix(desired_R_bodyt).as_quat()

            row[f"{body_name}_x"] = transformed_position[0]
            row[f"{body_name}_y"] = transformed_position[1]
            row[f"{body_name}_z"] = transformed_position[2]
            row[f"{body_name}_q1"] = transformed_quaternion[0]
            row[f"{body_name}_q2"] = transformed_quaternion[1]
            row[f"{body_name}_q3"] = transformed_quaternion[2]
            row[f"{body_name}_q4"] = transformed_quaternion[3]

        return row

    df_transformed = df.apply(transform_row, axis=1)

    return df_transformed


def flatten_along_x(df: pd.DataFrame) -> pd.DataFrame:
    pose_data = [
        df[f"{BODY_OBJECT_NAME}_x"].values,
        df[f"{BODY_OBJECT_NAME}_y"].values,
        df[f"{BODY_OBJECT_NAME}_z"].values,
    ]

    reg = LinearRegression().fit(pose_data[0].reshape(-1, 1), pose_data[1])
    correction_rot_matrix = R.from_rotvec(
        np.arctan(reg.coef_) * np.array([0, 0, 1])
    ).as_matrix()

    def transform_row(row):
        for body_name in [BODY_OBJECT_NAME, BOGIE_OBJECT_NAME]:
            position = np.array(
                [
                    row[f"{body_name}_x"],
                    row[f"{body_name}_y"],
                    row[f"{body_name}_z"],
                ]
            )
            corrected_position = correction_rot_matrix @ position
            row[f"{body_name}_x"] = corrected_position[0]
            row[f"{body_name}_y"] = corrected_position[1]
            row[f"{body_name}_z"] = corrected_position[2]
        return row

    df_transformed = df.apply(transform_row, axis=1)
    return df_transformed


def get_transitions(df: pd.DataFrame) -> Tuple[List[pd.DataFrame], List[pd.DataFrame]]:
    """Return a tuple, the first element is a list of dataframes which are subsets of the given df, one for
    each continous set of timestamps during a transition, the second element is the same but for timestamps
    outside of transitions.
    """

    # Identify the start and end indices of consecutive runs
    transition_starts = df.index[
        (df["in_transition"] == 1) & (df["in_transition"].shift(1) == 0)
    ]
    transition_ends = df.index[
        (df["in_transition"] == 0) & (df["in_transition"].shift(1) == 1)
    ]

    gait_starts = [0]
    gait_starts.extend(transition_ends)
    gait_ends = transition_starts

    # Handle the case when the last run is ongoing
    if len(transition_ends) < len(transition_starts):
        transition_ends = transition_ends.append(pd.Index([df.index[-1]]))

    if len(gait_ends) < len(gait_starts):
        gait_ends = gait_ends.append(pd.Index([df.index[-1]]))

    transitions = [
        df.iloc[start:end] for start, end in zip(transition_starts, transition_ends)
    ]
    gaits = [df.iloc[start:end] for start, end in zip(gait_starts, gait_ends)]
    return transitions, gaits


def plot_motion(
    df: pd.DataFrame,
    title: str,
    show_arrows: bool = False,
    show_transitions: bool = True,
) -> Figure:
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

    # Highlight transitions
    if show_transitions:
        transitions, gaits = get_transitions(df)
        for transition in transitions:
            ax.plot(
                transition[f"{BODY_OBJECT_NAME}_x"] / 10,
                transition[f"{BODY_OBJECT_NAME}_y"] / 10,
                transition[f"{BODY_OBJECT_NAME}_z"] / 10,
                color="red",
            )
            ax.plot(
                transition[f"{BOGIE_OBJECT_NAME}_x"] / 10,
                transition[f"{BOGIE_OBJECT_NAME}_y"] / 10,
                transition[f"{BOGIE_OBJECT_NAME}_z"] / 10,
                color="red",
            )
        for gait in gaits:
            try:
                ax.plot(
                    gait[f"{BODY_OBJECT_NAME}_x"] / 10,
                    gait[f"{BODY_OBJECT_NAME}_y"] / 10,
                    gait[f"{BODY_OBJECT_NAME}_z"] / 10,
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax.plot(
                    gait[f"{BOGIE_OBJECT_NAME}_x"] / 10,
                    gait[f"{BOGIE_OBJECT_NAME}_y"] / 10,
                    gait[f"{BOGIE_OBJECT_NAME}_z"] / 10,
                    color=gait_colors[gait["gait_name"].iloc[0]]["bogie"],
                )
            except KeyError:
                # This happens when "gait_name" does not have a defined color
                pass

    # Add arrows indicating the orientation
    if show_arrows:
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
                body_orientation = R.from_quat(body_quat).apply([8, 0, 0])
                bogie_orientation = R.from_quat(bogie_quat).apply([8, 0, 0])

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
    fig.legend(loc="lower center", ncol=2)

    return fig


def plot_motion_2d(
    df: pd.DataFrame, title: str, show_transitions: bool = True
) -> Figure:
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

    # Highlight transitions
    if show_transitions:
        transitions, gaits = get_transitions(df)
        for transition in transitions:
            ax1.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_x"] / 10,
                color="red",
            )
            ax2.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_y"] / 10,
                color="red",
            )
            ax3.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_z"] / 10,
                color="red",
            )
        for gait in gaits:
            try:
                ax1.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_x"] / 10,
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax2.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_y"] / 10,
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax3.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_z"] / 10,
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
            except KeyError:
                # This happens when "gait_name" does not have a defined color
                pass

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

    # TODO change to euler method by averaging the front and back bodies

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

    initial_steer_angle = df["hebi_steer_pos"].iloc[0]
    final_steer_angle = df["hebi_steer_pos"].iloc[-1]

    heading0_R_body0 = R.from_euler("z", initial_steer_angle / 2).as_matrix()
    heading0_H_body0 = np.eye(4)
    heading0_H_body0[:3, :3] = heading0_R_body0

    headingf_R_bodyf = R.from_euler("z", final_steer_angle / 2).as_matrix()
    headingf_H_bodyf = np.eye(4)
    headingf_H_bodyf[:3, :3] = headingf_R_bodyf

    # Convert initial and final poses to transformation matrices
    world_R_body0 = R.from_quat(initial_quaternion).as_matrix()
    world_T_body0 = initial_pos

    world_H_body0 = np.eye(4)
    world_H_body0[:3, :3] = world_R_body0
    world_H_body0[:3, -1] = world_T_body0

    world_R_bodyf = R.from_quat(final_quaternion).as_matrix()
    world_T_bodyf = final_pos

    world_H_bodyf = np.eye(4)
    world_H_bodyf[:3, :3] = world_R_bodyf
    world_H_bodyf[:3, -1] = world_T_bodyf

    # compute change the final and initial poses in the heading frame
    world_H_heading0 = world_H_body0 @ np.linalg.inv(heading0_H_body0)
    world_H_headingf = world_H_bodyf @ np.linalg.inv(headingf_H_bodyf)

    heading0_H_headingf = np.linalg.inv(world_H_heading0) @ world_H_headingf

    displacement = heading0_H_headingf[:3, -1].tolist()
    rotation_change_matrix = heading0_H_headingf[:3, :3]
    rotation_change = R.from_matrix(rotation_change_matrix).as_rotvec().tolist()

    return displacement + rotation_change


def summarize_pose_changes(df: pd.DataFrame) -> List[List[float]]:
    transitions, gaits = get_transitions(df)
    res = []
    for transition in transitions:
        res.append(calculate_transition_pose_change(transition))
    return res


def plot_pose_changes(
    df: pd.DataFrame, title: str, show_transitions: bool = True
) -> Figure:
    def add_orientation(row):
        quaternion_front = [
            row[f"{BODY_OBJECT_NAME}_q1"],
            row[f"{BODY_OBJECT_NAME}_q2"],
            row[f"{BODY_OBJECT_NAME}_q3"],
            row[f"{BODY_OBJECT_NAME}_q4"],
        ]

        quaternion_back = [
            row[f"{BOGIE_OBJECT_NAME}_q1"],
            row[f"{BOGIE_OBJECT_NAME}_q2"],
            row[f"{BOGIE_OBJECT_NAME}_q3"],
            row[f"{BOGIE_OBJECT_NAME}_q4"],
        ]

        front_rotation = R.from_quat(quaternion_front).as_euler("xyz").tolist()
        back_rotation = R.from_quat(quaternion_back).as_euler("xyz").tolist()

        (
            row[f"{BODY_OBJECT_NAME}_roll"],
            row[f"{BODY_OBJECT_NAME}_pitch"],
            row[f"{BODY_OBJECT_NAME}_yaw"],
        ) = front_rotation

        (
            row[f"{BOGIE_OBJECT_NAME}_roll"],
            row[f"{BOGIE_OBJECT_NAME}_pitch"],
            row[f"{BOGIE_OBJECT_NAME}_yaw"],
        ) = back_rotation

        row[f"{BODY_OBJECT_NAME}_heading"] = (
            row[f"{BODY_OBJECT_NAME}_yaw"] + row[f"{BOGIE_OBJECT_NAME}_yaw"]
        ) / 2

        return row

    df = df.apply(add_orientation, axis=1)

    # Create a new figure and three subplots
    fig, ((ax1, ax5), (ax2, ax6), (ax3, ax7), (ax4, ax8)) = plt.subplots(
        4, 2, sharex=True
    )
    if matplotlib.rcParams["text.usetex"]:
        fig.suptitle(
            r"\Large{\textbf{Pose Changes - Body}}" "\n" r"\small{" + title + "}"
        )
    else:
        fig.suptitle(f"Pose Changes - Bodies\n({title})")

    # Plot the data in each subplot
    ax1.plot(
        df["time"],
        df[f"{BODY_OBJECT_NAME}_roll"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax2.plot(
        df["time"],
        df[f"{BODY_OBJECT_NAME}_pitch"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax3.plot(
        df["time"],
        df[f"{BODY_OBJECT_NAME}_yaw"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax4.plot(
        df["time"],
        df[f"{BODY_OBJECT_NAME}_heading"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax5.plot(
        df["time"],
        df[f"{BOGIE_OBJECT_NAME}_roll"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax6.plot(
        df["time"],
        df[f"{BOGIE_OBJECT_NAME}_pitch"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )
    ax7.plot(
        df["time"],
        df[f"{BOGIE_OBJECT_NAME}_yaw"].astype(float).apply(math.degrees),
        linewidth=0.7,
    )

    # Highlight transitions
    if show_transitions:
        transitions, gaits = get_transitions(df)
        for transition in transitions:
            ax1.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_roll"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
            ax5.plot(
                transition["time"],
                transition[f"{BOGIE_OBJECT_NAME}_roll"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
            ax2.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_pitch"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
            ax6.plot(
                transition["time"],
                transition[f"{BOGIE_OBJECT_NAME}_pitch"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
            ax3.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_yaw"].astype(float).apply(math.degrees),
                color="red",
            )
            ax7.plot(
                transition["time"],
                transition[f"{BOGIE_OBJECT_NAME}_yaw"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
            ax4.plot(
                transition["time"],
                transition[f"{BODY_OBJECT_NAME}_heading"]
                .astype(float)
                .apply(math.degrees),
                color="red",
            )
        for gait in gaits:
            try:
                ax1.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_roll"].astype(float).apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax5.plot(
                    gait["time"],
                    gait[f"{BOGIE_OBJECT_NAME}_roll"].astype(float).apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax2.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_pitch"].astype(float).apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax6.plot(
                    gait["time"],
                    gait[f"{BOGIE_OBJECT_NAME}_pitch"]
                    .astype(float)
                    .apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax3.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_yaw"].astype(float).apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax7.plot(
                    gait["time"],
                    gait[f"{BOGIE_OBJECT_NAME}_yaw"].astype(float).apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
                ax4.plot(
                    gait["time"],
                    gait[f"{BODY_OBJECT_NAME}_heading"]
                    .astype(float)
                    .apply(math.degrees),
                    color=gait_colors[gait["gait_name"].iloc[0]]["body"],
                )
            except KeyError:
                # This happens when "gait_name" does not have a defined color
                pass

    # Set the labels and limits
    ax4.set_xlabel("Time (s)")
    ax8.set_xlabel("Time (s)")
    ax1.set_ylabel("Front Roll (deg)")
    ax5.set_ylabel("Back Roll (deg)")
    ax2.set_ylabel("Front Pitch (deg)")
    ax6.set_ylabel("Back Pitch (deg)")
    ax3.set_ylabel("Front Yaw (deg)")
    ax7.set_ylabel("Back Yaw (deg)")
    ax4.set_ylabel("Heading (deg)")

    ax1.set_xlim(df["time"].iloc[0], df["time"].iloc[-1])

    for ax in (ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8):
        ax.grid(True, which="both", color="lightgray", linestyle="--", linewidth="0.5")
        ax.minorticks_on()

    return fig


def plot_hebi_steer_pos(df: pd.DataFrame) -> Figure:
    """Plot the steer position of the HEBI actuator (for sanity checking)."""
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title("HEBI Steer Position")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steer Position (deg)")
    ax.plot(df["time"], df[f"hebi_steer_pos"].astype(float).apply(math.degrees))
    ax.set_xlim(df["time"].iloc[0], df["time"].iloc[-1])
    ax.grid(True, which="both", color="lightgray", linestyle="--", linewidth="0.5")
    ax.minorticks_on()
    transitions, gaits = get_transitions(df)
    for transition in transitions:
        ax.plot(
            transition["time"],
            transition[f"hebi_steer_pos"].astype(float).apply(math.degrees),
            color="red",
        )
    return fig


def calc_transition_duration(df: pd.DataFrame) -> float:
    return df["time"].iloc[-1] - df["time"].iloc[0]


def calc_transition_power_wheels(df: pd.DataFrame) -> float:
    duration_s = calc_transition_duration(df)

    wheels_energy_J = []
    for wheel_num in range(4):
        wheel_current_A = df[f"wheel{wheel_num+1}_cur"] / 1000
        wheel_voltage_V = df[f"wheel{wheel_num+1}_vol"]
        wheel_power_W = wheel_current_A * wheel_voltage_V
        wheel_energy_J = integrate.trapezoid(wheel_power_W, df["time"])
        wheels_energy_J.append(wheel_energy_J)
    total_wheels_energy_J = np.sum(wheels_energy_J)

    return total_wheels_energy_J / duration_s


def calc_transition_power_hebis(df: pd.DataFrame) -> float:
    duration_s = calc_transition_duration(df)

    total_hebis_energy_J = 0
    for hebi_name in HEBI_NAMES:
        hebi_current_A = df[f"{hebi_name}_cur_motor"]
        hebi_voltage_V = df[f"{hebi_name}_vol"]
        hebi_power_W = hebi_current_A * hebi_voltage_V
        hebi_energy_J = integrate.trapezoid(hebi_power_W, df["time"])
        total_hebis_energy_J += hebi_energy_J

    return total_hebis_energy_J / duration_s


def calc_transition_pos_change_xyz(df: pd.DataFrame) -> Tuple[float, float, float]:
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
    return tuple(final_pos - initial_pos)


def calc_heading_change(df: pd.DataFrame) -> float:
    # TODO: Validate this (unsure if dyaw is the relevant metric)
    dx, dy, dz, droll, dpitch, dyaw = calculate_transition_pose_change(df)
    return dyaw


def calc_cost_of_transport(df: pd.DataFrame) -> float:
    energy_J = telemetry.calculate_energy_consumption(df)
    mass_kg = 7.8
    g = 9.81
    pos_change = calc_transition_pos_change_xyz(df)
    planar_dist_m = math.sqrt(pos_change[0] ** 2 + pos_change[1] ** 2)
    return energy_J / (mass_kg * g * planar_dist_m)
