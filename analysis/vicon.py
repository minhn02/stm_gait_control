"""Functions for reading and processing Vicon motion data."""

import re
from pathlib import Path

import numpy as np
import pandas as pd

BODY_OBJECT_NAME = "asterix_front"
BOGIE_OBJECT_NAME = "asterix_back"


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

def offset_vicon_data_time(df: pd.DataFrame, time_offset: float) -> pd.DataFrame:
    df["time"] = df["time"] + time_offset
    return df


def update_body_names(df: pd.DataFrame):
    global BODY_OBJECT_NAME, BOGIE_OBJECT_NAME

    body_pattern = re.compile("asterix_front.*_x")
    bogie_pattern = re.compile("asterix_back.*_x")

    BODY_OBJECT_NAME = list(filter(body_pattern.match, df.columns))[0][:-2]
    BOGIE_OBJECT_NAME = list(filter(bogie_pattern.match, df.columns))[0][:-2]
