import math
import sys
from pathlib import Path

import analysis
import matplotlib.pyplot as plt
import telemetry
import vicon
from rich import print
from rich.console import Console
from rich.table import Table

# Default to using this example data if no arguments are provided
FALLBACK_ON_EXAMPLES = True
REPO_ROOT = Path(__file__).parent.parent
VICON_EXAMPLE_PATH = REPO_ROOT / "logs" / "8-5-bezierway-heading" / "8-5-bezierway-heading-2.dat"
TELEM_EXAMPLE_PATH = REPO_ROOT / "logs" / "8-5-bezierway-heading" / "8-5-bezierway-heading-2.csv"

# offset vicon time in 8-5 data due to improper epoch synching
eight_five_time_offset = 1691279982.08448 - 1691280146.5914912

if __name__ == "__main__":
    try:
        if len(sys.argv) != 3:
            raise ValueError(
                "Two file path arguments should be provided: vicon and telemetry data files."
            )

        vicon_path = Path(sys.argv[1])
        telem_path = Path(sys.argv[2])

    except ValueError as err:
        if FALLBACK_ON_EXAMPLES:
            vicon_path = VICON_EXAMPLE_PATH
            telem_path = TELEM_EXAMPLE_PATH
        else:
            raise err

    vicon_df = vicon.read_motion(vicon_path)
    vicon_df = vicon.offset_vicon_data_time(vicon_df, eight_five_time_offset)
    telem_df = telemetry.read_telem(telem_path)
    merged_df = analysis.merge_vicon_telemetry(telem_df, vicon_df)
    # transformed_df = analysis.transform_origin(merged_df)
    transformed_df = analysis.flatten_along_x(merged_df)

    analysis.plot_motion(merged_df, "Original Data", show_arrows=True)
    analysis.plot_motion_2d(merged_df, "Original Data")

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
