import sys
from pathlib import Path

import analysis
import matplotlib.pyplot as plt
import telemetry
import vicon

# Default to using this example data if no arguments are provided
FALLBACK_ON_EXAMPLES = True
REPO_ROOT = Path(__file__).parent.parent
VICON_EXAMPLE_PATH = REPO_ROOT / "logs" / "7-23-naive" / "7-23-naive-1.dat"
TELEM_EXAMPLE_PATH = REPO_ROOT / "logs" / "7-23-naive" / "7-23-naive-1.csv"

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
    telem_df = telemetry.read_telem(telem_path)
    merged_df = analysis.merge_vicon_telemetry(telem_df, vicon_df)
    transformed_df = analysis.transform_origin(merged_df)

    analysis.plot_motion(merged_df, "Original Data", show_arrows=True)
    analysis.plot_motion_2d(merged_df, "Original Data")

    analysis.plot_motion(transformed_df, "Transformed Data", show_arrows=True)
    analysis.plot_motion_2d(transformed_df, "Transformed Data")

    plt.show()
