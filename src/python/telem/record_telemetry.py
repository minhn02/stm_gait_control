from pathlib import Path
from datetime import datetime

log_dir_path = Path(f"/home/pi/minh/logs/{datetime.now().strftime('%Y%m%d')}")

def setup_log_file(filename):
        log_dir_path.mkdir(parents=True, exist_ok=True)
        log_path = log_dir_path / filename
        headers = [
            "time",
        ]

        for i in range(4):
            headers.extend(
                [
                    f"wheel{i+1}_vel",
                    f"wheel{i+1}_cur",
                    f"wheel{i+1}_vol",
                    f"wheel{i+1}_pos",
                    f"wheel{i+1}_temp",
                ]
            )

        for loc in ["steer", "bogie"]:
            for axis in ["x", "y", "z"]:
                headers.append(f"hebi_{loc}_acc_{axis}")
            for axis in ["x", "y", "z"]:
                headers.append(f"hebi_{loc}_gyro_{axis}")
            headers.extend(
                [
                    f"hebi_{loc}_pos_desired",
                    f"hebi_{loc}_pos",
                    f"hebi_{loc}_eff",
                    f"hebi_{loc}_cur_motor",
                    f"hebi_{loc}_cur_motor_winding",
                    f"hebi_{loc}_vol",
                    f"hebi_{loc}_temp_board",
                    f"hebi_{loc}_temp_motor_winding",
                ]
            )

        log_path.write_text(f"{','.join(headers)}\n")

def write_telemetry(filename, t, steering_motor, bogie_motor, wheels):
        log_row = [
            f"{t}",
        ]

        for i in range(4):
            log_row.extend(
                [
                    f"{wheels.get_telemetry()[i]['velocity']}",
                    f"{wheels.get_telemetry()[i]['current']}",
                    f"{wheels.get_telemetry()[i]['voltage']}",
                    f"{wheels.get_telemetry()[i]['position']}",
                    f"{wheels.get_telemetry()[i]['temperature']}",
                ]
            )

        for hebi_fbk in [steering_motor.get_feedback(), bogie_motor.get_feedback()]:
            for i in range(3):
                log_row.append(f"{hebi_fbk.accelerometer[0][i]}")
            for i in range(3):
                log_row.append(f"{hebi_fbk.gyro[0][i]}")

            log_row.extend(
                [
                    f"{hebi_fbk.position_command[0]}",
                    f"{hebi_fbk.position[0]}",
                    f"{hebi_fbk.effort[0]}",
                    f"{hebi_fbk.motor_current[0]}",
                    f"{hebi_fbk.motor_winding_current[0]}",
                    f"{hebi_fbk.voltage[0]}",
                    f"{hebi_fbk.board_temperature[0]}",
                    f"{hebi_fbk.motor_winding_temperature[0]}",
                ]
            )

        log_path = log_dir_path / filename
        with log_path.open("a") as f:
            f.write(f"{','.join(log_row)}\n")