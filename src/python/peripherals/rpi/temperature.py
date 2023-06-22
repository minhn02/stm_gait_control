import re
import subprocess
from pathlib import Path


def get_cpu_temp() -> float:
    cpu_temp_file = Path("/sys/class/thermal/thermal_zone0/temp")
    cpu_temp_C = int(cpu_temp_file.read_text().rstrip()) / 1000
    return cpu_temp_C


def get_gpu_temp() -> float:
    p1 = subprocess.run(["/opt/vc/bin/vcgencmd", "measure_temp"], capture_output=True, text=True)
    if p1.returncode == 0:
        match = re.search(r"temp=(\d+\.\d+)'C", p1.stdout.strip())
        if match:
            gpu_temp = float(match.group(1))
            return gpu_temp

    raise IOError("Failed to read GPU temperature")
