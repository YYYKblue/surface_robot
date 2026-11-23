"""Read map-specific GNSS coordinates from Cartographer's optimized pose graph."""
import math
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from f1_mapping_bringup.gnss_landmarks import ORIGIN_PREFIX, Origin, PlanarPose


def read_reference(path, trajectory):
    executable = Path(get_package_prefix("f1_localization")) / "lib/f1_localization/read_gnss_reference"
    result = subprocess.run([str(executable), str(path), str(trajectory), ORIGIN_PREFIX],
                            capture_output=True, text=True, timeout=15, check=True)
    rows = [line.split() for line in result.stdout.splitlines() if line.strip()]
    if not rows or rows[0][0] != "reference" or len(rows) > 2:
        raise ValueError("missing trajectory or ambiguous GNSS landmark")
    if len(rows) == 1:
        return None

    def pose(row):
        values = [float(v) for v in row[1:]]
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError("invalid map landmark pose")
        return PlanarPose(*values)

    return Origin.from_id(rows[1][0]), pose(rows[1]), pose(rows[0])
