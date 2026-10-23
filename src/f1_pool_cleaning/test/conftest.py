import pytest

from f1_pool_cleaning.perception_config import PerceptionConfig


@pytest.fixture
def calibrated():
    # Synthetic dimensions/response for tests, not robot measurements.
    return PerceptionConfig(geometry_confirmed=True, hull_length_m=.8, hull_width_m=.6,
                  brush_tip_y_m=.5, brush_compression_m=.02, brush_max_compression_m=.04,
                  stopping_distance_m=.25, front_slow_distance_m=.8).validate()
