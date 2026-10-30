import numpy as np
import pytest

from f1_localization.map_io import MapProfileError, Occupancy, load_occupancy, resolve_map_files, resolve_pbstream
from f1_localization.localization import bounds_overrides


CELL_LABELS = np.array([[1, 0, 2, 1],
                        [0, 2, 1, 0],
                        [2, 1, 0, 2]], dtype=np.uint8)
CELL_NAMES = ('free', 'occupied', 'unknown')


def labeled_map(resolution=.25, origin=(-1.5, .75)):
    free = (CELL_LABELS == 0).astype(np.uint8)
    occupied = (CELL_LABELS == 1).astype(np.uint8)
    return Occupancy(resolution, *origin, 4, 3, free, occupied, free)


@pytest.mark.parametrize('resolution', [1., .05])
@pytest.mark.parametrize('origin', [(0., 0.), (-1.312974, -1.097704)])
def test_cell_centres_round_trip_including_top_and_bottom_rows(resolution, origin):
    occupancy = labeled_map(resolution, origin)
    for row, labels in enumerate(CELL_LABELS):
        for column, label in enumerate(labels):
            point = occupancy.xy(column, row)
            assert occupancy.cell(*point) == CELL_NAMES[label], (column, row, point)


@pytest.mark.parametrize('point, expected', [
    ((-1.5, .75), 'unknown'),                    # bottom-left corner included
    ((-1.5, 1.5 - 1e-8), 'occupied'),            # top-left cell
    ((-.5 - 1e-8, .75), 'unknown'),              # bottom-right cell
    ((-.5 - 1e-8, 1.5 - 1e-8), 'occupied'),      # top-right cell
    ((-1.5 - 1e-8, 1.), None),                  # outside left
    ((-.5, 1.), None),                          # upper x bound excluded
    ((-1., .75 - 1e-8), None),                  # outside bottom
    ((-1., 1.5), None),                         # upper y bound excluded
    ((-1.125, 1.), 'unknown'),                  # internal horizontal boundary
    ((-1.125, 1. - 1e-8), 'occupied'),
    ((-1., 1.125), 'occupied'),                 # internal vertical boundary
    ((-1. - 1e-8, 1.125), 'unknown'),
])
def test_cell_classifies_boundaries_and_rejects_outside_points(point, expected):
    assert labeled_map().cell(*point) == expected


def test_yaml_and_pbstream_resolve_the_same_raster_and_bounds(reference_map):
    state = reference_map.with_suffix('.pbstream')
    assert resolve_pbstream(reference_map) == state
    assert resolve_map_files(state)[0] == reference_map
    assert bounds_overrides(state) == bounds_overrides(reference_map)
    assert bounds_overrides(state) == pytest.approx(dict(
        origin_x_m=.55, origin_y_m=.55, axis_yaw_rad=0., length_m=2.9, width_m=2.9))
    occupancy = load_occupancy(state)
    assert occupancy.cell(1., 1.) == 'free'
    assert occupancy.cell(.1, .1) == 'occupied'
    assert occupancy.cell(-1., -1.) is None


def test_missing_raster_requires_export_and_missing_state_is_rejected(tmp_path, reference_map):
    state = tmp_path / 'missing.pbstream'
    state.write_bytes(b'placeholder')
    with pytest.raises(MapProfileError, match='pbstream_to_ros_map'):
        resolve_map_files(state)
    reference_map.with_suffix('.pbstream').unlink()
    with pytest.raises(MapProfileError, match='raster map cannot replace'):
        resolve_pbstream(reference_map)


def test_copied_map_uses_sibling_image_and_rejects_unsupported_origin(reference_map):
    content = reference_map.read_text().replace('image: reference.pgm', 'image: /old/nano/reference.pgm')
    reference_map.write_text(content)
    assert load_occupancy(reference_map).region.sum() == 900
    reference_map.write_text(content.replace('origin: [0, 0, 0]', 'origin: [0, 0, 0.2]'))
    with pytest.raises(MapProfileError, match='rotated map origins'):
        load_occupancy(reference_map)
