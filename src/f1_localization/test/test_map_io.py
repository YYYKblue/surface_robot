import pytest

from f1_localization.map_io import MapProfileError, load_occupancy, resolve_map_files, resolve_pbstream
from f1_localization.localization import bounds_overrides


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
