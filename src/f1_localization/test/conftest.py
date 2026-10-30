import pytest


@pytest.fixture
def reference_map(tmp_path):
    """Small self-contained raster; the state is only a launch-path placeholder."""
    pixels = bytes(254 if 5 <= row < 35 and 5 <= col < 35 else 0
                   for row in range(40) for col in range(40))
    (tmp_path / 'reference.pgm').write_bytes(b'P5\n40 40\n255\n' + pixels)
    path = tmp_path / 'reference.yaml'
    path.write_text('image: reference.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    path.with_suffix('.pbstream').write_bytes(b'path fixture, not Cartographer state')
    return path
