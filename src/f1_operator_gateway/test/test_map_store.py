from pathlib import Path
import json
import re

import cv2
import numpy as np
import pytest
import yaml

from operator_gateway.map_store import MapStore


def triplet(directory, name):
    directory = Path(directory)
    (directory / (name + '.pbstream')).write_bytes(b'fixture-state')
    cv2.imwrite(str(directory / (name + '.pgm')), np.array([[0, 255], [205, 255]], dtype=np.uint8))
    (directory / (name + '.yaml')).write_text(yaml.safe_dump(dict(
        image=str(directory / (name + '.pgm')), resolution=.05, origin=[1., 2., 0.],
        negate=0, occupied_thresh=.65, free_thresh=.196)))


@pytest.mark.parametrize('name', ['../escape', '/tmp/map', 'a/b', 'a\\b', '..', 'a.yaml', 'CON', 'bad:name', 'x'*81])
def test_name_rejects_paths_and_windows_reserved_names(name):
    with pytest.raises(ValueError):
        MapStore.name(name)


def test_publication_timestamp_collision_relative_yaml_and_preview(tmp_path):
    store = MapStore(tmp_path)
    first = store.reserve(' 一号池 ')
    triplet(first['directory'], first['name'])
    saved = store.publish(first)
    assert saved['name'] == '一号池'
    assert all(Path(path).is_file() for path in saved['paths'])
    assert yaml.safe_load(store.path('一号池', '.yaml').read_text())['image'] == '一号池.pgm'
    assert store.reserve('一号池')['name'] == '一号池_01'
    assert re.fullmatch(r'\d{8}_\d{6}', store.reserve('  ')['name'])
    entry = store.catalog()[0]
    assert entry['name'] == '一号池' and entry['valid']
    preview = store.preview('一号池')
    assert preview['data'] == [-1, 0, 100, 0]
    assert preview['origin'] == dict(x=1., y=2., yaw=0.)


def test_incomplete_snapshot_recovery_and_existing_map_never_overwritten(tmp_path):
    store = MapStore(tmp_path)
    transaction = store.reserve('恢复')
    Path(transaction['stem'] + '.pbstream').write_bytes(b'saved-before-export')
    assert not store.catalog()  # Hidden until export succeeds.
    reopened = MapStore(tmp_path)
    assert reopened.recovered() == [transaction]
    triplet(transaction['directory'], transaction['name'])
    store.path('恢复', '.pgm').write_bytes(b'existing')
    with pytest.raises(ValueError, match='未覆盖'):
        reopened.publish(transaction)
    assert store.path('恢复', '.pgm').read_bytes() == b'existing'
    assert not store.catalog()[0]['valid']
    assert Path(transaction['stem'] + '.pbstream').exists()


def test_symlinks_cannot_escape_maps(tmp_path):
    store = MapStore(tmp_path)
    external = tmp_path / 'external'
    external.write_bytes(b'not-a-map')
    (store.root / 'outside.pbstream').symlink_to(external)
    assert not store.catalog()[0]['valid']
    with pytest.raises(ValueError):
        store.validate('outside')
    other = tmp_path / 'other'
    other.mkdir()
    (other / 'maps').symlink_to(store.root)
    with pytest.raises(ValueError):
        MapStore(other)


def test_bad_yaml_and_missing_image_are_reported_per_map(tmp_path):
    store = MapStore(tmp_path)
    triplet(store.root, 'good')
    triplet(store.root, 'bad')
    store.path('bad', '.yaml').write_text('image: ../../private.pgm')
    entries = {e['name']: e for e in store.catalog()}
    assert entries['good']['valid']
    assert not entries['bad']['valid'] and entries['bad']['reason']
