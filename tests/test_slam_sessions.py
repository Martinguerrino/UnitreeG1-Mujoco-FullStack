import hashlib
import json
import sqlite3

import pytest

from g1_backend.slam_sessions import restore_database


def saved_map(tmp_path):
    source = tmp_path / 'saved.db'
    with sqlite3.connect(source) as connection:
        for table in ('Node', 'Data', 'Statistics', 'Admin'):
            connection.execute(f'CREATE TABLE {table} (id INTEGER)')
        connection.execute('INSERT INTO Node VALUES (1)')
    (tmp_path / 'slam_session.json').write_text(json.dumps({'backend': 'lidar3d'}))
    return source


def test_restore_preserves_source_and_isolates_session(tmp_path):
    source = saved_map(tmp_path)
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    restored = tmp_path / 'restored.db'
    restore_database(source, restored, 'lidar3d')
    with sqlite3.connect(restored) as connection:
        assert connection.execute('SELECT id FROM Node').fetchall() == [(1,)]
        connection.execute('INSERT INTO Node VALUES (2)')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original
    with pytest.raises(ValueError):
        restore_database(source, restored, 'lidar3d')


def test_restore_rejects_wrong_sensor_or_empty_map(tmp_path):
    source = saved_map(tmp_path)
    output = tmp_path / 'restored.db'
    with pytest.raises(ValueError, match='backend'):
        restore_database(source, output, 'rgb-d')
    assert not output.exists()
    with sqlite3.connect(source) as connection:
        connection.execute('DELETE FROM Node')
    with pytest.raises(ValueError, match='no nodes'):
        restore_database(source, output, 'lidar3d')
    assert not output.exists()
