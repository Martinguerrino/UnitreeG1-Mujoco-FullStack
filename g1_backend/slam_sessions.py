"""Restore RTAB-Map sessions without modifying the source database."""
import json
from pathlib import Path
import sqlite3


def restore_database(source, destination, backend):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination or destination.exists():
        raise ValueError('Localization requires a new slam_output directory, separate from the saved map')
    manifest = source.parent / 'slam_session.json'
    if manifest.exists() and json.loads(manifest.read_text())['backend'] != backend:
        raise ValueError('Saved map backend does not match --slam')
    with sqlite3.connect(source.as_uri() + '?mode=ro', uri=True) as saved:
        tables = {row[0] for row in saved.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if not {'Node', 'Data', 'Statistics', 'Admin'}.issubset(tables):
            raise ValueError('Incomplete RTAB-Map schema')
        if saved.execute('SELECT COUNT(*) FROM Node').fetchone()[0] == 0:
            raise ValueError('Saved map contains no nodes')
        with sqlite3.connect(destination) as session:
            saved.backup(session)
