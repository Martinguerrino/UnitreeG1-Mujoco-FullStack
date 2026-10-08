"""Locate the user's simulation Python without depending on a Conda vendor."""

import argparse
import json
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]


def resolve_python(root=ROOT, environ=None):
    env = os.environ if environ is None else environ
    configured = env.get('G1_PYTHON')
    settings = Path(root) / '.g1-runtime.json'
    if not configured and settings.is_file():
        configured = json.loads(settings.read_text())['python']
    if not configured:
        prefix = env.get('CONDA_PREFIX') or env.get('VIRTUAL_ENV')
        candidates = [Path(prefix) / 'bin/python'] if prefix else []
        candidates.append(Path(root) / '.venv/bin/python')
        configured = next((str(p) for p in candidates if p.is_file()), None)
    if not configured:
        configured = shutil.which('python3') or sys.executable
    path = Path(configured).expanduser().absolute()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(
            f'Python no disponible: {path}. Ejecutá bash setup.sh --python /ruta/python'
        )
    # Do not resolve symlinks: a venv's executable points to the system binary,
    # but must be invoked through the venv path to retain its site-packages.
    return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--save', metavar='PYTHON')
    args = parser.parse_args()
    if args.save:
        path = resolve_python(environ={'G1_PYTHON': args.save})
        (ROOT / '.g1-runtime.json').write_text(json.dumps({'python': path}, indent=2) + '\n')
    else:
        path = resolve_python()
    print(path)


if __name__ == '__main__':
    main()
