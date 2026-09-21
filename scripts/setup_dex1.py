#!/usr/bin/env python3
"""Fetch pinned official Dex1 source assets; verify every download."""

import base64
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    destination = ROOT / 'artifacts/dex1'
    destination.mkdir(parents=True, exist_ok=True)
    for name, entry in json.loads((ROOT / 'configs/dex1.sources.json').read_text()).items():
        path = destination / name
        if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256']:
            continue
        url = (
            f'https://api.github.com/repos/unitreerobotics/{entry["repository"]}/contents/'
            f'{entry["path"]}?ref={entry["revision"]}'
        )
        with urllib.request.urlopen(url, timeout=60) as response:
            payload = json.load(response)
        content = base64.b64decode(payload['content'])
        if hashlib.sha256(content).hexdigest() != entry['sha256']:
            raise ValueError(f'Checksum mismatch: {name}')
        path.write_bytes(content)
        print(path)


if __name__ == '__main__':
    main()
