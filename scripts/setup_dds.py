"""Fetch pinned runtime artifacts and install optional dependencies locally.

Run with the existing mujoco-unitree Python. No ROS or ACT upgrades are made.
"""

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def fetch(destination, entry):
    if (
        destination.exists()
        and hashlib.sha256(destination.read_bytes()).hexdigest() == entry['sha256']
    ):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    if entry['path']:
        url = f'https://raw.githubusercontent.com/{entry["repo"]}/{entry["revision"]}/{entry["path"]}'
    else:
        url = f'https://api.github.com/repos/{entry["repo"]}/tarball/{entry["revision"]}'
    print(f'Downloading {destination.relative_to(ROOT)}', flush=True)
    try:
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
    except urllib.error.HTTPError:
        if not entry.get('blob_url'):
            raise
        with urllib.request.urlopen(entry['blob_url'], timeout=60) as response:
            data = base64.b64decode(json.load(response)['content'])
    if hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise ValueError(f'SHA-256 mismatch for {destination}')
    temporary = destination.with_suffix(destination.suffix + '.partial')
    temporary.write_bytes(data)
    temporary.replace(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download-only', action='store_true')
    args = parser.parse_args()
    lock = json.loads((ROOT / 'configs/sources.lock.json').read_text())
    for name, entry in lock['files'].items():
        fetch(ROOT / name, entry)
    if args.download_only:
        return
    native_revision = lock['files']['artifacts/dependencies/cyclonedds.tar.gz']['revision']
    source_dir = ROOT / f'artifacts/dependencies/cyclonedds-source-{native_revision[:8]}'
    source_dir.mkdir(exist_ok=True)
    with tarfile.open(ROOT / 'artifacts/dependencies/cyclonedds.tar.gz') as archive:
        archive.extractall(source_dir, filter='data')
    source = next(source_dir.glob('*/CMakeLists.txt')).parent
    build = ROOT / f'artifacts/dependencies/cyclonedds-build-{native_revision[:8]}'
    prefix = ROOT / 'artifacts/cyclonedds'
    subprocess.run(
        [
            'cmake',
            '-S',
            str(source),
            '-B',
            str(build),
            f'-DCMAKE_INSTALL_PREFIX={prefix}',
            '-DBUILD_EXAMPLES=OFF',
            '-DBUILD_TESTING=OFF',
            '-DENABLE_SECURITY=OFF',
        ],
        check=True,
    )
    subprocess.run(['cmake', '--build', str(build), '-j', '4'], check=True)
    subprocess.run(['cmake', '--install', str(build)], check=True)
    environment = dict(os.environ, CYCLONEDDS_HOME=str(prefix))
    target = ROOT / 'artifacts/dds_runtime'
    subprocess.run(
        [
            sys.executable,
            '-m',
            'pip',
            'install',
            '--upgrade',
            '--target',
            str(target),
            '--no-deps',
            '--no-cache-dir',
            '--no-binary=cyclonedds',
            str(ROOT / 'artifacts/dependencies/sdk2.tar.gz'),
            '-r',
            str(ROOT / 'requirements-dds.txt'),
        ],
        check=True,
        env=environment,
    )
    # SDK2 setup.py omits package_data for its original CRC shared libraries.
    with tarfile.open(ROOT / 'artifacts/dependencies/sdk2.tar.gz') as archive:
        for member in archive.getmembers():
            suffix = member.name.split('/', 1)[-1]
            if (
                suffix
                in (
                    'unitree_sdk2py/utils/lib/crc_amd64.so',
                    'unitree_sdk2py/utils/lib/crc_aarch64.so',
                )
                and member.isfile()
            ):
                destination = target / suffix
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(archive.extractfile(member).read())
    print('DDS runtime installed under artifacts/dds_runtime')


if __name__ == '__main__':
    main()
