"""Release navigation UDP ports held by this workspace's previous processes."""

import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PORTS = (19420, 19421, 19422, 19423)
SCRIPTS = {ROOT / 'scripts/navigation_bridge.py', ROOT / 'scripts/navigation_sim.py'}


def port_is_free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(('127.0.0.1', port))
        except OSError as error:
            if error.errno == 98:
                return False
            raise
    return True


def navigation_owner(pid):
    proc = Path('/proc') / str(pid)
    try:
        if proc.stat().st_uid != os.getuid():
            return False
        args = proc.joinpath('cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        if len(args) < 2 or not Path(os.fsdecode(args[0])).name.startswith('python'):
            return False
        # Spawned sensor workers have no script path in argv. Require the exact
        # multiprocessing entry point and this workspace before closing them.
        if len(args) == 4 and args[1] == b'-c' and args[3] == b'--multiprocessing-fork':
            return (
                re.fullmatch(
                    rb'from multiprocessing.spawn import spawn_main; '
                    rb'spawn_main\(tracker_fd=\d+, pipe_handle=\d+\)', args[2]
                ) is not None
                and proc.joinpath('cwd').resolve() == ROOT
            )
        script = Path(os.fsdecode(args[1]))
        if not script.is_absolute():
            script = proc / 'cwd' / script
        return script.resolve() in SCRIPTS
    except (FileNotFoundError, PermissionError):
        return False


def recover_ports():
    busy = [port for port in PORTS if not port_is_free(port)]
    if not busy:
        return
    owners = set()
    for port in busy:
        result = subprocess.run(['fuser', '-n', 'udp', str(port)],
                                capture_output=True, text=True, check=False)
        pids = {int(word) for word in result.stdout.split() if word.isdecimal()}
        if not pids or any(not navigation_owner(pid) for pid in pids):
            raise RuntimeError(f'Puerto {port} ocupado por otro proceso; no se cerrará automáticamente')
        owners.update(pids)
    print(f'Cerrando navegación anterior que ocupa UDP {busy} (PID: {sorted(owners)})...', flush=True)
    for sig, timeout in ((signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        for pid in owners:
            # Recheck identity before signaling; processes may have already exited.
            if navigation_owner(pid):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(port_is_free(port) for port in PORTS):
                print('Puertos de navegación liberados.', flush=True)
                return
            time.sleep(.1)
    raise RuntimeError('No se pudieron liberar los puertos de navegación')


if __name__ == '__main__':
    try:
        recover_ports()
    except (OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
