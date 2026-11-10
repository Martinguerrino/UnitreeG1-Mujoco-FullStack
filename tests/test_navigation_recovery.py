"""Only old navigation processes from this workspace may be stopped."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location(
    'navigation_recovery', Path(__file__).resolve().parents[1] / 'scripts/recover_navigation_ports.py'
)
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


def test_free_ports_do_not_signal_any_process(monkeypatch):
    monkeypatch.setattr(recovery, 'port_is_free', lambda port: True)
    kill = Mock()
    monkeypatch.setattr(recovery.os, 'kill', kill)
    recovery.recover_ports()
    kill.assert_not_called()


def test_unrelated_port_owner_is_preserved(monkeypatch):
    monkeypatch.setattr(recovery, 'port_is_free', lambda port: port != 19420)
    monkeypatch.setattr(recovery.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(stdout='123'))
    monkeypatch.setattr(recovery, 'navigation_owner', lambda pid: False)
    kill = Mock()
    monkeypatch.setattr(recovery.os, 'kill', kill)
    with pytest.raises(RuntimeError, match='otro proceso'):
        recovery.recover_ports()
    kill.assert_not_called()


def test_verified_navigation_owner_is_closed(monkeypatch):
    busy = {19420, 19422, 19423}
    monkeypatch.setattr(recovery, 'port_is_free', lambda port: port not in busy)
    monkeypatch.setattr(recovery.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(stdout='123'))
    monkeypatch.setattr(recovery, 'navigation_owner', lambda pid: pid == 123)
    kill = Mock(side_effect=lambda pid, sig: busy.clear())
    monkeypatch.setattr(recovery.os, 'kill', kill)
    recovery.recover_ports()
    kill.assert_called_once_with(123, recovery.signal.SIGTERM)


def test_camera_port_owner_is_force_closed_when_term_does_not_release_it(monkeypatch):
    busy = {19423}
    monkeypatch.setattr(recovery, 'port_is_free', lambda port: port not in busy)
    monkeypatch.setattr(recovery.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(stdout='123'))
    monkeypatch.setattr(recovery, 'navigation_owner', lambda pid: pid == 123)
    monkeypatch.setattr(recovery.time, 'monotonic', Mock(side_effect=[0, 4, 4, 4]))

    def signal_owner(pid, sig):
        if sig == recovery.signal.SIGKILL:
            busy.clear()

    kill = Mock(side_effect=signal_owner)
    monkeypatch.setattr(recovery.os, 'kill', kill)
    recovery.recover_ports()
    assert [call.args for call in kill.call_args_list] == [
        (123, recovery.signal.SIGTERM), (123, recovery.signal.SIGKILL)
    ]


@pytest.mark.parametrize('workspace, expected', [(True, True), (False, False)])
def test_spawned_worker_must_belong_to_workspace(tmp_path, monkeypatch, workspace, expected):
    proc = tmp_path / '123'
    proc.mkdir()
    proc.joinpath('cmdline').write_bytes(
        b'/usr/bin/python3\0-c\0from multiprocessing.spawn import spawn_main; '
        b'spawn_main(tracker_fd=61, pipe_handle=66)\0--multiprocessing-fork\0'
    )
    proc.joinpath('cwd').symlink_to(recovery.ROOT if workspace else tmp_path)
    monkeypatch.setattr(recovery, 'Path', lambda value: tmp_path if value == '/proc' else Path(value))
    assert recovery.navigation_owner(123) is expected


def test_shutdown_ignores_repeated_signals_during_cleanup(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'navigation_sim', recovery.ROOT / 'scripts/navigation_sim.py'
    )
    simulator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(simulator)
    handler = Mock()
    monkeypatch.setattr(simulator.signal, 'signal', handler)
    with pytest.raises(KeyboardInterrupt):
        simulator.request_shutdown(simulator.signal.SIGINT, None)
    assert handler.call_count == 2
    handler.assert_any_call(simulator.signal.SIGINT, simulator.signal.SIG_IGN)
    handler.assert_any_call(simulator.signal.SIGTERM, simulator.signal.SIG_IGN)
