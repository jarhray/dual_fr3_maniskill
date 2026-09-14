"""Ensure nested performance categories do not double-count GPU operations."""
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def profile_module():
    path = Path(__file__).resolve().parents[1] / 'scripts/profile_trunking_cable.py'
    spec = importlib.util.spec_from_file_location('cable_profile_script', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_nested_components_conserve_wall_time(profile_module, monkeypatch):
    ticks = iter([0., 1., 2., 4., 7., 10.])
    monkeypatch.setattr(profile_module.time, 'perf_counter', lambda: next(ticks))
    timer = profile_module.Timings()
    with timer.measure('other'):
        with timer.measure('MPM'):
            with timer.measure('kernel/p2g', operation=True):
                pass
    report = timer.report()
    assert report['component_seconds'] == {'MPM': 6., 'other': 4.}
    assert sum(report['component_seconds'].values()) == 10.
    assert report['operations']['kernel/p2g']['inclusive_seconds'] == 2.
    assert report['operations']['MPM']['exclusive_seconds'] == 4.


def test_failed_operation_preserves_exception_and_timer_stack(profile_module, monkeypatch):
    ticks = iter([0., 1., 2., 3.])
    monkeypatch.setattr(profile_module.time, 'perf_counter', lambda: next(ticks))
    timer = profile_module.Timings()
    barriers = []
    timer.sync = lambda: barriers.append(True)
    def fail():
        raise ValueError('original numerical error')
    with pytest.raises(ValueError, match='original numerical error'):
        with timer.measure('MPM'):
            timer.wrapper(fail, 'kernel/failure', operation=True, barrier=True)()
    assert not timer.frames
    assert len(barriers) == 2
    assert sum(timer.components.values()) == 3.


def test_graph_recording_is_not_counted_as_kernel_execution(profile_module):
    timer = profile_module.Timings()
    timer.capture_active = lambda: True
    calls = []
    timer.sync = lambda: pytest.fail('Synchronization inside CUDA graph capture')
    wrapped = timer.wrapper(lambda: calls.append('recorded'), 'kernel/example',
                            operation=True, barrier=True)
    wrapped()
    assert calls == ['recorded']
    assert not timer.operations and not timer.frames
