"""Portable toolkit discovery and early reporting of incomplete cable installs."""
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from dual_fr3_maniskill.warp_setup import (
    find_cuda_toolkit, patch_cuda_codegen, patch_driver_soname,
)
from dual_fr3_maniskill import warp_setup


def toolkit(root):
    compiler = root / "bin/nvcc"
    compiler.parent.mkdir(parents=True)
    compiler.write_text("#!/bin/sh\nexit 0\n")
    compiler.chmod(0o755)
    return root


@pytest.fixture
def clean_cuda_env(monkeypatch):
    for name in ("MANISKILL_CUDA_PATH", "CUDA_HOME", "CUDA_PATH"):
        monkeypatch.delenv(name, raising=False)


def test_explicit_toolkit_overrides_environment(tmp_path, monkeypatch, clean_cuda_env):
    desired = toolkit(tmp_path / "desired toolkit")
    monkeypatch.setenv("MANISKILL_CUDA_PATH", str(tmp_path / "missing"))
    assert find_cuda_toolkit(str(desired)) == desired


def test_invalid_explicit_environment_does_not_silently_switch_toolkits(tmp_path, monkeypatch, clean_cuda_env):
    monkeypatch.setenv("MANISKILL_CUDA_PATH", str(tmp_path / "missing"))
    monkeypatch.setenv("CUDA_HOME", str(toolkit(tmp_path / "other")))
    with pytest.raises(RuntimeError, match="CUDA compiler not found"):
        find_cuda_toolkit()


@pytest.mark.parametrize("variable", ["MANISKILL_CUDA_PATH", "CUDA_HOME", "CUDA_PATH"])
def test_selected_environment_toolkit(tmp_path, monkeypatch, clean_cuda_env, variable):
    desired = toolkit(tmp_path / "custom")
    monkeypatch.setenv(variable, str(desired))
    assert find_cuda_toolkit() == desired


def test_path_compiler_symlink_resolves_to_toolkit(tmp_path, monkeypatch, clean_cuda_env):
    desired = toolkit(tmp_path / "toolkit")
    executables = tmp_path / "executables"
    executables.mkdir()
    (executables / "nvcc").symlink_to(desired / "bin/nvcc")
    monkeypatch.setenv("PATH", str(executables))
    assert find_cuda_toolkit() == desired


@pytest.mark.parametrize("patcher,original,fixed", [
    (patch_driver_soname, 'dlopen("libcuda.so", RTLD_NOW)', 'dlopen("libcuda.so.1", RTLD_NOW)'),
    (patch_cuda_codegen, 'cu_source += warp.codegen.codegen_module(kernel, device="cuda")',
     '# CUDA kernels are launched directly through cuLaunchKernel, not host wrappers.'),
])
def test_fork_patches_are_repeatable_and_preserve_surrounding_source(tmp_path, patcher, original, fixed):
    source = tmp_path / "source"
    source.write_text("before\n" + original + "\nafter\n")
    assert patcher(source)
    assert source.read_text() == "before\n" + fixed + "\nafter\n"
    assert not patcher(source)


def test_mtc_checks_missing_warp_before_scene_or_robot_initialization(tmp_path, monkeypatch):
    pytest.importorskip("sapien")
    pytest.importorskip("mani_skill2")
    from dual_fr3_maniskill.cable import mpm_cable
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableEnv

    monkeypatch.setattr(mpm_cable.wp, "__file__", str(tmp_path / "warp/__init__.py"))
    # No assets or cable configuration are needed to diagnose the missing library.
    with pytest.raises(RuntimeError, match="Warp library is missing.*warp_setup"):
        TrunkingCableEnv(None, cable_config={})


@pytest.fixture
def installed_fork(tmp_path, monkeypatch):
    fork = tmp_path / "warp_maniskill"
    source = fork / "warp/native/warp.cu"
    context = fork / "warp/context.py"
    library = fork / "warp/bin/warp.so"
    source.parent.mkdir(parents=True)
    library.parent.mkdir()
    (fork / "__init__.py").write_text("")
    source.write_text('driver = dlopen("libcuda.so", RTLD_NOW);\n')
    context.write_text('cu_source += warp.codegen.codegen_module(kernel, device="cuda")\n')
    library.write_bytes(b"previous working binary")
    monkeypatch.setattr(warp_setup.importlib.metadata, "version", lambda _: "0.5.3")
    monkeypatch.setattr(warp_setup.importlib.util, "find_spec",
                        lambda name: SimpleNamespace(origin=str(tmp_path / name / "__init__.py")))
    monkeypatch.setattr(warp_setup, "find_cuda_toolkit", lambda _: tmp_path / "cuda")
    monkeypatch.setattr(warp_setup, "library_loads", lambda path: path.is_file())
    return source, context, library


def test_unsupported_version_is_rejected_before_patching(installed_fork, monkeypatch):
    before = [path.read_bytes() for path in installed_fork]
    monkeypatch.setattr(warp_setup.importlib.metadata, "version", lambda _: "0.6.0")
    with pytest.raises(RuntimeError, match="require ManiSkill2 0.5.3"):
        warp_setup.build_warp()
    assert [path.read_bytes() for path in installed_fork] == before


def test_foreign_warp_is_rejected_before_patching(installed_fork, monkeypatch, tmp_path):
    before = [path.read_bytes() for path in installed_fork]
    monkeypatch.setattr(warp_setup.importlib.util, "find_spec", lambda name: SimpleNamespace(
        origin=str(tmp_path / ("foreign" if name == "warp_maniskill" else "") / name / "__init__.py")))
    with pytest.raises(RuntimeError, match="different installation"):
        warp_setup.build_warp()
    assert [path.read_bytes() for path in installed_fork] == before


def test_unrecognized_second_patch_preserves_first_source(installed_fork):
    installed_fork[1].write_text("unsupported code generator")
    before = [path.read_bytes() for path in installed_fork]
    with pytest.raises(RuntimeError, match="Unrecognized Warp CUDA code generator"):
        warp_setup.build_warp()
    assert [path.read_bytes() for path in installed_fork] == before


def test_failed_build_preserves_working_installation(installed_fork, monkeypatch, tmp_path):
    before = [path.read_bytes() for path in installed_fork]
    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(1, args[0])
    monkeypatch.setattr(warp_setup.subprocess, "run", fail)
    with pytest.raises(subprocess.CalledProcessError):
        warp_setup.build_warp(force=True)
    assert [path.read_bytes() for path in installed_fork] == before
    assert not list(tmp_path.glob(".maniskill-warp-build-*"))


def fake_compile(*args, **kwargs):
    stage = kwargs["cwd"] / "warp_maniskill/warp"
    assert 'dlopen("libcuda.so.1", RTLD_NOW)' in (stage / "native/warp.cu").read_text()
    (stage / "bin").mkdir()
    (stage / "bin/warp.so").write_bytes(b"new binary")


def test_unloadable_build_is_not_published(installed_fork, monkeypatch):
    before = [path.read_bytes() for path in installed_fork]
    monkeypatch.setattr(warp_setup.subprocess, "run", fake_compile)
    monkeypatch.setattr(warp_setup, "library_loads", lambda path: path == installed_fork[2])
    with pytest.raises(RuntimeError, match="cannot be loaded"):
        warp_setup.build_warp(force=True)
    assert [path.read_bytes() for path in installed_fork] == before


def test_failed_publish_restores_source_patches(installed_fork, monkeypatch):
    before = [path.read_bytes() for path in installed_fork]
    replace = warp_setup.os.replace
    def fail_library_publish(candidate, destination):
        if destination == installed_fork[2]:
            raise OSError("simulated publish failure")
        replace(candidate, destination)
    monkeypatch.setattr(warp_setup.subprocess, "run", fake_compile)
    monkeypatch.setattr(warp_setup.os, "replace", fail_library_publish)
    with pytest.raises(OSError, match="simulated publish failure"):
        warp_setup.build_warp(force=True)
    assert [path.read_bytes() for path in installed_fork] == before


def test_successful_build_can_be_repeated_without_recompilation(installed_fork, monkeypatch):
    calls = []
    def compile_once(*args, **kwargs):
        calls.append(True)
        fake_compile(*args, **kwargs)
    monkeypatch.setattr(warp_setup.subprocess, "run", compile_once)
    assert warp_setup.build_warp() == installed_fork[2]
    assert installed_fork[2].read_bytes() == b"new binary"
    warp_setup.build_warp()
    assert len(calls) == 1
