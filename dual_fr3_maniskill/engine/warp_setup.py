"""Build the Warp fork bundled with the active ManiSkill2 installation."""
import argparse
import importlib.metadata
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def find_cuda_toolkit(requested=None):
    selected = requested or next((os.environ[name] for name in
        ("MANISKILL_CUDA_PATH", "CUDA_HOME", "CUDA_PATH") if os.environ.get(name)), None)
    if selected:
        candidates = [Path(selected).expanduser()]
    else:
        nvcc = shutil.which("nvcc")
        candidates = ([Path(nvcc).resolve().parent.parent] if nvcc else [])
        candidates += [Path("/usr/local/cuda"), Path("/usr/local/cuda-11.8")]
    for path in candidates:
        if os.access(path / "bin/nvcc", os.X_OK):
            return path.resolve()
    raise RuntimeError("CUDA compiler not found. Set MANISKILL_CUDA_PATH or pass "
                       "--cuda-path to a toolkit containing bin/nvcc. "
                       f"Checked: {', '.join(map(str, candidates))}")


def patch_driver_soname(source):
    """Use the Linux runtime SONAME; the unversioned development link is optional."""
    text = source.read_text()
    old = 'dlopen("libcuda.so", RTLD_NOW)'
    new = 'dlopen("libcuda.so.1", RTLD_NOW)'
    if old in text:
        source.write_text(text.replace(old, new))
        return True
    if new not in text:
        raise RuntimeError(f"Unrecognized Warp CUDA loader in {source}; expected ManiSkill2's fork")
    return False


def patch_cuda_codegen(source):
    """Omit unused host launch wrappers from NVRTC's device-only compilation."""
    text = source.read_text()
    old = 'cu_source += warp.codegen.codegen_module(kernel, device="cuda")'
    new = '# CUDA kernels are launched directly through cuLaunchKernel, not host wrappers.'
    if old in text:
        source.write_text(text.replace(old, new))
        return True
    if new not in text:
        raise RuntimeError(f"Unrecognized Warp CUDA code generator in {source}")
    return False


def build_warp(cuda_path=None, *, force=False):
    version = importlib.metadata.version("mani-skill2")
    if version != "0.5.3":
        raise RuntimeError(f"Warp compatibility patches require ManiSkill2 0.5.3, found {version}")
    mani_spec = importlib.util.find_spec("mani_skill2")
    spec = importlib.util.find_spec("warp_maniskill")
    if spec is None or spec.origin is None or mani_spec is None or mani_spec.origin is None:
        raise RuntimeError("Install the pinned ManiSkill2 requirements before building its Warp fork")
    fork = Path(spec.origin).resolve().parent
    expected = Path(mani_spec.origin).resolve().parent.parent / "warp_maniskill"
    if fork != expected.resolve():
        raise RuntimeError(f"Warp is from a different installation: {fork}; expected {expected}")
    source = fork / "warp/native/warp.cu"
    context = fork / "warp/context.py"
    library = fork / "warp/bin/warp.so"
    toolkit = find_cuda_toolkit(cuda_path)

    # Validate both patches and compile outside the active package. A failed
    # compiler or missing shared dependency must leave the working files intact.
    with tempfile.TemporaryDirectory(prefix=".maniskill-warp-build-", dir=fork.parent) as temporary:
        stage = Path(temporary) / "warp_maniskill"
        shutil.copytree(fork, stage, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.o", "bin"))
        staged_source = stage / "warp/native/warp.cu"
        staged_context = stage / "warp/context.py"
        native_changed = patch_driver_soname(staged_source)
        context_changed = patch_cuda_codegen(staged_context)
        needs_build = (force or native_changed or not library.is_file() or
                       library.stat().st_mtime < source.stat().st_mtime or
                       not library_loads(library))
        staged_library = stage / "warp/bin/warp.so"
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(filter(None, (
            str(stage.parent), str(stage), env.get("PYTHONPATH"))))
        if needs_build:
            print(f"Building ManiSkill2 Warp with CUDA toolkit {toolkit}", flush=True)
            subprocess.run([sys.executable, "-m", "warp_maniskill.build_lib",
                            "--cuda_path", str(toolkit)], env=env, cwd=stage.parent, check=True)
            if not library_loads(staged_library):
                raise RuntimeError(f"Built Warp library cannot be loaded: {staged_library}")

        replacements = []
        if native_changed:
            replacements.append((staged_source, source))
        if context_changed:
            replacements.append((staged_context, context))
        if needs_build:
            replacements.append((staged_library, library))
        backups = {}
        for index, (_, destination) in enumerate(replacements):
            backup = Path(temporary) / f"backup-{index}"
            if destination.exists():
                shutil.copy2(destination, backup)
                backups[destination] = backup
            else:
                backups[destination] = None
        published = []
        try:
            for candidate, destination in replacements:
                destination.parent.mkdir(parents=True, exist_ok=True)
                os.replace(candidate, destination)
                published.append(destination)
        except BaseException:
            for destination in reversed(published):
                backup = backups[destination]
                if backup is None:
                    destination.unlink()
                else:
                    os.replace(backup, destination)
            raise
    print(f"Warp library ready: {library}", flush=True)
    return library


def library_loads(library):
    """Check ELF integrity/shared dependencies without initializing a CUDA device."""
    if not library.is_file() or library.stat().st_size == 0:
        return False
    result = subprocess.run([sys.executable, "-c", "import ctypes, sys; ctypes.CDLL(sys.argv[1])",
                             str(library)], capture_output=True, text=True)
    if result.returncode:
        print(result.stderr.strip(), file=sys.stderr)
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cuda-path", help="CUDA toolkit root; otherwise detect from environment/PATH")
    parser.add_argument("--force", action="store_true", help="Rebuild even if warp.so already exists")
    args = parser.parse_args()
    build_warp(args.cuda_path, force=args.force)


if __name__ == "__main__":
    main()
