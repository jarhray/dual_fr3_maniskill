#!/usr/bin/env bash
# Install the matching ManiSkill2 and Warp source together. Never change the
# system Python, user-site packages, NVIDIA driver, or global CUDA symlink.
set -euo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
package="$workspace/src/dual_fr3_maniskill"
deps="$workspace/.deps"
source_dir="$deps/ManiSkill-0.5.3"
venv="$workspace/.venv"
cuda_path=$(PYTHONPATH="$package${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -c \
    'from dual_fr3_maniskill.warp_setup import find_cuda_toolkit; print(find_cuda_toolkit())')
mkdir -p "$deps"
touch "$deps/COLCON_IGNORE"
if [[ ! -f "$source_dir/setup.py" ]]; then
    curl -L --fail --retry 3 https://codeload.github.com/mani-skill/ManiSkill/tar.gz/refs/tags/v0.5.3 \
        -o "$deps/maniskill2-v0.5.3.tar.gz"
    echo "a61c953cb8fee91aaf8668f34c9fcfe79d1d5b340875d67ac5926d635a0dfdf3  $deps/maniskill2-v0.5.3.tar.gz" | sha256sum --check
    tar -xzf "$deps/maniskill2-v0.5.3.tar.gz" -C "$deps"
fi
if [[ -d "$venv" ]] && ! "$venv/bin/python" -c \
    'import importlib.util, site; assert not site.ENABLE_USER_SITE; assert importlib.util.find_spec("mani_skill") is None' 2>/dev/null; then
    backup="$workspace/.venv.maniskill3-backup-$(date +%Y%m%d-%H%M%S)"
    mv "$venv" "$backup"
    echo "Previous venv preserved at $backup (restore to .venv to use it)."
fi
# Ubuntu may lack python3.10-venv/ensurepip; the stdlib can still create an
# isolated environment, and pip can be bootstrapped without apt or sudo.
/usr/bin/python3 -m venv --without-pip "$venv"
touch "$venv/COLCON_IGNORE"
export PIP_CACHE_DIR="$deps/pip-cache"
if ! "$venv/bin/python" -m pip --version >/dev/null 2>&1; then
    curl -L --fail --retry 3 https://bootstrap.pypa.io/get-pip.py -o "$deps/get-pip.py"
    "$venv/bin/python" "$deps/get-pip.py" 'pip==25.3' 'setuptools==75.8.0' 'wheel==0.45.1'
fi
"$venv/bin/python" -m pip install 'pip==25.3' 'setuptools==75.8.0' 'wheel==0.45.1'
"$venv/bin/python" -m pip install -r "$package/requirements-maniskill2.txt" -e "$source_dir"
PYTHONPATH="$package${PYTHONPATH:+:$PYTHONPATH}" \
    "$venv/bin/python" -m dual_fr3_maniskill.warp_setup --cuda-path "$cuda_path"
test -s "$source_dir/warp_maniskill/warp/bin/warp.so"
"$venv/bin/python" -m pip check
"$venv/bin/python" -m pip freeze --local > "$deps/maniskill2-installed.txt"
echo "ManiSkill2 and its CUDA Warp library are installed in $venv."
