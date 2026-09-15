"""Own and clean up the simulation processes used by integration checks."""
import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("actions", "mtc", "tests", "render", "physics", "mpm"), default="actions")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--cable-solver", choices=("mpm", "rope_actor"), default="mpm")
    parser.add_argument("--cable-config", default="")
    parser.add_argument("--fixture-closeup", action="store_true")
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    python = str(workspace / ".venv/bin/python")
    if args.mode in ("physics", "mpm"):
        script = "check_mpm.py" if args.mode == "mpm" else "check_simulation.py"
        command = [python, str(Path(__file__).parents[1] / "scripts" / script)]
        if args.viewer:
            command.append("--viewer")
        if args.mode == "mpm":
            command += ["--render-output", "/tmp/dual_fr3_maniskill2_mpm.png"]
        return subprocess.call(command, cwd=workspace)
    if args.mode == "tests":
        env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
        return subprocess.call([python, "-m", "pytest", "-q", "src/dual_fr3_maniskill/test",
                                "src/dual_fr3_trunking_mtc/test"], cwd=workspace, env=env)
    if args.mode == "render":
        command = [python, str(Path(__file__).parents[1] / "scripts/check_simulation.py"),
                   "--render-output", "/tmp/dual_fr3_maniskill_scene.png"]
        if args.fixture_closeup:
            command.append("--fixture-closeup")
        return subprocess.call(command, cwd=workspace)
    log = Path(f"/tmp/dual_fr3_maniskill_{args.mode}.log")
    command = ["ros2", "launch"]
    if args.mode == "actions":
        command += ["dual_fr3_moveit_config", "demo.launch.py", "simulation_backend:=maniskill"]
    else:
        command += ["dual_fr3_trunking_mtc", "mtc_prototype.launch.py", "simulation_backend:=maniskill",
                    "execute:=true", "preparation_interactive:=false", "mtc_keep_alive_sec:=0.1",
                    f"cable_solver:={args.cable_solver}"]
        if args.cable_config:
            command.append(f"cable_config:={args.cable_config}")
    command += [f"use_rviz:={'true' if args.viewer else 'false'}",
                f"maniskill_viewer:={'true' if args.viewer else 'false'}", f"maniskill_python:={python}"]
    print(f"Starting {args.mode} verification; log: {log}", flush=True)
    with log.open("w") as output:
        process = subprocess.Popen(command, cwd=workspace, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            if args.mode == "actions":
                env = dict(os.environ, MANISKILL_CHECK_RVIZ="1" if args.viewer else "0")
                code = subprocess.call(["/usr/bin/python3", str(Path(__file__).with_name("check_ros_execution.py"))],
                                       cwd=workspace, env=env)
                if "process has died" in log.read_text(errors="replace"):
                    print("A launch process failed; see " + str(log), flush=True)
                    return 1
                return code
            deadline = time.monotonic() + args.timeout
            last_line = ""
            while time.monotonic() < deadline:
                contents = log.read_text(errors="replace")
                finished = re.search(
                    r"^\[([^]]+)\].*cached MTC solution execution finished",
                    contents, re.MULTILINE,
                )
                # Let the task dispose of its publishers and native objects
                # before stopping the remaining launch processes.
                if finished and f"[INFO] [{finished[1]}]: process has finished cleanly" in contents:
                    print("PASS: full existing MTC plan and cached stage execution completed", flush=True)
                    return 0
                failures = [line for line in contents.splitlines() if "process has died" in line]
                if failures or process.poll() is not None:
                    print("\n".join(contents.splitlines()[-35:]), flush=True)
                    return 1
                progress = [line for line in contents.splitlines()
                            if any(word in line for word in ("executing cached", "completed", "preparation pair", "planning attempt"))]
                if progress and progress[-1] != last_line:
                    last_line = progress[-1]
                    print(last_line, flush=True)
                time.sleep(1)
            print("Timed out; see " + str(log), flush=True)
            return 1
        finally:
            if process.poll() is None:
                # Let ROS launch signal its own children once. Signaling the whole
                # group here would deliver a second SIGINT during their cleanup.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


if __name__ == "__main__":
    raise SystemExit(main())
