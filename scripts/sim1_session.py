#!/usr/bin/env python3
"""Start Isaac GUI, OCS2, pygame recording and the six-camera viewer together."""
import argparse
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default=time.strftime('episode_%Y%m%d_%H%M%S'))
    parser.add_argument('--no-camera-grid', action='store_true')
    parser.add_argument('--validate-10s', action='store_true',
                        help='Record the bounded ten-second integration exercise and exit.')
    args = parser.parse_args()
    if Path(args.name).name != args.name or args.name in ('.', '..'):
        parser.error('--name must be a single directory name')
    run = ROOT / 'outputs' / 'sessions' / args.name
    run.mkdir(parents=True, exist_ok=False)
    children = []
    logs = []
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    def launch(name, command):
        path = run / f'{name}.log'
        log = path.open('w')
        logs.append(log)
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        print(f'{name}: PID {process.pid}, log {path}', flush=True)
        return process, path
    def wait_for(process, path, marker, seconds=180):
        deadline = time.monotonic() + seconds
        while not stopping and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'{path.name} exited with {process.returncode}; see {path}')
            output = re.sub(r'\x1b\[[0-9;]*m', '', path.read_text(errors='replace'))
            if marker in output:
                return
            time.sleep(.25)
        raise RuntimeError(f'Interrupted or timed out waiting for {marker}: {path}')
    try:
        sim, path = launch('isaac', [str(ROOT/'scripts/project.sh'), 'start-isaac'])
        wait_for(sim, path, '"playing": true')
        ocs, path = launch('ocs2', [str(ROOT/'scripts/project.sh'), 'launch-ocs2',
            'enable_rviz:=false', 'enable_target_manager:=false'])
        wait_for(ocs, path, 'Configured and activated ocs2_arm_controller')
        # Wait for both controllers and end-effector subscriptions to settle.
        time.sleep(3)
        teleop_command = [sys.executable,
            str(ROOT/'projects/teleoperation/sim1/keyboard_ocs2_gripper_teleop.py'),
            '--input-mode', 'pygame', '--reset-targets', '--record-cameras',
            '--no-reset-on-exit', '--log-csv', str(run/'trace.csv')]
        if args.validate_10s:
            teleop_command += ['--duration', '10', '--validation-motion', '--camera-startup-grace', '0.5']
        teleop, path = launch('teleop', teleop_command)
        if not args.no_camera_grid:
            launch('camera_grid', [sys.executable, str(ROOT/'projects/teleoperation/sim1/tools/camera_grid_viewer.py')])
        print(f'Recording session: {run}; focus pygame to control; Q finishes.', flush=True)
        while not stopping and teleop.poll() is None:
            if sim.poll() is not None or ocs.poll() is not None:
                raise RuntimeError('Isaac or OCS2 exited during recording')
            time.sleep(.25)
        return 0 if stopping else teleop.returncode
    finally:
        # Stop recorders before publishers so final CSV/video indexes are flushed.
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
        for log in logs:
            log.close()

if __name__ == '__main__':
    raise SystemExit(main())
