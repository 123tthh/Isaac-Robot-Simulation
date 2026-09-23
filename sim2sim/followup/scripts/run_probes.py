#!/usr/bin/env python3
"""Run the bounded native OmniSim probes without a harness or WebSocket."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]
DEFAULT_CASES = [
    'mid_empty', 'mid_direct', 'root_empty', 'r1_portable_joint7',
    'r1_omnisim_joint7', 'range_probe', 'thin_16ms', 'thin_8ms',
    'thin_4ms', 'thick_16ms', 'thin_16ms_sub4', 'no_floor_16ms',
]

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('cases', nargs='*', default=DEFAULT_CASES, choices=DEFAULT_CASES)
    ap.add_argument('--omnisim-home', type=Path, default=Path(os.environ.get('OMNISIM_HOME', ROOT / '.runtime/omnisim-source')))
    ap.add_argument('--output-dir', type=Path, default=ROOT / 'outputs/omnisim_followup/replay')
    ap.add_argument('--timeout-s', type=int, default=180)
    args = ap.parse_args()
    engine = args.omnisim_home.expanduser().resolve()
    binary = engine / 'bin/omnisim-bin'
    if not binary.is_file():
        ap.error(f'OmniSim binary missing: {binary}')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=engine, text=True).strip()
    sys.path.insert(0, str(engine))
    from omnisim.paths import linux_runtime_env
    env = os.environ.copy()
    env['OMNISIM_HOME'] = str(engine)
    physics = os.environ.get('OMNISIM_PHYSICS_PYTHONPATH')
    if not physics and (ROOT / '.runtime/omnisim-physics').is_dir():
        physics = str(ROOT / '.runtime/omnisim-physics')
    if physics:
        env['PYTHONPATH'] = physics + (os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    native_libs = os.environ.get('OMNISIM_NATIVE_LIB_PATH')
    if not native_libs and (ROOT / '.runtime/omnisim-build-deps/root/usr/lib/x86_64-linux-gnu').is_dir():
        native_libs = str(ROOT / '.runtime/omnisim-build-deps/root/usr/lib/x86_64-linux-gnu')
    if native_libs:
        env['LD_LIBRARY_PATH'] = native_libs + (os.pathsep + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    env = linux_runtime_env(engine, env)
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    failed = []
    for name in args.cases:
        world = BASE / 'worlds' / (name + '.omniworld')
        local_env = env.copy()
        local_env['OMNI_PROBE_OUTPUT'] = str(output / (name + '.json'))
        local_env['OMNISIM_LOG_PATH'] = str(output / (name + '.engine.log'))
        command = [str(binary), '--batch', '--mode=fast', '--no-rendering', '--stdout', '--stderr', str(world)]
        with (output / (name + '.stdout.log')).open('w') as log:
            try:
                process = subprocess.run(command, cwd=engine, env=local_env, stdout=log,
                                         stderr=subprocess.STDOUT, timeout=args.timeout_s, check=False)
                status = process.returncode
            except subprocess.TimeoutExpired:
                status = 'timeout'
        record = {
            'started_at_utc': datetime.now(timezone.utc).isoformat(),
            'omnisim_commit': commit,
            'world': 'worlds/' + world.name,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'returncode': status,
        }
        (output / (name + '.run.json')).write_text(json.dumps(record, indent=2) + '\n')
        print(name, status, flush=True)
        if status != 0:
            failed.append(name)
    return 1 if failed else 0

if __name__ == '__main__':
    raise SystemExit(main())
