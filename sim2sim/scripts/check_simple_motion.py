#!/usr/bin/env python3
"""Check fixed-base R1 arm/finger motion through OmniSim's native joint API."""
import argparse
import hashlib
import json
import math
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'sim2sim'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:6989')
    parser.add_argument('--settle-steps', type=int, default=160)
    args = parser.parse_args()
    if args.settle_steps <= 0:
        parser.error('--settle-steps must be positive')

    def rpc(path, data=None):
        request = urllib.request.Request(
            args.url + path, None if data is None else json.dumps(data).encode(),
            {'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=120) as response:
            return json.load(response)

    world = DIRECTORY / 'worlds/r1_simple_motion.omniworld'
    urdf = ROOT / 'projects/ros2_ws/src/robot/urdf/r1_fixed_omnisim.urdf'
    result = {
        'tested_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'Fixed-base joint motion only; no recording, object grasp, mobile-base or sim-to-real claim.',
        'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
        'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
        'arm_error_tolerance_rad': 0.03,
        'finger_error_tolerance_m': 0.003,
        'settle_steps_per_phase': args.settle_steps,
        'load': rpc('/world/load', {'path': str(world), 'wait_s': 60}),
        'phases': [],
    }
    output = DIRECTORY / 'results/simple_motion.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    if not result['load'].get('ok') or not result['load'].get('supervisor_connected'):
        output.write_text(json.dumps(result, indent=2) + '\n')
        raise RuntimeError('World did not load completely; see ' + str(output))
    result['capabilities'] = rpc('/capabilities')
    result['joint_mapping'] = rpc('/robot/R1/joints')
    result['base_before'] = rpc('/scene/node/R1')
    arms = ['left_joint1_motor', 'right_joint1_motor',
            'left_joint4_motor', 'right_joint4_motor']
    fingers = [f'{side}_PGIA_joint{n}_motor' for side in ('left', 'right') for n in (1, 2)]
    home = {**dict.fromkeys(arms, 0.0), **dict.fromkeys(fingers, 0.04)}
    phases = [
        ('home', home),
        ('arms_out', {**home, **dict(zip(arms, [0.15, -0.15, -0.2, -0.2]))}),
        ('arms_return', home),
        ('fingers_open', {**home, **dict.fromkeys(fingers, 0.08)}),
        ('fingers_close', home),
    ]
    for name, commands in phases:
        response = rpc('/robot/R1/joints/set', {
            'joints': commands, 'settle_steps': args.settle_steps})
        values = response.get('joints', {})
        checks = {}
        for joint, requested in commands.items():
            measured = values.get(joint, {})
            actual = measured.get('achieved')
            tolerance = 0.003 if joint in fingers else 0.03
            checks[joint] = (isinstance(actual, (int, float)) and math.isfinite(actual)
                             and abs(actual - requested) <= tolerance
                             and measured.get('position_controllable') is True
                             and measured.get('clamped') is False)
        phase = {'name': name, 'commands': commands, 'response': response,
                 'joint_checks': checks, 'passed': all(checks.values())}
        result['phases'].append(phase)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(name, 'PASS' if phase['passed'] else 'FAIL', flush=True)

    # Explicit movement checks prevent a non-moving model passing a loose tolerance.
    home_values = result['phases'][0]['response']['joints']
    out_values = result['phases'][1]['response']['joints']
    open_values = result['phases'][3]['response']['joints']
    result['measured_excursion'] = {
        joint: abs((out_values if joint in arms else open_values)[joint]['achieved']
                   - home_values[joint]['achieved']) for joint in arms + fingers}
    result['movement_checks'] = {j: value > (0.05 if j in arms else 0.01)
                                 for j, value in result['measured_excursion'].items()}
    result['base_after'] = rpc('/scene/node/R1')
    result['fixed_base_displacement_m'] = math.dist(
        result['base_before']['position'], result['base_after']['position'])
    result['passed'] = (all(p['passed'] for p in result['phases'])
                        and all(result['movement_checks'].values())
                        and result['fixed_base_displacement_m'] < 1e-6)
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'passed': result['passed'], 'excursion': result['measured_excursion'],
                      'base_displacement_m': result['fixed_base_displacement_m']}), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
