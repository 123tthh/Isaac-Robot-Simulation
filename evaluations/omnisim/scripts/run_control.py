#!/usr/bin/env python3
"""Repeat the cube/floor control without importing R1 (no Isaac recording)."""
import argparse
import json
import urllib.request
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:6989')
    args = parser.parse_args()
    directory = Path(__file__).resolve().parents[1]

    def rpc(path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        request = urllib.request.Request(
            args.url + path, data, {'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=120) as response:
            return json.load(response)

    result = {'load': rpc('/world/load', {
        'path': str(directory / 'worlds/cube_floor_control.omniworld'), 'wait_s': 60})}
    result['step'] = rpc('/sim/step', {'steps': 160})
    result['target'] = rpc('/scene/node/TARGET')
    result['contacts'] = rpc('/sim/contacts')
    destination = directory / 'results/cube_floor_control.json'
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print(destination)
    print('Final cube position:', result['target']['position'])


if __name__ == '__main__':
    main()
