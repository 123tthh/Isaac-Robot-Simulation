#!/usr/bin/env python3
"""Capture an actual OmniSim viewport after loading the simple-motion world."""
import argparse
import hashlib
import json
import urllib.request
from pathlib import Path

DIRECTORY = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:6989')
    args = parser.parse_args()

    def rpc(path, payload):
        request = urllib.request.Request(args.url + path, json.dumps(payload).encode(),
                                         {'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=120) as response:
            return json.load(response)

    output = DIRECTORY / 'images/omnisim_r1_motion.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    record = {'pose': rpc('/robot/R1/joints/set', {'joints': {
        'left_joint1_motor': 0.15, 'right_joint1_motor': -0.15,
        'left_joint4_motor': -0.2, 'right_joint4_motor': -0.2,
        'left_PGIA_joint1_motor': 0.08, 'left_PGIA_joint2_motor': 0.08,
        'right_PGIA_joint1_motor': 0.08, 'right_PGIA_joint2_motor': 0.08,
    }, 'settle_steps': 160})}
    # Explicit framing avoids the importer's inflated aggregate bounds query.
    record['frame'] = rpc('/scene/frame', {
        'target': [0, 0, 1.05], 'radius': 1.05, 'mode': 'hero', 'margin': 1.15})
    record['step'] = rpc('/sim/step', {'steps': 10})
    record['screenshot'] = rpc('/world/screenshot', {'path': str(output)})
    data = output.read_bytes()
    if not data.startswith(b'\x89PNG\r\n\x1a\n'):
        raise RuntimeError('Renderer did not produce a PNG')
    record['image_sha256'] = hashlib.sha256(data).hexdigest()
    record['note'] = 'Native OmniSim viewport capture; no synthetic image or compositing.'
    (DIRECTORY / 'results/screenshot_capture.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record['screenshot']))


if __name__ == '__main__':
    main()
