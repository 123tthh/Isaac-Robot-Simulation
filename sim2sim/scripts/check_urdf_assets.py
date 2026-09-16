#!/usr/bin/env python3
"""Check relative mesh references and hydrated files in both standalone URDFs."""
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    reports = []
    for name in ('r1_fixed_portable.urdf', 'r1_fixed_omnisim.urdf'):
        source = ROOT / 'projects/ros2_ws/src/robot/urdf' / name
        robot = ET.parse(source).getroot()
        references = [node.attrib['filename'] for node in robot.findall('.//mesh')]
        errors = []
        hashes = {}
        for reference in sorted(set(references)):
            path = source.parent / reference
            if '://' in reference or Path(reference).is_absolute():
                errors.append({'reference': reference, 'error': 'not relative'})
            elif not path.is_file():
                errors.append({'reference': reference, 'error': 'missing file'})
            else:
                content = path.read_bytes()
                if content.startswith(b'version https://git-lfs.github.com/spec/v1'):
                    errors.append({'reference': reference, 'error': 'LFS pointer'})
                hashes[reference] = hashlib.sha256(content).hexdigest()
        reports.append({'urdf': str(source.relative_to(ROOT)),
                        'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                        'links': len(robot.findall('link')),
                        'joints': len(robot.findall('joint')),
                        'mesh_references': len(references),
                        'unique_meshes': len(set(references)),
                        'mesh_sha256': hashes, 'errors': errors, 'passed': not errors})
    output = ROOT / 'sim2sim/results/urdf_assets.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(reports, indent=2) + '\n')
    for report in reports:
        print(report['urdf'], 'PASS' if report['passed'] else 'FAIL',
              report['mesh_references'], 'references,', report['unique_meshes'], 'unique meshes')
    return 0 if all(r['passed'] for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
