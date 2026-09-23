#!/usr/bin/env python3
"""Validate the published OmniSim follow-up evidence offline."""
from pathlib import Path
import hashlib
import json
import math

BASE=Path(__file__).resolve().parents[1]
OLD=BASE/'results/v8_5_1'
NEW=BASE/'results/v9_0_0_rc2'
COMMITS={'v8_5_1':'0b9b07f5b646295bf4599a06e30f616b99090581',
         'v9_0_0_rc2':'661100a19f8d3f98f952f757409ca8ad99547e61'}

def read(directory,name):
    return json.loads((directory/name).read_text())

for version,directory in [('v8_5_1',OLD),('v9_0_0_rc2',NEW)]:
    for run in directory.glob('*.run.json'):
        record=json.loads(run.read_text())
        assert record['omnisim_commit']==COMMITS[version],run
        fixture=BASE/record['world']
        assert fixture.is_file() and hashlib.sha256(fixture.read_bytes()).hexdigest()==record['world_sha256'],run
        assert '/' not in Path(record['world']).parts[0],run
    for sidecar in directory.glob('*.engine.log.newton.json'):
        assert read(directory,sidecar.name)['finalised'],sidecar
for case in ['mid_empty','r1_portable_joint7']:
    assert read(OLD,case+'.run.json')['returncode']!=0
    assert read(NEW,case+'.run.json')['returncode']==0
assert 'left_base_link' in '\n'.join(read(OLD,'engine_excerpts.json')['r1_portable_joint7'])
assert 'FATAL' not in '\n'.join(read(NEW,'engine_excerpts.json')['r1_portable_joint7'])
for version,directory in [('old',OLD),('new',NEW)]:
    d=read(directory,'range_probe.json')
    asym=d['phases'][3]['sensor_positions']['j_asym_motor']
    expected=-3.13159 if version=='old' else -6.283
    assert math.isclose(asym,expected,abs_tol=1e-3),(version,asym)
    r1=read(directory,'r1_omnisim_joint7.json')
    for name in ['left_joint7_motor','right_joint7_motor']:
        j=next(j for j in r1['harness_snapshot']['joints'] if j['name']==name)
        assert [j['lower'],j['upper']]==[0,0]
        assert r1['motor_device_limits'][name]=={'min':-6.283,'max':6.283}
    thin=read(directory,'thin_16ms.json')
    small=read(directory,'thin_8ms.json')
    sub4=read(directory,'thin_16ms_sub4.json')
    assert not thin['stable_on_floor'] and thin['final']['position'][2]<-30
    assert small['stable_on_floor'] and sub4['stable_on_floor']
portable=read(NEW,'r1_portable_joint7.json')
assert max(abs(phase['sensor_positions'][name]-phase['goals'][name])
           for phase in portable['phases'] for name in ['left_joint7_motor','right_joint7_motor'])<1e-3
for world in (BASE/'worlds').glob('r1_*.omniworld'):
    import re
    match=re.search(r'url "([^"]+)"',world.read_text())
    assert match and (world.parent/match.group(1)).is_file(),world
print('PASS: published paths, engine revisions, import, limits, joint 7 and contacts')
