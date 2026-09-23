from pathlib import Path
import os
BASE=Path(__file__).resolve().parents[1]; ROOT=BASE.parents[1]
W=BASE/'worlds'; W.mkdir(parents=True,exist_ok=True)
header='''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep @DT@ gravity GRAV defaultPhysicsBackend "newton" newtonSolver "mujoco" newtonStatics TRUE SUBSTEPS }
Viewpoint { position 3 -4 2 orientation -0.42 0.38 0.82 1.85 }
'''
for name,step,thick,sub in [('thin_16ms',16,.05,1),('thin_8ms',8,.05,1),('thin_4ms',4,.05,1),('thick_16ms',16,.2,1),('thin_16ms_sub4',16,.05,4),('no_floor_16ms',16,0,1)]:
 floor=f'DEF FLOOR Solid {{ translation 0 0 {.025-thick/2} boundingObject Box {{ size 6 6 {thick} }} }}' if thick else ''
 body='''DEF TARGET Robot {
 translation 0 0.35 0.8 name "target"
 children [Shape { geometry Box {size 0.04 0.04 0.04} }]
 boundingObject Box {size 0.04 0.04 0.04}
 physics Physics {density -1 mass 0.05}
 supervisor TRUE controller "local_probe" controllerArgs ["contact"]
}
'''
 (W/f'{name}.omniworld').write_text(header.replace('@DT@',str(step)).replace('GRAV','9.81').replace('SUBSTEPS',f'newtonSubsteps {sub}')+floor+'\n'+body)
for variant in ['portable','omnisim']:
 model=ROOT/f'projects/ros2_ws/src/robot/urdf/r1_fixed_{variant}.urdf'
 # R1 no gravity is an explicit isolated motor/readback diagnostic, not a task claim.
 (W/f'r1_{variant}_joint7.omniworld').write_text(header.replace('@DT@','8').replace('GRAV','0').replace('SUBSTEPS','')+f'''DEF R1 URDFRobot {{ url "{os.path.relpath(model,W)}" name "r1" staticBase TRUE supervisor TRUE controller "local_probe" controllerArgs ["joint"] }}\n''')
def link(name,empty=False):
 return f'<link name="{name}">'+('' if empty else '<inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>')+'</link>'
def joint(name,typ,parent,child,lo=-2,hi=2):
 return f'<joint name="{name}" type="{typ}"><parent link="{parent}"/><child link="{child}"/><origin xyz="0 0 0.15"/><axis xyz="0 0 1"/>'+('' if typ=='fixed' else f'<limit lower="{lo}" upper="{hi}" effort="100" velocity="2"/>')+'</joint>'
for case in ['mid_empty','mid_direct','root_empty']:
 nodes=['root','mid','attach','bracket','arm']; parts=[link(n,n=='attach') for n in nodes if not (case=='mid_direct' and n=='attach') and not (case=='root_empty' and n=='mid')]
 if case!='root_empty':parts.append(joint('j_mid','revolute','root','mid'))
 if case=='mid_direct':parts.append(joint('j_bracket','fixed','mid','bracket'))
 else:parts.extend([joint('j_attach','fixed','root' if case=='root_empty' else 'mid','attach'),joint('j_bracket','fixed','attach','bracket')])
 parts.append(joint('j_arm','revolute','bracket','arm'))
 (W/f'{case}.urdf').write_text('<robot name="merge_probe">'+''.join(parts)+'</robot>\n')
 (W/f'{case}.omniworld').write_text(header.replace('@DT@','8').replace('GRAV','0').replace('SUBSTEPS','')+f'DEF R1 URDFRobot {{ url "{case}.urdf" staticBase TRUE supervisor TRUE controller "local_probe" controllerArgs ["merge"] }}\n')
parts=[link('root')]
for i,(name,lo,hi) in enumerate([('j_inside',-2,2),('j_full',-6.283,6.283),('j_asym',-6.283,.785)]):
 parts.extend([link(name),joint(name,'revolute','root',name,lo,hi).replace('0 0 0.15',f'{i*.3} 0 .15')])
(W/'range_probe.urdf').write_text('<robot name="range_probe">'+''.join(parts)+'</robot>\n')
(W/'range_probe.omniworld').write_text(header.replace('@DT@','8').replace('GRAV','0').replace('SUBSTEPS','')+'DEF R1 URDFRobot {url "range_probe.urdf" staticBase TRUE supervisor TRUE controller "local_probe" controllerArgs ["range"]}\n')
print(W)
