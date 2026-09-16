#!/usr/bin/env python3
"""Check relocation without starting Isaac or recording data."""
import ast,os,tempfile,shutil,subprocess,json
from pathlib import Path
root=Path(__file__).resolve().parents[1];checks={}
with tempfile.TemporaryDirectory(prefix='r1 relocation ') as temp:
 relocated=Path(temp)/'project with spaces';relocated.mkdir()
 shutil.copy2(root/'run.sh',relocated/'run.sh')
 shutil.copytree(root/'scripts',relocated/'scripts',ignore=shutil.ignore_patterns('__pycache__'))
 shutil.copy2(root/'project_manifest.yaml',relocated/'project_manifest.yaml')
 result=subprocess.run([str(relocated/'run.sh'),'--help'],cwd='/tmp',capture_output=True,text=True,check=True)
 assert f'PROJECT_ROOT={relocated}' in result.stdout
 assert str(root) not in result.stdout
 checks['launcher_relocated_with_spaces']=True
 count=0
 for p in (root/'projects/physics_parameters').rglob('*.py'):
  if '.venv' in p.parts:continue
  tree=ast.parse(p.read_text());functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_physics_project_root']
  if not functions:continue
  code=compile(ast.Module(body=functions,type_ignores=[]),str(p),'exec')
  target=relocated/p.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);target.touch()
  env={'os':os,'_ProjectPath':Path,'__file__':str(target)}
  saved=os.environ.pop('ISAAC_OCS_PROJECT_ROOT',None)
  try:
   exec(code,env);assert env['_physics_project_root']()==relocated
   del env['__file__'];os.environ['ISAAC_OCS_PROJECT_ROOT']=str(relocated)
   assert env['_physics_project_root']()==relocated
  finally:
   if saved is None:os.environ.pop('ISAAC_OCS_PROJECT_ROOT',None)
   else:os.environ['ISAAC_OCS_PROJECT_ROOT']=saved
  count+=1
 checks['physics_resolvers_file_and_editor_modes']=count
for p in [root/'run.sh',* (root/'scripts').glob('*.sh')]:subprocess.run(['bash','-n',str(p)],check=True)
checks['shell_syntax']='passed'
checks['scope']='Relocation, path resolution and syntax only; no new simulation recording.'
(root/'reports/layout_portability_20260916.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,indent=2))
