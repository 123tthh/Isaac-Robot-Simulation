#!/usr/bin/env python3
"""Create/verify a portable content-hash freeze of runtime source and assets."""
import argparse,hashlib,json,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
ROOTS=('scripts','assets','projects/ros2_ws/src','projects/teleoperation/sim1','projects/physics_parameters','dependencies','docker','docs','tests','spec')
SKIP={'__pycache__','.git','.venv','venv','trace_data','outputs','build','install','log','wheelhouse','docker-wheelhouse'}
BINARY={'.stl','.usd','.usda','.usdc','.png','.jpg','.jpeg','.mp4','.gif','.npy','.npz','.so','.whl','.pdf'}
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def files():
 for folder in ROOTS:
  for p in sorted((ROOT/folder).rglob('*')):
   rel=p.relative_to(ROOT)
   if p.is_file() and not p.is_symlink() and not any(x in SKIP or x.startswith('trace_data_backup') for x in rel.parts) and p.suffix.lower() not in {'.pyc','.log'}:
    yield p
 for name in ('run.sh','README.md','README.zh-CN.md','project_manifest.yaml','.gitignore','.dockerignore','.gitattributes','.gitmodules','LICENSE'):
  p=ROOT/name
  if p.is_file():yield p

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('release',type=Path);p.add_argument('--verify',action='store_true');a=p.parse_args();release=a.release.resolve();manifest=release/'core_manifest.json'
 if a.verify:
  old=json.loads(manifest.read_text());fail=[name for name,sha in old['sha256'].items() if not (ROOT/name).is_file() or digest(ROOT/name)!=sha]
  print(json.dumps({'verified_files':len(old['sha256']),'mismatches':fail}));return bool(fail)
 release.mkdir(parents=True,exist_ok=True)
 if manifest.exists():raise SystemExit('Freeze already exists; verify it or choose a new release directory.')
 paths=list(files());hashes={str(x.relative_to(ROOT)):digest(x) for x in paths}
 manifest.write_text(json.dumps({'schema_version':1,'scope':list(ROOTS),'sha256':hashes},indent=2)+'\n')
 with tarfile.open(release/'source.tar.gz','w:gz') as tar:
  for item in paths:
   if item.suffix.lower() not in BINARY and item.stat().st_size<5_000_000:
    tar.add(item,arcname=str(item.relative_to(ROOT)),recursive=False)
 (release/'SHA256SUMS').write_text('\n'.join(f'{digest(release/n)}  {n}' for n in ('core_manifest.json','source.tar.gz'))+'\n')
 print(json.dumps({'frozen_files':len(paths),'release':str(release),'source_archive_bytes':(release/'source.tar.gz').stat().st_size}))
 return 0
if __name__=='__main__':raise SystemExit(main())
