#!/usr/bin/env python3
from pathlib import Path
import argparse,tempfile,subprocess,json,hashlib,tarfile,shutil
p=argparse.ArgumentParser();p.add_argument('--out',required=True,type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
ref=json.loads((Path(__file__).parent/'DATA_REF.json').read_text())
with tempfile.TemporaryDirectory(prefix='asmc-data-') as tmp:
 repo=Path(tmp)/'repo'
 subprocess.run(['git','clone','--quiet','--no-checkout','--single-branch','--branch',ref['branch'],ref['repository'],str(repo)],check=True)
 def get(name):return subprocess.check_output(['git','show',ref['commit']+':'+name],cwd=repo)
 index=json.loads(get('INDEX.json'));pack=Path(tmp)/'data.tar.gz'
 with pack.open('wb') as out:
  for part in index['parts']:
   b=get(part['name'])
   if hashlib.sha256(b).hexdigest()!=part['sha256']:raise SystemExit('part hash mismatch')
   out.write(b)
 if hashlib.sha256(pack.read_bytes()).hexdigest()!=ref['archive_sha256']:raise SystemExit('archive hash mismatch')
 target=a.out/index['directory']
 if target.exists():raise SystemExit('Refusing existing target '+str(target))
 with tarfile.open(pack) as tar:
  for member in tar.getmembers():
   dest=(a.out/member.name).resolve()
   if a.out.resolve() not in dest.parents and dest!=a.out.resolve():
    raise SystemExit('Unsafe archive path')
   if member.issym() or member.islnk():raise SystemExit('Unexpected link in data archive')
  tar.extractall(a.out)
 print(target)
