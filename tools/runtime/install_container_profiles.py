#!/usr/bin/env python3
"""Install explicit per-container interactive ROS profiles, with rollback copies.

Only /root shell configuration changes. No source checkout, process, master,
world, mount, Docker Config.Env, or noninteractive automation is modified.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


ROOT = Path(__file__).resolve().parent
BLOCK = ('\n# BEGIN ASMC CONTAINER ROS PROFILE\n'
         'if [[ $- == *i* ]] && [[ -r /root/.asmc-container-ros.bash ]]; then\n'
         '  source /root/.asmc-container-ros.bash\n'
         'fi\n# END ASMC CONTAINER ROS PROFILE\n')


def read_file(container, name):
    result = subprocess.run(['docker', 'exec', container, 'cat', name], capture_output=True)
    if result.returncode:
        exists = subprocess.run(['docker', 'exec', container, 'test', '-e', name])
        if exists.returncode == 0:
            raise RuntimeError('cannot read existing file: ' + container + ':' + name)
        return None
    return result.stdout


def shell_config(old):
    text = old.decode()
    for marker in ('ASMC SHARED INTERACTIVE ROS DEFAULTS', 'ASMC CONTAINER ROS PROFILE'):
        text = re.sub(r'\n# BEGIN ' + marker + r'\n.*?# END ' + marker + r'\n?', '\n', text, flags=re.S)
    # These two exact blocks were installed by the Localization migration.
    text = re.sub(r'\n# Interactive terminals use the shared bridge; automated exec defaults stay isolated\.\n'
                  r'export ASMC_ROS_PROFILE=.*?\nsource /root/localization-env\.sh\n', '\n', text)
    text = text.replace('\n# Interactive comparison shells use the common raw bridge/master.\n'
                        'export ASMC_ROS_PROFILE=shared\nsource /root/localization-env.sh\n', '\n')
    return (text.rstrip() + '\n' + BLOCK).encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profiles', type=Path)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--backup', type=Path)
    args = parser.parse_args()
    profiles = json.loads(args.profiles.read_text())['containers']
    ports = [int(p[k]) for p in profiles for k in ('master_port', 'planning_port', 'lidar_port')]
    if len(ports) != len(set(ports)) or any(not 1024 <= port <= 65535 for port in ports):
        raise RuntimeError('container master/HTTP ports must be distinct and unprivileged')
    if args.apply:
        if not args.backup:
            parser.error('--apply requires a new --backup directory')
        args.backup.mkdir(parents=True, exist_ok=False)
    reports = []
    for profile in profiles:
        container = profile['container']
        actual_source = subprocess.check_output(
            ['docker', 'exec', container, 'readlink', '-f', '/root/ws'], text=True).strip()
        if actual_source != profile['source_root']:
            raise RuntimeError('source changed; container left untouched: ' + container)
        values = dict(ASMC_ROS_MASTER_PORT=profile['master_port'],
                      ASMC_PLANNING_WEB_PORT=profile['planning_port'],
                      ASMC_LIDAR_WEB_PORT=profile['lidar_port'],
                      ASMC_DEVEL_SETUP=profile['devel_setup'],
                      ASMC_EXTRA_SETUP=profile.get('extra_setup', ''),
                      ASMC_VALIDATION_LEASE_DIR=profile['lease_directory'],
                      ASMC_RUNTIME_LABEL=container)
        if profile.get('data_root'):
            if not str(profile['data_root']).startswith('/'):
                raise ValueError('data_root must be an absolute container path')
            values['ASMC_DATA']=profile['data_root']
        config = ''.join('export %s=%s\n' % (key, shlex.quote(str(value))) for key, value in values.items()).encode()
        bashrc = read_file(container, '/root/.bashrc')
        if bashrc is None:
            raise RuntimeError('existing .bashrc required')
        replacements = {
            '/root/.bashrc': shell_config(bashrc),
            '/root/.asmc-container-profile.env': config,
            '/root/.asmc-container-ros.bash': (ROOT/'container_ros.bash').read_bytes(),
            '/root/.asmc-launch-guard.py': (ROOT/'launch_guard.py').read_bytes(),
        }
        files = {}
        backup = args.backup/container if args.apply else None
        if backup:
            backup.mkdir(mode=0o700)
        for target, contents in replacements.items():
            before = read_file(container, target)
            files[target] = dict(before_sha256=hashlib.sha256(before).hexdigest() if before is not None else None,
                                 contents=base64.b64encode(contents).decode())
            if backup and before is not None:
                path = backup/Path(target).name
                path.write_bytes(before)
                path.chmod(0o600)
        if args.apply:
            script = '''import base64,hashlib,json,os,sys
from pathlib import Path
files=json.loads(sys.stdin.read())
for target,entry in files.items():
 p=Path(target);actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
 if actual!=entry['before_sha256']:raise RuntimeError('concurrent edit; stopped: '+target)
for target,entry in files.items():
 p=Path(target);tmp=p.with_name(p.name+'.asmc-install-'+str(os.getpid()))
 tmp.write_bytes(base64.b64decode(entry['contents']));tmp.chmod(p.stat().st_mode & 0o777 if p.exists() else 0o600);tmp.replace(p)
'''
            # The Python program is passed as argv; JSON is data on stdin.
            subprocess.run(['docker', 'exec', '-i', container, 'python3', '-c', script],
                           input=json.dumps(files).encode(), check=True)
        reports.append(dict(profile, source_verified=True, mode='standalone',
                            changed_files={name: hashlib.sha256(base64.b64decode(entry['contents'])).hexdigest()
                                           for name, entry in files.items()},
                            backup=str(backup) if backup else None,
                            master='http://127.0.0.1:' + str(profile['master_port'])))
    result = dict(applied=args.apply, containers=reports, nodes_started=False,
                  existing_process_environment_changed=False, new_interactive_shell_required=True)
    if args.apply:
        (args.backup/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
