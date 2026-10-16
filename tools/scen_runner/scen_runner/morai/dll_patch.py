"""Build, install and restore the retained MORAI fixes from WSL."""
import argparse
from pathlib import Path
import subprocess

from scen_runner.config import TOOL_ROOT, LOCAL_CONFIG, load_local_config

FIX_FLAGS = ('-FixSelf', '-FixContacts', '-LifecycleTrace', '-SpawnTrace', '-StaticContacts')
COMMANDS = {
    'build-fix': ('build_trace.ps1', FIX_FLAGS),
    'install-fix': ('install_trace.ps1', FIX_FLAGS),
    'build-static-contact-fix': ('build_trace.ps1', FIX_FLAGS),
    'install-static-contact-fix': ('install_trace.ps1', FIX_FLAGS),
    'restore-static-contact-fix': ('install_trace.ps1', ('-RestoreStaticContacts',)),
    'build-path-cleanup-fix': ('build_path_cleanup_fix.ps1', ()),
    'install-path-cleanup-fix': ('install_path_cleanup_fix.ps1', ()),
    'restore-path-cleanup-fix': ('install_path_cleanup_fix.ps1', ('-Restore',)),
    'restore-activation-trace': ('restore_activation_trace.ps1', ()),
    'restore': ('install_trace.ps1', ('-Restore',)),
}


def windows_path(path):
    return subprocess.check_output(['wslpath', '-w', str(path)], text=True).strip()


def main(argv=None):
    parser = argparse.ArgumentParser(description='MORAI 최종 접촉/경로 수정본 빌드·설치·원복')
    parser.add_argument('action', choices=COMMANDS)
    parser.add_argument('--managed', help='Simulator_Data/Managed WSL 경로; 생략하면 config/local.yaml의 dll.managed_path')
    args = parser.parse_args(argv)
    # An explicit path must work without local.yaml (including an invalid one).
    managed = args.managed
    if managed is None:
        try:
            managed = load_local_config().get('dll', {}).get('managed_path')
        except (OSError, ValueError) as error:
            parser.error(str(error))
    if not isinstance(managed, str) or not managed.strip():
        parser.error(f'DLL 설치 경로가 없습니다. {LOCAL_CONFIG}의 dll.managed_path를 입력하거나 '
                     "--managed '<자신의 Simulator_Data/Managed WSL 경로>'를 지정하세요.")
    name, flags = COMMANDS[args.action]
    script = TOOL_ROOT/'dll/scripts'/name
    if not script.is_file():
        parser.error('필수 DLL 도구 파일이 없습니다: '+str(script))
    command = ['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
               '-File', windows_path(script), '-ManagedPath', windows_path(Path(managed.strip())), *flags]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(result.stdout.decode('cp949', errors='replace'))
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
