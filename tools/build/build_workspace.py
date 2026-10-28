#!/usr/bin/env python3
"""One catkin invocation and one dependency graph for runtime/full/selected builds."""
import fcntl
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
HELP = """Usage: scripts/build_ws.sh [runtime|full|TARGET ...] [catkin_make options]
  (no target) / runtime   build the competition runtime, including native camera kernel
  full / all             build every configured package's runtime, tool and C++ test
  TARGET ...             build selected targets through one shared dependency graph
  clean/install/run_tests and --catkin ... retain the original catkin_make interface
Build modes do not run tests. Full regression: see docs/operations/build.md.
ASMC_CCACHE=auto|off|on (on requires ccache). Explicit compiler launcher flags win.
"""
VALUE_OPTIONS = {'-j', '-l', '-D', '-G', '-A', '-T', '-b', '--build', '--devel',
                 '--source', '--install-space', '-C', '--directory'}
RAW_COMMANDS = {'clean', 'install', 'run_tests', 'help'}


def plan(arguments, cache_executable=None, cache_policy='auto'):
    args = list(arguments)
    if args and (args[0] == '--catkin' or args[0] in RAW_COMMANDS):
        return ['catkin_make'] + (args[1:] if args[0] == '--catkin' else args)
    if '--make-args' in args:
        raise ValueError('Use --catkin for raw --make-args, or put target names before the build options.')
    if cache_policy not in ('auto', 'off', 'on'):
        raise ValueError('ASMC_CCACHE must be auto, off or on')
    targets, options = [], []
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ('-C', '--directory', '--source'):
            raise ValueError('Use --catkin when selecting a different source/workspace directory')
        if arg in VALUE_OPTIONS:
            if i+1 == len(args):
                raise ValueError('Missing value for '+arg)
            options.extend(args[i:i+2]); i += 2; continue
        if arg.startswith('-'):
            options.append(arg)
        else:
            if not re.fullmatch(r'[A-Za-z0-9_.+-]+', arg):
                raise ValueError('Use --catkin for nonstandard target: '+arg)
            targets.append({'runtime':'asmc_runtime', 'full':'asmc_full', 'all':'asmc_full'}.get(arg, arg))
        i += 1
    targets = list(dict.fromkeys(targets or ['asmc_runtime']))
    definitions = [o[2:] for o in options if o.startswith('-D')]
    definitions += [options[n+1] for n,o in enumerate(options[:-1]) if o == '-D']
    keys = {d.split('=',1)[0].split(':',1)[0] for d in definitions}
    if 'ASMC_BUILD_REQUESTS' in keys:
        raise ValueError('ASMC_BUILD_REQUESTS is owned by the build wrapper; pass target names instead')
    flags = ['-DASMC_BUILD_REQUESTS='+';'.join(targets)]
    if 'CMAKE_BUILD_TYPE' not in keys:
        flags.append('-DCMAKE_BUILD_TYPE=Release')
    if {'asmc_runtime', 'asmc_full'}.intersection(targets) and 'ASMC_LIDAR_PIPELINE_REQUIRED' not in keys:
        flags.append('-DASMC_LIDAR_PIPELINE_REQUIRED=ON')
    if 'asmc_full' in targets and 'CATKIN_ENABLE_TESTING' not in keys:
        flags.append('-DCATKIN_ENABLE_TESTING=ON')
    if cache_policy == 'on' and not cache_executable:
        raise ValueError('ASMC_CCACHE=on requires ccache in PATH')
    for lang in ('C', 'CXX'):
        key = 'CMAKE_'+lang+'_COMPILER_LAUNCHER'
        if key not in keys and (cache_executable or cache_policy == 'off'):
            flags.append('-D'+key+'='+('' if cache_policy == 'off' else cache_executable))
    return ['catkin_make', 'asmc_requested', *options, *flags]


def build_directory(command):
    for arg in command:
        if arg.startswith('--build='):
            return Path(arg.split('=',1)[1]).resolve()
    for n,arg in enumerate(command[:-1]):
        if arg in ('-b', '--build'):
            return Path(command[n+1]).resolve()
    return ROOT/'build'


def main():
    if '-h' in sys.argv[1:] or '--help' in sys.argv[1:]:
        print(HELP); return 0
    os.chdir(ROOT)
    os.environ.setdefault('CCACHE_DIR', str(ROOT/'.ci-cache/ccache'))
    os.environ.setdefault('CCACHE_BASEDIR', str(ROOT))
    os.environ.setdefault('CCACHE_MAXSIZE', '400M')
    try:
        cmd = plan(sys.argv[1:], shutil.which('ccache'), os.environ.get('ASMC_CCACHE', 'auto'))
        directory = build_directory(cmd); directory.mkdir(parents=True, exist_ok=True)
        with (directory/'.asmc-build.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise ValueError('Another wrapper build owns this build directory: '+str(directory))
            print('[build_ws] '+' '.join(cmd), flush=True)
            return subprocess.call(cmd)
    except ValueError as error:
        print('[build_ws] ERROR: '+str(error), file=sys.stderr); return 2


if __name__ == '__main__':
    sys.exit(main())
