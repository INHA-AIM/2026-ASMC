#!/usr/bin/env python3
"""Build the production targets and small safety tests selected for PR CI."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def command(scope, root=ROOT):
    spec = json.loads((root/'.github/ci/essential.json').read_text())['jobs'][scope]
    return ['./scripts/build_ws.sh', *spec['targets'], '-j2', '-l2',
            '-DCATKIN_WHITELIST_PACKAGES='+';'.join(spec['packages']),
            '-DASMC_LIDAR_PIPELINE_REQUIRED=ON', '-DCMAKE_BUILD_TYPE=',
            '-DCMAKE_C_COMPILER_LAUNCHER=ccache', '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scope', choices=['planning', 'perception'])
    args = parser.parse_args()
    raise SystemExit(subprocess.call(command(args.scope), cwd=str(ROOT)))
