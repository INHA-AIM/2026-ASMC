#!/usr/bin/env python3
"""Record Windows Simulator state for the isolated measurement's host guard."""
import argparse
import json
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--interval',type=float,default=15)
    args=parser.parse_args()
    command=['/mnt/c/WINDOWS/System32/WindowsPowerShell/v1.0/powershell.exe','-NoProfile','-Command',
             'Get-Process | Where-Object { $_.ProcessName -eq "Simulator" } | Select-Object -ExpandProperty ProcessName']
    args.state.parent.mkdir(parents=True,exist_ok=True)
    try:
        while True:
            result=subprocess.run(command,capture_output=True,text=True,timeout=15)
            state=dict(checked_unix=time.time(),checker='Windows Get-Process Simulator',return_code=result.returncode,
                       simulator_processes=[s.strip() for s in result.stdout.splitlines() if s.strip()],error=result.stderr.strip())
            temporary=args.state.with_suffix('.tmp');temporary.write_text(json.dumps(state)+'\n');temporary.replace(args.state)
            with args.state.with_suffix('.jsonl').open('a') as history:history.write(json.dumps(state)+'\n')
            time.sleep(args.interval)
    except KeyboardInterrupt:pass


if __name__=='__main__':main()
