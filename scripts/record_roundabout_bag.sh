#!/usr/bin/env bash
# Host entry: explicit runtime and a storage-registered run; starts no ROS node except recorder.
set -euo pipefail
if [[ ${1:-} == --help || $# != 3 ]]; then
  echo 'Usage: record_roundabout_bag.sh CONTAINER ROS_MASTER_URI REGISTERED_RUN_ID'
  echo 'The run must be active and its D output must have a writable container mount.'
  [[ ${1:-} == --help ]] && exit 0
  exit 2
fi
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(dirname "$repo_dir")"
source "$workspace_dir/.storage/env.sh"
python3 "$workspace_dir/tools/storage/manage.py" check --writing
exec python3 - "$workspace_dir" "$@" <<'PY'
import datetime,json,os,re,subprocess,sys
from pathlib import Path
from urllib.parse import urlparse
workspace,container,master,run_id=sys.argv[1:]
if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*',run_id): raise SystemExit('Invalid run ID')
u=urlparse(master)
if u.scheme!='http' or not u.hostname or not u.port: raise SystemExit('Explicit ROS HTTP master required')
record=json.loads((Path(workspace)/'.storage/runs'/run_id/'run.json').read_text())
if record['status']!='active': raise SystemExit('Run is frozen; register a new run')
root=Path(os.environ['ASMC_STORAGE_DATA']).resolve()
# ASMC_STORAGE_DATA is D/data; the registry uses paths relative to the D root.
while not (root/'catalog').is_dir() and root.parent!=root: root=root.parent
output=(root/record['output']).resolve()
if not output.is_relative_to(root): raise SystemExit('Output outside storage root')
info=json.loads(subprocess.check_output(['docker','inspect',container]))[0]
if not info['State']['Running']: raise SystemExit('Container is not running')
choices=[]
for m in info['Mounts']:
 p=Path(m['Source']).resolve()
 if m.get('RW') and output.is_relative_to(p): choices.append((len(str(p)),Path(m['Destination'])/output.relative_to(p)))
if not choices: raise SystemExit('Registered output has no writable mount; existing data preserved')
destination=max(choices)[1]
output.mkdir(parents=True,exist_ok=True)
name='roundabout-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.bag'
if (output/name).exists(): raise SystemExit('Recording exists')
cmd=['docker','exec','-i',container,'bash','-c',
 'set -eo pipefail; source /opt/ros/noetic/setup.bash; source /devel/setup.bash; export ROS_MASTER_URI="$1"; exec rosbag record --buffsize=512 -O "$2" /planning/vehicle/state /planning/objects /behavior/command_v2 /planning/frenet_feedback_v2 /planning/feedback /planning/trajectory /ctrl_cmd/normal /ctrl_cmd /control/binding_status /planning/mode',
 'record-roundabout',master,str(destination/name)]
print('Recording',run_id,'on',master,'to',output/name,flush=True)
os.execvp(cmd[0],cmd)
PY
