#!/usr/bin/env python3
"""Reject reintroduced legacy build/runtime dependencies after B4.

History, bag schemas, source-provenance comments and historical checkout guards
are allowed. This checks executable imports/launches/dependencies, not prose.
"""
import ast
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
EXCLUDE={'build','devel','install','__pycache__','.git','node_modules','lidar+sensorfusion','MORAI-ROS_morai_msgs'}

def check(root=ROOT):
    errors=[]
    for relative in ('src/lidar+sensorfusion/src/Main/package.xml','src/lidar+sensorfusion/src/Msg/package.xml',
                     'src/planning_inputs/scripts/tracked_wire_bridge.py',
                     'src/planning_inputs/msg/ValidatedTrackedObjectArray.msg'):
        if (root/relative).exists():errors.append('retired runtime remains: '+relative)
    for base in ('src','tools/scen_runner','tools/perception','scripts'):
        for path in (root/base).rglob('*'):
            if not path.is_file() or EXCLUDE.intersection(path.relative_to(root).parts):continue
            rel=path.relative_to(root).as_posix()
            if path.name=='package.xml':
                for node in ET.parse(path).getroot():
                    if node.tag.endswith('depend') and node.text and node.text.strip() in {'Main','Msg'}:
                        errors.append(rel+': legacy package dependency')
            if path.suffix in ('.launch','.test'):
                xml=ET.parse(path).getroot()
                if any(n.get('pkg') in {'Main','Msg'} for n in xml.iter('node')) or any(
                    re.search(r'\$\(find (Main|Msg)\)',n.get('file','')) for n in xml.iter('include')):
                    errors.append(rel+': legacy launch dependency')
            if path.suffix=='.py' and 'test' not in path.parts and not path.name.startswith('test_'):
                try:tree=ast.parse(path.read_text())
                except SyntaxError as e:errors.append(rel+': '+str(e));continue
                for n in ast.walk(tree):
                    if isinstance(n,ast.ImportFrom) and (n.module or '').split('.')[0]=='Msg':errors.append(rel+': Msg import')
                    if isinstance(n,ast.Import) and any(i.name.split('.')[0]=='Msg' for i in n.names):errors.append(rel+': Msg import')
                    if isinstance(n,(ast.List,ast.Tuple)):
                        values=[v.value for v in n.elts if isinstance(v,ast.Constant) and isinstance(v.value,str)]
                        if values and values[0] in ('roslaunch','rosrun') and len(values)>1 and values[1]=='Main':errors.append(rel+': Main process launch')
            if path.suffix=='.sh':
                executable='\n'.join(line for line in path.read_text().splitlines() if not line.lstrip().startswith('#'))
                if re.search(r'(source|PYTHONPATH|SCEN_STACK_MESSAGES).*lidar\+sensorfusion',executable):errors.append(rel+': nested overlay dependency')
    adapter=(root/'src/planning_inputs/src/objects_node.cpp').read_text()
    if any(n in adapter for n in ('MotionClassifier','PlanningObjectFilter','ValidatedTrackedObjectArray','source_policy')):
        errors.append('planning adapter repeats retired perception policy')
    if '#include <asmc_msgs/LidarTrackingObservation.h>' not in adapter:errors.append('missing typed common consumer')
    for relative in ('src/asmc_planner/launch/autonomy_no_bridges_v2.launch','src/planning_inputs/launch/objects.launch'):
        xml=ET.parse(root/relative).getroot()
        if any(n.get('name') in ('source_policy','lidar_source_policy','objects_source') for n in xml.iter('arg')):
            errors.append(relative+': retired source selection')
    manifest=json.loads((root/'config/baselines/lidar_canonicalization_v1.json').read_text())
    for source in manifest['legacy_sources']:
        for key in ('current_reference','additional_reference'):
            if key in source and not (root/source[key]).exists():
                errors.append(source['path']+': missing preservation reference '+source[key])
    return sorted(set(errors))

def main():
    errors=check()
    for error in errors:print('FAIL '+error)
    if not errors:print('PASS canonical LiDAR build/launch/import/input ownership')
    return bool(errors)
if __name__=='__main__':sys.exit(main())
