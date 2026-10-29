#!/usr/bin/env python3
"""Generate launch argument reference and check literal commands without running ROS.

Only current operator guides listed in the manifest are checked. Historical
records and frozen protocol examples are intentionally outside this command gate.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import re
import shlex
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'config/docs/runtime_documentation_v1.json'


def launch_args(path):
    """Top-level declarations only; include arguments are caller assignments."""
    result = {}
    for arg in ET.parse(path).getroot().findall('arg'):
        name = arg.attrib['name']
        if name in result:
            raise ValueError('duplicate top-level arg: ' + name)
        result[name] = dict(arg.attrib)
    return result


def table_cell(value):
    return value.replace('|', '&#124;').replace('\n', ' ')


def render(root, config):
    out = ['# Launch 인자 목록', '',
           '현재 XML의 최상위 인자를 자동 추출합니다. 실행 순서·권한·추천 조합은 [명령 안내](commands.md)를 따릅니다.',
           '인자가 없다고 component를 독립 실행해도 되는 것은 아닙니다. 내부 namespace·모델 경로·실험 설정은 YAML을 우선합니다.',
           '`$(...)`는 roslaunch가 해석하는 표현이며 여기에 표시된 문자열이 평가 결과는 아닙니다.', '',
           '갱신: `python3 tools/audit/check_runtime_documentation.py --write`',
           '검사: `python3 tools/audit/check_runtime_documentation.py`', '']
    for entry in config['launches']:
        path = root / entry['path']
        args = launch_args(path)
        out += ['## ' + entry['title'], '',
                '[소스](' + Path('../../'+entry['path']).as_posix() + ')', '',
                '| 인자 | 기본값 / 필수 | 소스 설명 |', '|---|---|---|']
        count = 0
        for name, arg in args.items():
            if name in entry.get('internal_args', []):
                continue
            default = arg.get('default', ('고정: '+arg['value']) if 'value' in arg else '필수')
            if default == '':
                default = '(빈 문자열)'
            out.append('| `' + name + '` | `' + table_cell(default) + '` | ' + table_cell(arg.get('doc','—')) + ' |')
            count += 1
        if not count:
            out.append('| — | 최상위 공개 인자 없음 | 구성은 소스/포함 launch 확인 |')
        if entry.get('internal_args'):
            out += ['', '유효성 검사 전용 내부 인자는 일반 사용자 옵션에서 제외합니다.']
        out.append('')
    return '\n'.join(out).rstrip() + '\n'


def shell_commands(text):
    """Read bash/sh fences, join continuations, and never expand shell expressions."""
    # Inline commands in operator tables are part of the same public surface.
    for match in re.finditer(r'`(roslaunch [^`\n]+)`', text):
        yield shlex.split(match[1], comments=True)
    blocks = re.finditer(r'^```(?:bash|sh|shell)\s*\n(.*?)^```', text, flags=re.M|re.S)
    for block in blocks:
        joined = re.sub(r'\\\n\s*', ' ', block[1])
        for line in joined.splitlines():
            if re.match(r'^\s*roslaunch\s', line):
                try:
                    yield shlex.split(line, comments=True)
                except ValueError as exc:
                    raise ValueError('malformed shell example: ' + line) from exc


def packages(root):
    result = {}
    for path in (root/'src').rglob('package.xml'):
        name = ET.parse(path).findtext('name')
        result.setdefault(name, []).append(path.parent)
    return result


def check_commands(root, config):
    errors = []
    package_paths = packages(root)
    count = 0
    for rel in config['command_guides']:
        path = root/rel
        for cmd in shell_commands(path.read_text()):
            count += 1
            if len(cmd) < 2:
                errors.append(f'{rel}: incomplete roslaunch example')
                continue
            if cmd[1].endswith('.launch'):
                # roslaunch also accepts a file outside a ROS package. Map the
                # documented container workspace to this checkout, never exec.
                pkg, launch = 'file', cmd[1]
                relative = launch[len('/root/ws/'):] if launch.startswith('/root/ws/') else launch
                target = (root / relative).resolve()
                matches = [target] if (target == root.resolve() or root.resolve() in target.parents) and target.is_file() else []
                arguments = cmd[2:]
            else:
                if len(cmd) < 3:
                    errors.append(f'{rel}: incomplete roslaunch example')
                    continue
                pkg, launch = cmd[1:3]
                if '$' in pkg or '<' in launch:
                    continue
                matches = [p for base in package_paths.get(pkg, []) for p in base.rglob(launch)]
                arguments = cmd[3:]
            if not matches:
                errors.append(f'{rel}: missing roslaunch target {pkg}/{launch}')
                continue
            declared = launch_args(matches[0])
            supplied = set()
            for token in arguments:
                if ':=' not in token or token.startswith('['):
                    continue
                name = token.split(':=',1)[0]
                supplied.add(name)
                if name not in declared:
                    errors.append(f'{rel}: unknown arg {pkg}/{launch} {name}')
            for name, declaration in declared.items():
                if 'default' not in declaration and 'value' not in declaration and name not in supplied:
                    errors.append(f'{rel}: missing required arg {pkg}/{launch} {name}')
    return errors, count



def check_labels(root):
    """Human label IDs/groups must match training order and canonical deployment."""
    labels = json.loads((root/'config/datasets/camera_object_labels_v2.json').read_text())
    runtime = yaml.safe_load((root/'src/asmc_camera/config/camera/object_classes_mission16_v1.yaml').read_text())
    groups = {name: group for group, names in runtime['class_groups']['groups'].items() for name in names}
    table = (root/'docs/competition/obstacle-assets-and-labels.md').read_text()
    actual = [(int(m[1]), m[2], m[3]) for m in re.finditer(
        r'^\| (\d+) \| ([a-z_]+) \| ([a-z_]+) \|', table, flags=re.M)]
    expected = [(i, name, groups.get(name)) for i, name in enumerate(labels['pilot_target_classes'])]
    return [] if actual == expected and all(row[2] for row in expected) else ['asset label table differs from fixed training IDs/deployment groups']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args(argv)
    config = json.loads(MANIFEST.read_text())
    expected = render(ROOT, config)
    output = ROOT/config['output']
    if args.write:
        output.write_text(expected)
    errors = []
    if not output.is_file() or output.read_text() != expected:
        errors.append('launch reference is stale; regenerate with --write')
    command_errors, count = check_commands(ROOT, config)
    errors += command_errors
    errors += check_labels(ROOT)
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'runtime documentation passed: launches={len(config["launches"])}, literal_commands={count}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
