"""Pure B1a fixture, launch and GPS protocol rules (no ROS dependency)."""
import copy
import math
import re
import xml.etree.ElementTree as ET


def gps_action(mode, relative_sec, window):
    if mode not in ('normal', 'invalid', 'missing'):
        raise ValueError('unknown GPS mode')
    return mode if window[0] <= relative_sec < window[1] else 'normal'


def legacy_boxes(row, allowed=('bus', 'truck', 'person', 'car'), confidence=.7):
    if not row['valid'] or row['frame_id'] != 'camera_front':
        return []
    result = []
    for box in row['boxes']:
        label, score, coords = box['label'].lower(), box['confidence'], box['xyxy']
        if label not in allowed or not math.isfinite(score) or score < confidence:
            continue
        if len(coords) != 4 or not all(math.isfinite(v) for v in coords):
            raise ValueError('invalid original box coordinates')
        result.append(dict(label=box['label'], xyxy=[int(v) for v in coords]))
    return result


def frozen_events(observation):
    rows, stamps, previous = [], set(), -1
    for row in observation['outputs']['yolo']:
        if not row['valid']:
            continue
        if row['stamp_ns'] in stamps:
            raise ValueError('duplicate YOLO source stamp')
        stamps.add(row['stamp_ns'])
        available = row['ros_ns']
        if available <= 0 or available < previous:
            raise ValueError('invalid recorded result availability clock')
        previous = available
        rows.append(dict(stamp_ns=row['stamp_ns'], frame_id=row['frame_id'],
                         available_ns=available, original_arrival_monotonic=row['arrival'],
                         boxes=legacy_boxes(row)))
    return rows


def launch_values(source):
    """Resolve the original Main node's private parameters without copying logic."""
    tree = ET.fromstring(source)
    defaults = {a.attrib['name']: a.attrib['default'] for a in tree.findall('arg')}
    main = next(n for n in tree.findall('node') if n.attrib['name'] == 'Main')
    values = {}
    for param in main.findall('param'):
        value = param.attrib['value']
        value = re.sub(r'\$\(arg ([^)]+)\)', lambda m: defaults[m.group(1)], value)
        if '$(find' in value:
            values[param.attrib['name']] = value
        elif value in ('true', 'false'):
            values[param.attrib['name']] = value == 'true'
        else:
            try:
                values[param.attrib['name']] = float(value) if '.' in value else int(value)
            except ValueError:
                values[param.attrib['name']] = value
    return values


def test_launch(source):
    """Retain Main parameters/remaps; exclude driver, detector and GUI nodes."""
    original = ET.fromstring(source)
    result = ET.Element('launch')
    for arg in original.findall('arg'):
        result.append(copy.deepcopy(arg))
    node = copy.deepcopy(next(n for n in original.findall('node') if n.attrib['name'] == 'Main'))
    ET.SubElement(node, 'param', {'name': 'planner_udp_enabled', 'value': 'false'})
    for public, private in [('/ctrl_cmd', '/b1a/discarded_ctrl_cmd'),
                            ('/ctrl_cmd_0', '/b1a/discarded_ctrl_cmd_0'),
                            ('/image_jpeg/compressed', '/camera/image/compressed'),
                            ('/camera/image/compressed', '/b1a/front_forwarded')]:
        ET.SubElement(node, 'remap', {'from': public, 'to': private})
    result.append(node)
    return ET.tostring(result, encoding='unicode')+'\n'


def jobs(config, phase):
    if phase == 'offline':
        return [dict(phase=phase, bag_id=b, profile=p, gps_mode=g, repeat=1)
                for b in config['bags'] for g in config['offline_gps_modes'] for p in config['offline_profiles']]
    return [dict(phase=phase, bag_id=b, profile=p, gps_mode=g, repeat=r+1)
            for r in range(config['conditions']['repeats']) for b in config['bags']
            for g in config['gps_modes'] for p in config['realtime_profiles'][r % 2:]+config['realtime_profiles'][:r % 2]]
