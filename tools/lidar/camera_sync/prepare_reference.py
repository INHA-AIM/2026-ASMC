#!/usr/bin/env python3
"""Extract exact pinned functions into a BUILD directory, never edit upstream."""
import argparse
import hashlib
import json
from pathlib import Path
import re

PIN = {
    'src/Lidar/TunnelFilter/TunnelFilterRos.cpp': '9bbcaef407b526185f61ab90fd9d9ba90bac432cbbbf81b5eb53826fd90d94ea',
    'src/Lidar/TunnelFilter/TunnelFilterCore.cpp': 'a0bef37f2ebb5aac53930514b02b345b668a57987c72b739af62259cb6c59dd3',
    'src/Lidar/TunnelFilter/TunnelFilterCore.hpp': 'b19eb10786cb1bb869b71106492fe1a02d656396a3ee8fb5da563b65a50d8f15',
    'src/Lidar/Tracking/Tracking.cpp': 'fd2fecf94b2689ababb3211db9bcdae2e85df5780ae732033e8aa9e70fea416d',
    'src/Lidar/Tracking/tracking_utils.hpp': 'af19a59406c1c1687f7af3bae99e71d7a0dc79ead9c04ee2b90a0115d3a0f332',
    'src/Lidar/RoiVoxel/RoiVoxel.cpp': '0d0ade13762fe75b6934203790041baa697053b17edadf4cde0589f795d64d2d',
    'src/Lidar/RoiVoxel/RoiVoxel.hpp': '87e774a14559a11f8a1485a42b1e51857504adf09740c434fc69cfda7fe9f4c0',
    'src/Camera/Camera.cpp': '11ca19e074104f9b3f5dfa267304b8e6af2dd55a1ee37702cd1988c1b138621b',
    'src/Global/Global.hpp': 'd96f607b6c1f025ce22ab35693d029161d6a38c028846535d8ea0bbe4c01fb95',
    'src/Global/Global.cpp': '37f4566358cfbf3e13db6ffd576641e07b330481d388da640158dfb2c6be7fd6',
    'setting.yaml': '0c317bcad9fc0162959dc7d4658e47cdf89ff557b8d47db2db73679f654f8f15',
}


def block(text, marker):
    start = text.index(marker)
    # Preserve offsets while ignoring comments/literals during brace balancing.
    mask = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: ' ' * len(m.group()), text, flags=re.S)
    brace = mask.index('{', start)
    depth = 1
    end = brace + 1
    while depth and end < len(mask):
        depth += (mask[end] == '{') - (mask[end] == '}')
        end += 1
    if depth:
        raise ValueError('unclosed source block: ' + marker)
    if text[end:end+1] == ';':
        end += 1
    return text[start:end]


def prepare(source, output):
    for rel, expected in PIN.items():
        if hashlib.sha256((source / rel).read_bytes()).hexdigest() != expected:
            raise ValueError('Son 6435af1 source mismatch: ' + rel)
    ros = (source / 'src/Lidar/TunnelFilter/TunnelFilterRos.cpp').read_text()
    tracking = (source / 'src/Lidar/Tracking/Tracking.cpp').read_text()
    camera = (source / 'src/Camera/Camera.cpp').read_text()
    global_source = (source / 'src/Global/Global.cpp').read_text()
    parts = ['#pragma once\n#include <Global/Global.hpp>\n#include <Eigen/Geometry>\n'
             '#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>\n'
             'extern VehicleSpec st_VehicleSpec;\n'
             'inline YAML::Node resolved_params;\n'
             'template<class T> void OfflineParam(const std::string& name, T& value, const T&) {'
             'const auto key=name.substr(name[0]==\'~\' ? 1 : 0);'
             'if(resolved_params[key]) value=resolved_params[key].as<T>();}\n']
    parts += [block(global_source, 'float32 Deg2Rad('),
              block(tracking, 'namespace obb_util {'), block(tracking, 'void Objectinfo_for_km(')]
    parts += [block(camera, 'namespace\n{\nEigen::Matrix3d RotationVehicleFromSensor'),
              block(camera, 'void InitProjectionMatrix(').replace('ros::param::param', 'OfflineParam')]
    parts += ['namespace pinned_camera {\nusing namespace tunnel_filter;\n',
              block(ros, 'struct YoloFrame {'), block(ros, 'struct ProtectionParams {'),
              'struct ExperimentState { ProtectionParams protection; '
              'Eigen::Matrix<double,3,4> projection; int image_width=1280; int image_height=720; };\n',
              block(ros, 'void ComputeProtection('), block(ros, 'ObbFunction MakeTrackingObbFunction()'),
              'struct OfflineNode { template<class T> void param(const std::string& n, T& v, const T& d) '
              '{v=resolved_params[n] ? resolved_params[n].as<T>() : d;} };\n',
              block(ros, 'template <typename T>\nvoid Param(').replace('ros::NodeHandle', 'OfflineNode'),
              block(ros, 'void ParamFloat(').replace('ros::NodeHandle', 'OfflineNode'),
              block(ros, 'Params LoadParams(').replace('ros::NodeHandle', 'OfflineNode'), '\n}\n']
    output.mkdir(parents=True, exist_ok=True)
    content = '\n'.join(parts) + '\n'
    (output / 'reference_generated.hpp').write_text(content)
    (output / 'reference_provenance.json').write_text(json.dumps({
        'revision': '6435af10671235e735e4a9ed1bd747bd8d8389ac', 'source_sha256': PIN,
        'generated_sha256': hashlib.sha256(content.encode()).hexdigest(),
        'adaptations': ['minimal ExperimentState without publishers',
            'namespace pinned_camera isolates extracted protection/OBB functions',
            'NodeHandle.param and projection overrides read frozen YAML without ROS master'],
        'exact_bodies': ['ComputeProtection', 'MakeTrackingObbFunction', 'Objectinfo_for_km',
                         'obb_util', 'LoadParams (parameter accessor only replaced)',
                         'InitProjectionMatrix (parameter accessor only replaced)'],
    }, sort_keys=True, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    prepare(a.source, a.output)
