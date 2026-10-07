#!/usr/bin/env python3
"""Export the pinned teammate source into a separate disposable catkin workspace."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

import yaml
from b1a_protocol import test_launch, launch_values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
    revision = config['source_revision']
    prefix = 'src/lidar+sensorfusion/'
    data = subprocess.check_output(['git', 'archive', revision, '--']+
                                  [prefix+'src/'+name for name in ('Main', 'Msg', 'morai_msgs')], cwd=root)
    args.output.mkdir(parents=True, exist_ok=False)
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        for member in archive.getmembers():
            if not member.name.startswith(prefix) or member.isdir():
                continue
            relative = Path(member.name[len(prefix):])
            if relative.is_absolute() or '..' in relative.parts or not member.isfile():
                raise ValueError('unsupported archive member '+member.name)
            dest = args.output/relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(archive.extractfile(member).read())
            dest.chmod(member.mode)
    original = (args.output/'src/Main/main.launch').read_text()
    (args.output/'b1a_main.launch').write_text(test_launch(original))
    (args.output/'main_parameters.json').write_text(json.dumps(launch_values(original), indent=2)+'\n')
    sources = {str(p.relative_to(args.output)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in (args.output/'src').rglob('*') if p.is_file()}
    (args.output/'source-manifest.json').write_text(json.dumps(dict(revision=revision, source_sha256=sources,
        operational_source_modified=False, original_launch_sha256=hashlib.sha256(original.encode()).hexdigest()), indent=2)+'\n')
    print('exported',revision,len(sources),'unchanged source files')


if __name__ == '__main__':
    main()
