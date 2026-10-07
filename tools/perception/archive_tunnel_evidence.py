#!/usr/bin/env python3
"""Preserve all B1a fixed/preliminary attempts and pinned replay source."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import xml.etree.ElementTree as ET


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(4194304),b''):h.update(chunk)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--analysis-revision',required=True)
    p.add_argument('--full-evidence-cache',type=Path,required=True,help='retained full archives outside Git')
    p.add_argument('--case-review',default='final-case-review-v2',help='representative same-scan figures and lossless point arrays')
    args=p.parse_args();root=Path(__file__).resolve().parents[2]
    fixed=args.evidence/'protocol-19a0b29'
    summary=json.loads((fixed/'summary-b1a.json').read_text())
    if not summary['fixed_protocol_complete']:raise ValueError('fixed protocol is incomplete')
    args.output.mkdir(parents=True,exist_ok=False)
    support=args.output/'support';support.mkdir()
    for src,name in [(fixed/'manifest.json','run-manifest.json'),(fixed/'summary-b1a.json','summary.json'),
                     (fixed/'analysis.json','analysis.json')]:shutil.copyfile(src,args.output/name)
    shutil.copytree(args.evidence/args.case_review,args.output/'cases')
    for name in ('environment-start.txt','environment-fixed.txt','mounts.json','image-network.txt',
                 'shared-processes-during-measurement.txt','morai-process-check.txt','host-state.jsonl',
                 'probe-core.log','probe-v1.json','probe-v1.log','source-build.log',
                 'map-probe-build.log','map-probe-build-failed-v1.log','map-probe.jsonl','map-probe.log'):
        path=args.evidence/name
        if path.exists():shutil.copyfile(path,support/name)
    for name in ('source-manifest.json','main_parameters.json','b1a_main.launch'):
        shutil.copyfile(args.source/name,support/name)
    test=args.source/'build/test_results/Main/gtest-test_tunnel_filter_core.xml'
    shutil.copyfile(test,support/test.name)
    probes=[]
    for name,prefix in [('probe-v1.json','B1A_PROBE '),('map-probe.jsonl','B1A_MAP_PROBE ')]:
        for line in (args.evidence/name).read_text().splitlines():
            if line.startswith(prefix):probes.append(json.loads(line[len(prefix):]))
    (support/'probes.json').write_text(json.dumps(probes,indent=2)+'\n')
    srcmanifest=json.loads((args.source/'source-manifest.json').read_text())
    mismatches=[name for name,sha in srcmanifest['source_sha256'].items() if digest(args.source/name)!=sha]
    if mismatches:raise ValueError('pinned source modified: '+str(mismatches))
    omitted=[]
    with tarfile.open(args.output/'replay-source.tar.gz','w:gz') as tar:
        for name in sorted(srcmanifest['source_sha256']):
            if Path(name).suffix=='.pt':omitted.append(name);continue
            tar.add(args.source/name,arcname=name)
        for path in sorted((args.source/'src/b1a_probe').glob('*')):
            tar.add(path,arcname=str(path.relative_to(args.source)))
        for name in ('b1a_main.launch','main_parameters.json','source-manifest.json'):
            tar.add(args.source/name,arcname=name)
    for phase in ('offline','realtime'):
        with tarfile.open(args.output/('raw-fixed-'+phase+'.tar.gz'),'w:gz') as tar:
            for path in sorted(fixed.iterdir()):
                if path.name.startswith(phase+'-') or path.name in ('manifest.json','protocol.json','index.json'):
                    tar.add(path,arcname='protocol-19a0b29/'+path.name,filter=lambda m:None if m.name.endswith('/clouds.bin') else m)
    with tarfile.open(args.output/'raw-preliminary.tar.gz','w:gz') as tar:
        for path in sorted(args.evidence.iterdir()):
            if path.name.startswith(('smoke-v1','protocol-79448bc','preliminary-case-review')):
                tar.add(path,arcname=path.name,filter=lambda m:None if m.name.endswith('/clouds.bin') else m)
        for fixture in ('fixtures','fixtures-final'):
            for path in sorted((args.evidence/fixture).glob('*.json')):
                tar.add(path,arcname=fixture+'/'+path.name)
    binaries=json.loads((fixed/'manifest.json').read_text())['binary_sha256']
    unchanged={name:digest(args.source/'devel/lib/Main'/name)==sha for name,sha in binaries.items()}
    fixture=json.loads((args.evidence/'fixtures-final/manifest.json').read_text())
    fixture_verified={b:{g:digest(args.evidence/'fixtures-final'/(b+'-'+g+'.bag'))==v['sha256'] for g,v in info['variants'].items()} for b,info in fixture['bags'].items()}
    closure=ET.parse(test).getroot()
    provenance=dict(analysis_revision=args.analysis_revision,packaging_script_sha256=digest(Path(__file__)),
        source_revision=srcmanifest['revision'],original_source_files=len(srcmanifest['source_sha256']),source_mismatches=mismatches,
        source_archive_excluded_unused_weights=omitted,measurement_binaries_unchanged=unchanged,
        fixture_bags_verified=fixture_verified,fixture_bags_archived=False,
        fixture_recreation='Original bag SHA + frozen event JSON + make_b1a_fixtures.py; original sensor data read-only.',
        retained_full_archives={name:dict(path=str(args.full_evidence_cache/name),sha256=digest(args.full_evidence_cache/name),bytes=(args.full_evidence_cache/name).stat().st_size) for name in ('raw-fixed-offline.tar.gz','raw-preliminary.tar.gz')},
        original_raw_bags_verified={b:digest(root.parent/info['source'].lstrip('/'))==info['source_sha256'] for b,info in fixture['bags'].items()},
        core_tests=closure.attrib,generated_message_MD5_in='support/environment-fixed.txt',
        postprocessing_sha256={name:digest(root/'tools/perception'/name) for name in ('analyze_b1a_matrix.py','summarize_tunnel_evidence.py','render_tunnel_filter_cases.py','tunnel_case_geometry.py')})
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    files={str(path.relative_to(args.output)):dict(sha256=digest(path),bytes=path.stat().st_size) for path in sorted(args.output.rglob('*')) if path.is_file()}
    (args.output/'files.json').write_text(json.dumps(files,indent=2)+'\n')
    print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),fixed_trials=44)))


if __name__=='__main__':main()
