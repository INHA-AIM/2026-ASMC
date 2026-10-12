#!/usr/bin/env python3
"""Preserve readable originals when a public project has no released version.

This is explicitly NOT a publisher dataset version. No version is generated and
no original annotation is overwritten. Signed media URLs are never persisted.
Run after version downloads, so the shared two-download limit is respected.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import threading
import zipfile

from download_roboflow_sources import sha, write_json, error_kind, extract_checked


def prepare_export(project_id, root, key):
    import requests
    from roboflow.adapters import rfapi
    path = root / 'search-export-jobs.json'
    jobs = json.loads(path.read_text()) if path.exists() else {}
    if project_id not in jobs:
        workspace, project = project_id.split('/')
        with requests.Session() as session:
            token = rfapi.start_search_export(key, workspace, 'project:'+project, 'coco', session, dataset=project)
        if token.startswith('http'):
            raise ValueError('unexpected_export_url_response')
        jobs[project_id] = token
        write_json(path, jobs)
    return jobs[project_id]


def bulk_snapshot(project_id, root, key):
    """Official search export preserves native polygon/RLE in the original ZIP."""
    import requests
    from roboflow.adapters import rfapi
    info = rfapi.get_project(key, *project_id.split('/'))
    if not info['project'].get('public') or info.get('versions'):
        raise ValueError('snapshot_only_for_public_projects_without_versions')
    token = prepare_export(project_id, root, key)
    deadline = time.monotonic() + 600
    with requests.Session() as session:
        while True:
            result = rfapi.get_search_export(key, project_id.split('/')[0], token, session)
            if result.get('ready'):
                break
            if time.monotonic() > deadline:
                raise TimeoutError('snapshot_export_timeout')
            time.sleep(5)
    destination = root / 'raw' / project_id / 'unversioned-snapshot'
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / 'original.zip'
    if not archive.exists():
        temporary = destination / 'original.zip.partial'
        with requests.get(result['link'],stream=True,timeout=(30,90)) as response:
            response.raise_for_status()
            with temporary.open('wb') as f:
                for chunk in response.iter_content(1024*1024):
                    f.write(chunk)
        temporary.replace(archive)
    extract_checked(archive,destination/'export')
    images = 0
    for path in (destination/'export').rglob('*.json'):
        data=json.loads(path.read_text())
        if isinstance(data,dict) and all(k in data for k in ('images','annotations','categories')):
            images += len(data['images'])
    expected=info['project']['images']
    metadata = {k:info['project'].get(k) for k in ('id','name','type','classes','images','license','public')}
    metadata.update(version=None,snapshot=True,acquired_images=images,
        source_url='https://universe.roboflow.com/'+project_id,captured_at=datetime.now(timezone.utc).isoformat(),
        snapshot_method='official_search_export',version_policy_exception='No released public version')
    write_json(destination/'metadata.json',metadata)
    complete=images == expected
    return dict(project=project_id,status='downloaded_snapshot' if complete else 'incomplete_snapshot',
        version='unversioned-snapshot',raw_path=str(destination.resolve()),metadata=metadata,
        images=images,expected_images=expected,archive_sha256=sha(archive),publisher_archive=True,
        original_version_available=False)


def boxes_to_coco(record, image_id, categories, first_annotation):
    annotation = record.get('annotation') or {}
    if isinstance(annotation,str):
        annotation=json.loads(annotation)
    width, height = annotation.get('width'), annotation.get('height')
    if not width or not height:
        raise ValueError('missing_native_dimensions')
    result = []
    for box in annotation.get('boxes', []):
        name = str(box['label'])
        if name not in categories:
            categories[name] = len(categories)
        x,y,w,h = (float(box[k]) for k in ('x','y','width','height'))
        # Native Roboflow boxes use centre x/y. Original values remain in JSONL.
        item=dict(id=first_annotation+len(result),image_id=image_id,
            category_id=categories[name],bbox=[x-w/2,y-h/2,w,h],area=w*h,iscrowd=0)
        points=box.get('points')
        if points:
            if isinstance(points,str):points=json.loads(points)
            xy=[(float(p['x']),float(p['y'])) if isinstance(p,dict) else (float(p[0]),float(p[1])) for p in points]
            item['segmentation']=[[v for point in xy for v in point]]
            xs,ys=zip(*xy)
            item['bbox']=[min(xs),min(ys),max(xs)-min(xs),max(ys)-min(ys)]
        result.append(item)
    return result, width, height


def search_records(project, fields):
    # Installed SDK 1.7.0 yields pages, despite describing an image generator.
    for page in project.search_all(limit=250, fields=fields, in_dataset=True):
        if not isinstance(page,list):
            raise ValueError('unexpected_search_page_shape')
        yield from page


def dataset_records(project, fields, expected):
    """Reconcile overlapping offset pages against the public Dataset count.

    The service caps pages at 250 and can repeat records on page boundaries.
    Never use a truncated page count as proof of acquiring the whole Dataset.
    """
    unique = {}
    for _ in range(3):
        for item in search_records(project, fields):
            unique[item['id']] = item
        if len(unique) > expected:
            raise ValueError('source_membership_count_changed')
        if len(unique) == expected:
            return sorted(unique.values(),key=lambda item:item['id'])
    raise ValueError('incomplete_source_enumeration')


def snapshot(project_id, root, key):
    import requests
    from roboflow.adapters import rfapi
    from roboflow.config import API_URL
    from roboflow.core.project import Project
    info = rfapi.get_project(key, *project_id.split('/'))
    if not info['project'].get('public') or info.get('versions'):
        raise ValueError('snapshot_only_for_public_projects_without_versions')
    if info['project'].get('type') not in ('object-detection','instance-segmentation'):
        raise ValueError('unsupported_snapshot_task')
    project = Project(key, info['project'])
    expected = info['project']['images']
    records = dataset_records(project, ['id','name','split'], expected)
    if not records or len({r['id'] for r in records}) != len(records):
        raise ValueError('empty_or_duplicate_source_image_ids')
    destination = root / 'raw' / project_id / 'unversioned-snapshot'
    destination.mkdir(parents=True, exist_ok=True)
    export = destination / 'export'
    export.mkdir(exist_ok=True)
    (destination/'native').mkdir(exist_ok=True)
    local=threading.local()
    def fetch(item):
        result = None
        cached=destination/'native'/(item['id']+'.json')
        if cached.exists():
            value=json.loads(cached.read_text())
            path=export/value['local_file']
            if path.exists() and sha(path)==value['image_sha256']:
                return value
        if not hasattr(local,'session'):
            local.session=requests.Session()
        for attempt in range(3):
            try:
                response = local.session.get(API_URL+'/'+project_id+'/images/'+item['id'], params={'api_key':key},timeout=(30,60))
                response.raise_for_status()
                record = response.json()['image']
                original = record['urls']['original']
                if isinstance(original, dict):
                    original = original['url']
                target = next((p for p in export.glob(item['id']+'.*') if p.suffix != '.partial'), export / (item['id'] + '.image'))
                if not target.exists():
                    with local.session.get(original, stream=True, timeout=(30,60)) as media:
                        media.raise_for_status()
                        temp = target.with_suffix('.partial')
                        with temp.open('wb') as f:
                            for chunk in media.iter_content(1024*1024):
                                f.write(chunk)
                        temp.replace(target)
                from PIL import Image
                with Image.open(target) as image:
                    image.load()
                    suffix = {'JPEG':'.jpg','PNG':'.png','WEBP':'.webp'}.get(image.format)
                    image_width,image_height=image.size
                if not suffix:
                    raise ValueError('unsupported_snapshot_image_format')
                if target.suffix != suffix:
                    final = target.with_suffix(suffix)
                    target.replace(final)
                    target = final
                # Preserve original annotation values and IDs, excluding media URLs,
                # embeddings and unrelated account information.
                result = {k:record.get(k) for k in ('id','name','annotation','split','created')}
                result['annotation_missing']=result.get('annotation') is None
                annotation=result.get('annotation') or {}
                if isinstance(annotation,str):annotation=json.loads(annotation)
                result['annotation'] = {k:v for k,v in annotation.items() if k != 'key'}
                result['annotation'].setdefault('width',image_width)
                result['annotation'].setdefault('height',image_height)
                result.update(local_file=target.name,image_sha256=sha(target))
                write_json(cached,result)
                return result
            except Exception as error:
                if attempt == 2:
                    raise RuntimeError(error_kind(error)) from None
                time.sleep(2**attempt)
        return result
    with ThreadPoolExecutor(max_workers=2) as pool:
        originals = []
        for index, original in enumerate(pool.map(fetch, records), 1):
            originals.append(original)
            if index % 50 == 0:
                print(json.dumps(dict(project=project_id, snapshot_images=index)),flush=True)
    # Detect changed membership rather than labelling a moving source immutable.
    after = dataset_records(project, ['id'], expected)
    if {r['id'] for r in records} != {r['id'] for r in after}:
        raise ValueError('source_membership_changed_during_snapshot')
    categories, annotations, images = {}, [], []
    with (destination/'native-annotations.jsonl').open('w') as f:
        for index, record in enumerate(originals):
            f.write(json.dumps(record,ensure_ascii=False)+'\n')
            boxes,width,height=boxes_to_coco(record,index,categories,len(annotations))
            annotations.extend(boxes)
            images.append(dict(id=index,file_name=record['local_file'],width=width,height=height,
                               original_image_id=record['id'],original_split=record.get('split'),
                               native_annotation_missing=record['annotation_missing']))
    write_json(export/'_annotations.coco.json',dict(images=images,annotations=annotations,
        categories=[dict(id=index,name=name) for name,index in categories.items()]))
    metadata = {k:info['project'].get(k) for k in ('id','name','type','classes','images','license','public')}
    metadata.update(version=None, snapshot=True, acquired_images=len(images),
        search_scope='in_dataset=true', expected_images=info['project']['images'],
        source_url='https://universe.roboflow.com/'+project_id, captured_at=datetime.now(timezone.utc).isoformat(),
        version_policy_exception='No released public version; read-only original-image snapshot')
    write_json(destination/'metadata.json',metadata)
    archive = destination/'local-preservation.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_STORED) as z:
        # Previously cached, unassigned uploads remain on disk, but are not
        # represented as public Dataset members in the preservation archive.
        for name in ('metadata.json','native-annotations.jsonl','export/_annotations.coco.json'):
            z.write(destination/name,name)
        for record in originals:
            for path in (export/record['local_file'], destination/'native'/(record['id']+'.json')):
                z.write(path,path.relative_to(destination))
    return dict(project=project_id,status='downloaded_snapshot',version='unversioned-snapshot',
        raw_path=str(destination.resolve()),metadata=metadata, images=len(images),
        archive_sha256=sha(archive),publisher_archive=False, original_version_available=False,
        native_annotation_sha256=sha(destination/'native-annotations.jsonl'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true',help='Queue official exports, without media downloads')
    p.add_argument('--method',choices=['bulk','native'],default='bulk')
    a=p.parse_args()
    import requests
    original_request=requests.sessions.Session.request
    def bounded_request(self,method,url,**kwargs):
        kwargs.setdefault('timeout',(30,60))
        return original_request(self,method,url,**kwargs)
    requests.sessions.Session.request=bounded_request
    from roboflow.config import load_roboflow_api_key
    key=os.environ.get('ROBOFLOW_API_KEY') or load_roboflow_api_key()
    report=json.loads((a.root/'download-report.json').read_text())
    if not a.prepare_only and report['completed_projects'] != report['expected_projects']:
        p.error('wait for the two-worker version downloader to finish first')
    previous_path=a.root/'snapshot-report.json'
    previous=json.loads(previous_path.read_text())['projects'] if previous_path.exists() else []
    results={r['project']:r for r in previous}
    for row in report['projects']:
        if row['status']=='downloaded':
            continue
        try:
            if a.prepare_only:
                prepare_export(row['project'],a.root.resolve(),key)
                print(json.dumps(dict(project=row['project'],status='snapshot_export_queued')),flush=True)
                continue
            prior=results.get(row['project'],{})
            if prior.get('status')=='downloaded_snapshot':
                archive=Path(prior['raw_path'])/('original.zip' if prior.get('publisher_archive') else 'local-preservation.zip')
                if not archive.exists() or sha(archive)!=prior['archive_sha256']:
                    raise ValueError('preserved_snapshot_archive_changed')
                result=prior
            else:
                result=(bulk_snapshot if a.method=='bulk' else snapshot)(row['project'],a.root.resolve(),key)
        except Exception as error:
            result=dict(project=row['project'],status='failed',reason=error_kind(error))
            if isinstance(error,ValueError) and str(error) in {
                'source_membership_count_changed','incomplete_source_enumeration',
                'source_membership_changed_during_snapshot','missing_native_dimensions',
                'empty_or_duplicate_source_image_ids'}:
                result['diagnostic']=str(error)
        results[row['project']]=result
        with (a.root/'snapshot-attempts.jsonl').open('a') as history:
            history.write(json.dumps(dict(at=datetime.now(timezone.utc).isoformat(),method=a.method,**result))+'\n')
        print(json.dumps({k:result.get(k) for k in ('project','status','images','reason')}),flush=True)
        values=[results[k] for k in sorted(results)]
        write_json(a.root/'snapshot-report.json',dict(projects=values,
            versioned=report['downloaded'], snapshots=sum(r['status']=='downloaded_snapshot' for r in values),
            all_sources_acquired=report['downloaded']+sum(r['status']=='downloaded_snapshot' for r in values)==22,
            all_latest_public_versions_acquired=report['complete']))


if __name__=='__main__':main()
