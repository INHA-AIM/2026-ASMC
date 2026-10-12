#!/usr/bin/env python3
"""Download pinned public dataset versions using the official Roboflow SDK API.

Only read/export/download endpoints are used. No upload, training or version
generation. Signed URLs and SDK exception bodies must never enter reports.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import stat
import time
import zipfile


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temp.replace(path)


def latest_version(versions):
    available = [v for v in versions if not v.get('generating') and int(v.get('images', 0)) > 0]
    if not available:
        raise ValueError('no_completed_public_version')
    return max(available, key=lambda v: int(str(v['id']).rstrip('/').split('/')[-1]))


def extract_checked(archive, destination):
    destination = Path(destination).resolve()
    with zipfile.ZipFile(archive) as z:
        entries = z.infolist()
        if sum(e.file_size for e in entries) > shutil.disk_usage(destination.parent).free:
            raise ValueError('insufficient_disk')
        for e in entries:
            target = (destination / e.filename).resolve()
            try:
                target.relative_to(destination)
            except ValueError:
                raise ValueError('unsafe_zip_entry') from None
            if stat.S_ISLNK(e.external_attr >> 16):
                raise ValueError('unsafe_zip_entry')
        z.extractall(destination)


def error_kind(error):
    """Inspect transient error text in memory, emit a fixed category only."""
    message = str(error).lower()
    if any(s in message for s in ('401', 'api key', 'authentication', 'unauthorized')):
        return 'authentication'
    if any(s in message for s in ('403', 'permission', 'forbidden', 'private')):
        return 'access_denied'
    if any(s in message for s in ('404', 'does not exist', 'not found')):
        return 'not_found_or_inaccessible'
    if 'no_completed_public_version' in message:
        return 'no_completed_public_version'
    if 'not_public' in message:
        return 'not_public'
    if 'unsupported_task' in message:
        return 'unsupported_task'
    if any(s in message for s in ('timeout', 'timed out', '429', '500', '502', '503', '504')):
        return 'transient_network_or_server'
    return type(error).__name__


def fetch_project(project_id, root, key, attempts=3, metadata_only=False):
    import requests
    from roboflow.adapters import rfapi
    workspace, project = project_id.split('/')
    state = dict(project=project_id, url='https://universe.roboflow.com/' + project_id,
                 status='pending', attempts=[], checked_at=datetime.now(timezone.utc).isoformat())
    for attempt in range(1, attempts + 1):
        try:
            info = rfapi.get_project(key, workspace, project)
            p = info['project']
            state['project_inventory'] = {k:p.get(k) for k in ('id','name','type','public','license','classes','images')}
            state['public_version_inventory'] = [{k:v.get(k) for k in ('id','images','generating')} for v in info.get('versions',[])]
            if not p.get('public'):
                raise ValueError('not_public')
            versions = info.get('versions', [])
            selected = latest_version(versions)
            number = int(str(selected['id']).rstrip('/').split('/')[-1])
            # Pin the first observed version, including across interrupted resumes.
            pin_path = root / 'pins' / workspace / (project + '.json')
            pin = json.loads(pin_path.read_text()) if pin_path.exists() else {'version': number}
            number = pin['version']
            version_info = rfapi.get_version(key, workspace, project, number)
            version = version_info['version']
            task = p.get('type')
            if task not in ('object-detection', 'instance-segmentation', 'semantic-segmentation'):
                raise ValueError('unsupported_task')
            fmt = 'png-mask-semantic' if task == 'semantic-segmentation' else 'coco'
            metadata = {k: p.get(k) for k in ('id', 'name', 'type', 'classes', 'images', 'license', 'public', 'created', 'updated')}
            metadata.update(version=number, format=fmt, source_url=state['url'],
                selected_version={k: version.get(k) for k in ('id', 'name', 'images', 'created', 'preprocessing', 'augmentation', 'splits')},
                public_versions=[{k: v.get(k) for k in ('id', 'name', 'images', 'created', 'generating')} for v in versions],
                sdk_version=importlib.metadata.version('roboflow'))
            write_json(pin_path, pin)
            destination = root / 'raw' / workspace / project / str(number)
            destination.mkdir(parents=True, exist_ok=True)
            metadata_path=destination/'metadata.json'
            if metadata_path.exists():
                metadata=json.loads(metadata_path.read_text())
                if metadata.get('version')!=number or metadata.get('source_url')!=state['url']:
                    raise ValueError('preserved_metadata_mismatch')
            else:
                write_json(metadata_path, metadata)
            state.update(version=number, task=task, format=fmt, metadata=metadata,
                         raw_path=str(destination), status='metadata_ready')
            if metadata_only:
                break
            archive = destination / 'original.zip'
            receipt = destination / 'receipt.json'
            if receipt.exists():
                previous = json.loads(receipt.read_text())
                if not archive.exists() or sha(archive)!=previous['archive_sha256']:
                    raise ValueError('preserved_archive_changed')
                if not (destination/'export').is_dir():
                    extract_checked(archive,destination/'export')
                state.update(status='downloaded', **previous)
                break
            export = rfapi.get_version_export(key, workspace, project, number, fmt)
            deadline = time.monotonic() + 600
            while export.get('ready') is False:
                if time.monotonic() > deadline:
                    raise TimeoutError('export_timeout')
                time.sleep(5)
                export = rfapi.get_version_export(key, workspace, project, number, fmt)
            link = export['export']['link']  # signed URL lives only in memory
            temporary = destination / 'original.zip.partial'
            with requests.get(link, stream=True, timeout=(30, 90)) as response:
                response.raise_for_status()
                with temporary.open('wb') as f:
                    for chunk in response.iter_content(1024 * 1024):
                        if chunk:
                            f.write(chunk)
            if not zipfile.is_zipfile(temporary):
                raise ValueError('download_is_not_zip')
            temporary.replace(archive)
            extract_checked(archive, destination / 'export')
            result = dict(archive_sha256=sha(archive), archive_bytes=archive.stat().st_size,
                          downloaded_at=datetime.now(timezone.utc).isoformat())
            write_json(receipt, result)
            state.update(status='downloaded', **result)
            break
        except Exception as error:
            state['attempts'].append(dict(number=attempt, reason=error_kind(error)))
            state['status'] = 'failed'
            if attempt < attempts:
                time.sleep(min(2 ** attempt, 8))
    write_json(root / 'status' / workspace / (project + '.json'), state)
    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sources', type=Path, required=True)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--metadata-only', action='store_true')
    a = p.parse_args()
    config = json.loads(a.sources.read_text())
    projects = config['projects']
    if len(projects) != 22 or len(set(projects)) != 22:
        p.error('exactly 22 unique approved projects are required')
    import requests
    from roboflow.config import load_roboflow_api_key
    # SDK requests have no default timeout. Bound all requests in this process.
    original_request = requests.sessions.Session.request
    def bounded_request(self, method, url, **kwargs):
        kwargs.setdefault('timeout', (30, 60))
        return original_request(self, method, url, **kwargs)
    requests.sessions.Session.request = bounded_request
    try:
        key = os.environ.get('ROBOFLOW_API_KEY') or load_roboflow_api_key()
    except Exception:
        key = None
    if not key:
        p.error('local Roboflow login or ROBOFLOW_API_KEY is required; do not paste secrets into logs')
    rows = []
    with ThreadPoolExecutor(max_workers=min(config.get('workers', 2), 2)) as pool:
        jobs = [pool.submit(fetch_project, project, a.root.resolve(), key,
                            min(config.get('max_attempts', 3), 3), a.metadata_only) for project in projects]
        for job in as_completed(jobs):
            row = job.result()
            rows.append(row)
            print(json.dumps({k: row.get(k) for k in ('project', 'version', 'status')}) , flush=True)
            write_json(a.root / 'download-report.json', dict(
                expected_projects=22, completed_projects=len(rows),
                downloaded=sum(r['status'] == 'downloaded' for r in rows),
                complete=len(rows) == 22 and all(r['status'] == 'downloaded' for r in rows),
                sdk_version=importlib.metadata.version('roboflow'),
                sources_sha256=sha(a.sources), projects=sorted(rows, key=lambda r: r['project'])))


if __name__ == '__main__':
    main()
