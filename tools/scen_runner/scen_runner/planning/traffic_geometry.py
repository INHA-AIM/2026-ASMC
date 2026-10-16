"""Link routing and source clearance; no GUI or runner lifecycle dependencies."""
import heapq
import json
import math
from functools import lru_cache
from pathlib import Path
from types import MappingProxyType
import numpy as np


def _immutable(value):
    if isinstance(value,dict):
        return MappingProxyType({k:_immutable(v) for k,v in value.items()})
    if isinstance(value,list):
        return tuple(_immutable(v) for v in value)
    return value


@lru_cache(maxsize=4)
def _read_links(path,signature):
    return _immutable({link['idx']:link for link in json.loads(Path(path).read_text())})


def load_links(path):
    """Share immutable geometry; reload when the underlying map file changes."""
    path=Path(path).resolve();stat=path.stat()
    signature=(stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns,stat.st_ctime_ns)
    return _read_links(str(path),signature)


def route_between(links, start, end):
    outgoing = {}
    for lid, link in links.items():
        outgoing.setdefault(link['from_node_idx'], []).append(lid)
    queue, seen = [(0., start, [start])], set()
    while queue:
        distance, lid, path = heapq.heappop(queue)
        if lid in seen:
            continue
        seen.add(lid)
        if lid == end:
            return path
        for next_id in outgoing.get(links[lid]['to_node_idx'], []):
            points = np.asarray(links[next_id]['points'])
            length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
            heapq.heappush(queue, (distance+length, next_id, path+[next_id]))
    raise ValueError(f'연결 경로 없음: {start} → {end}')


def spawn_clear(cfg, model, states):
    pose, dims = cfg['spawn_pose'], cfg['models'][model]
    radius = math.hypot(dims['length'], dims['width'])/2
    for state in states:
        if state.actor_info.object_type != 1:
            continue
        box = state.bounding_box
        valid = box.extent.x > 0 and box.extent.y > 0
        p = box.location if valid else state.transform.location
        other_radius = math.hypot(box.extent.x, box.extent.y) if valid else 4.
        dx, dy = p.x-pose['x'], p.y-pose['y']
        vx, vy = state.velocity.x/3.6, state.velocity.y/3.6
        vv = vx*vx+vy*vy
        t = max(0., min(cfg['ttc_s'], -(dx*vx+dy*vy)/vv)) if vv else 0.
        if math.hypot(dx+vx*t, dy+vy*t) < radius+other_radius+cfg['safety_distance_m']:
            return False
    return True
