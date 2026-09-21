"""H5 출력.

주의: 이 저장소에는 좌표계가 다른 MGeo 맵이 둘 있다 (behavior_planner 판본과
R_KR_PG_KATRI 판본, local_origin 차이 약 (-135, -1509) m). H5 만 보고 어느 맵인지
분간이 안 되면 사고가 나므로 map_id 와 local_origin_in_global 을 항상 기록한다.
"""
from __future__ import annotations

import datetime
import json
import subprocess
from pathlib import Path
from typing import Any, Dict, Mapping

import h5py
import numpy as np

from .raster import RasterGrid


def _git_sha(cwd: Path) -> str:
    try:
        out = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=str(cwd), capture_output=True, text=True, timeout=5,
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return 'unknown'


def write_h5(
    output_path: Path,
    layers: Mapping[str, np.ndarray],
    grid: RasterGrid,
    *,
    map_id: str,
    local_origin_in_global: Any = (),
    source_files: Mapping[str, str] = (),
    layer_params: Mapping[str, Any] = (),
    stats: Mapping[str, Any] = (),
    stopline_features: Any = (),
    observed_lane_features: Any = (),
    links: Any = (),
    compression_level: int = 9,
    repo_root: Path = None,
) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(output_path, 'w') as hf:
        hf.attrs['pixels_per_meter'] = float(grid.pixels_per_meter)
        hf.attrs['world_offset_in_meters'] = grid.world_offset.astype(np.float32)
        hf.attrs['width_in_meters'] = float(grid.width_m)
        hf.attrs['height_in_meters'] = float(grid.height_m)
        hf.attrs['width_in_pixels'] = int(grid.width_px)
        hf.attrs['height_in_pixels'] = int(grid.height_px)

        hf.attrs['map_id'] = str(map_id)
        hf.attrs['local_origin_in_global'] = np.asarray(
            list(local_origin_in_global), dtype=np.float64
        )
        hf.attrs['layers'] = json.dumps(sorted(layers.keys()))
        hf.attrs['source_files'] = json.dumps(dict(source_files), ensure_ascii=False)
        hf.attrs['layer_params'] = json.dumps(dict(layer_params), ensure_ascii=False)
        hf.attrs['stats'] = json.dumps(dict(stats), ensure_ascii=False)
        hf.attrs['baked_at'] = datetime.datetime.now().astimezone().isoformat()
        hf.attrs['git_sha'] = _git_sha(repo_root or output_path.parent)

        # 래스터만으로는 '가장 가까운 정지선까지 몇 m' 를 답할 수 없다.
        # 정지선의 위치·방향을 벡터로 함께 저장해 소비 측이 바로 쓰게 한다.
        feats = list(stopline_features or ())
        if feats:
            hf.create_dataset(
                'stopline_points',
                data=np.array([[f['x'], f['y']] for f in feats], dtype=np.float32))
            hf.create_dataset(
                'stopline_dirs',
                data=np.array([[f['dir_x'], f['dir_y']] for f in feats], dtype=np.float32))
            hf.create_dataset(
                'stopline_headings',
                data=np.array([[f['heading_x'], f['heading_y']] for f in feats],
                              dtype=np.float32))
            hf.create_dataset(
                'stopline_lengths',
                data=np.array([f['length_m'] for f in feats], dtype=np.float32))
            hf.attrs['stopline_ids'] = json.dumps([f['idx'] for f in feats])
            hf.attrs['stopline_traffic_lights'] = json.dumps(
                [f['traffic_light_id'] for f in feats])

        # 객체-실측 차선 거리를 래스터 오차 없이 계산할 수 있도록 보정된 중심선도 저장한다.
        observed = list(observed_lane_features or ())
        if observed:
            pts, offsets = [], [0]
            for feature in observed:
                p_ = np.asarray(feature['points'], dtype=np.float32).reshape(-1, 2)
                pts.append(p_)
                offsets.append(offsets[-1] + p_.shape[0])
            hf.create_dataset('observed_lane_points', data=np.vstack(pts),
                              compression='gzip', compression_opts=4)
            hf.create_dataset('observed_lane_offsets',
                              data=np.asarray(offsets, dtype=np.int32))
            hf.attrs['observed_lane_sides'] = json.dumps([f['side'] for f in observed])
            hf.attrs['observed_lane_colors'] = json.dumps([f['color'] for f in observed])
            hf.attrs['observed_lane_sources'] = json.dumps([f['source'] for f in observed])

        # 어떤 링크가 어느 레이어에 들어갔는지 벡터로 남긴다. 주행하면서 '지금 밟고
        # 있는 링크' 를 알아야 제외 목록을 만들 수 있다.
        link_list = list(links or ())
        if link_list:
            pts, offsets = [], [0]
            for l in link_list:
                p_ = np.asarray(l['points'], dtype=np.float32).reshape(-1, 2)
                pts.append(p_)
                offsets.append(offsets[-1] + p_.shape[0])
            hf.create_dataset('link_points', data=np.vstack(pts),
                              compression='gzip', compression_opts=4)
            hf.create_dataset('link_offsets', data=np.array(offsets, dtype=np.int32))
            hf.attrs['link_ids'] = json.dumps([l['idx'] for l in link_list])
            hf.attrs['link_in_road'] = json.dumps([bool(l['in_road']) for l in link_list])
            hf.attrs['link_in_lane'] = json.dumps([bool(l['in_lane']) for l in link_list])

        for name, img in layers.items():
            hf.create_dataset(
                name,
                data=np.ascontiguousarray(img, dtype=np.uint8),
                compression='gzip',
                compression_opts=compression_level,
            )


def read_h5_meta(path: Path) -> Dict[str, Any]:
    """굽고 나서 확인할 때 쓰는 최소 리더."""
    with h5py.File(path, 'r') as hf:
        meta = {k: hf.attrs[k] for k in hf.attrs}
        meta['datasets'] = {k: (hf[k].shape, str(hf[k].dtype)) for k in hf}
    return meta
