"""직접 수집 차선 override 테스트."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from mapbake.observed_lane import (
    apply_observed_lane_overrides,
    exclude_road_between_observed_boundaries,
    restore_trimmed_road_on_links,
    trim_road_outside_observed_lanes,
)
from mapbake.raster import RasterGrid, draw_polyline


def _write_csv(path: Path) -> None:
    with open(path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        for i, x in enumerate(np.linspace(2.0, 16.0, 29)):
            writer.writerow([0.1 * i, 'LEFT', x, 5.4, 'WHITE'])


def test_replace_removes_near_prior_but_preserves_adjacent_lane(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 5.0, 100, 100)
    prior = grid.blank()
    draw_polyline(prior, grid, np.array([[2.0, 5.0], [16.0, 5.0]]), 2)
    # 같은 경계를 공유하지 않는 3 m 떨어진 인접 차선은 남아야 한다.
    draw_polyline(prior, grid, np.array([[2.0, 8.0], [16.0, 8.0]]), 2)
    csv_path = tmp_path / 'observed.csv'
    _write_csv(csv_path)

    result = apply_observed_lane_overrides(
        prior,
        grid,
        [{
            'path': str(csv_path),
            'action': 'replace',
            # CSV의 잘못된 색보다 설정에 명시한 중앙선 규칙을 우선한다.
            'side_colors': {'LEFT': 'YELLOW'},
        }],
        replace_radius_m=0.8,
    )

    near_old = grid.to_pixel(np.array([[8.0, 5.0]]))[0]
    observed = grid.to_pixel(np.array([[8.0, 5.4]]))[0]
    adjacent = grid.to_pixel(np.array([[8.0, 8.0]]))[0]
    assert result.image[near_old[1], near_old[0]] == 0
    assert result.observed_yellow[observed[1], observed[0]] == 255
    assert result.image[observed[1], observed[0]] == 255
    assert result.image[adjacent[1], adjacent[0]] == 255
    assert result.stats['thickness_px'] == 2
    assert result.stats['effective_width_m'] == 0.4


def test_add_mode_does_not_erase_prior(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 5.0, 100, 100)
    prior = grid.blank()
    draw_polyline(prior, grid, np.array([[2.0, 5.0], [16.0, 5.0]]), 2)
    csv_path = tmp_path / 'observed.csv'
    _write_csv(csv_path)

    result = apply_observed_lane_overrides(
        prior,
        grid,
        [{'path': str(csv_path), 'action': 'add',
          'side_colors': {'LEFT': 'YELLOW'}}],
    )

    old = grid.to_pixel(np.array([[8.0, 5.0]]))[0]
    assert result.erase_mask.sum() == 0
    assert result.image[old[1], old[0]] == 255


def test_source_time_gap_override_keeps_stationary_pause_connected(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 200, 100)
    prior = grid.blank()
    csv_path = tmp_path / 'paused.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        writer.writerow([0.0, 'RIGHT', 2.0, 5.0, 'WHITE'])
        writer.writerow([0.1, 'RIGHT', 3.0, 5.0, 'WHITE'])
        writer.writerow([0.9, 'RIGHT', 3.1, 5.0, 'WHITE'])
        writer.writerow([1.0, 'RIGHT', 4.0, 5.0, 'WHITE'])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path), 'max_time_gap_sec': 1.0}],
        max_time_gap_sec=0.5,
    )

    assert result.stats['segments'] == 1
    assert result.stats['sources'][0]['max_time_gap_sec'] == 1.0


def test_observed_only_removes_every_prior_lane(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 5.0, 100, 100)
    prior = grid.blank()
    draw_polyline(prior, grid, np.array([[2.0, 5.0], [16.0, 5.0]]), 2)
    draw_polyline(prior, grid, np.array([[2.0, 8.0], [16.0, 8.0]]), 2)
    csv_path = tmp_path / 'observed.csv'
    _write_csv(csv_path)

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path), 'side_colors': {'LEFT': 'YELLOW'}}],
        keep_prior=False,
    )

    adjacent = grid.to_pixel(np.array([[8.0, 8.0]]))[0]
    observed = grid.to_pixel(np.array([[8.0, 5.4]]))[0]
    assert result.image[adjacent[1], adjacent[0]] == 0
    assert result.image[observed[1], observed[0]] == 255
    assert np.array_equal(result.image, result.observed)
    assert np.array_equal(result.erase_mask, prior)


def test_inward_offset_and_observed_road_corridor(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 200, 200)
    prior = grid.blank()
    csv_path = tmp_path / 'both.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        for i, x in enumerate(np.linspace(2.0, 16.0, 29)):
            writer.writerow([0.1 * i, 'LEFT', x, 8.0, 'YELLOW'])
            writer.writerow([0.1 * i, 'RIGHT', x, 4.0, 'WHITE'])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path)}],
        inward_offset_m=0.3,
        min_thickness_px=1,
        keep_prior=False,
        fill_road_between_sides=True,
    )

    # 진행 방향이 +x일 때 LEFT는 오른쪽(-y), RIGHT는 왼쪽(+y)으로 이동한다.
    left = grid.to_pixel(np.array([[8.0, 7.7]]))[0]
    right = grid.to_pixel(np.array([[8.0, 4.3]]))[0]
    middle = grid.to_pixel(np.array([[8.0, 6.0]]))[0]
    assert result.observed_yellow[left[1], left[0]] == 255
    assert result.observed_white[right[1], right[0]] == 255
    assert result.observed_road[middle[1], middle[0]] == 255
    assert result.stats['inward_offset_m'] == 0.3
    assert result.stats['keep_prior'] is False


def test_merged_csv_groups_do_not_create_road_between_datasets(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 300)
    prior = grid.blank()
    csv_path = tmp_path / 'merged.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color',
            'source_dataset', 'source_file',
        ])
        for i, x in enumerate(np.linspace(2.0, 8.0, 13)):
            writer.writerow([0.1 * i, 'LEFT', x, 8.0, 'YELLOW', '1', 'paired.csv'])
            writer.writerow([0.1 * i, 'RIGHT', x, 4.0, 'WHITE', '1', 'paired.csv'])
        for i, x in enumerate(np.linspace(18.0, 24.0, 13)):
            # 별도 단일 경계 수집이며 LEFT지만 실제 색은 WHITE다.
            writer.writerow([0.1 * i, 'LEFT', x, 20.0, 'WHITE', '2', 'single.csv'])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path), 'group_column': 'source_dataset'}],
        keep_prior=False,
        fill_road_between_sides=True,
    )

    paired_middle = grid.to_pixel(np.array([[5.0, 6.0]]))[0]
    false_bridge = grid.to_pixel(np.array([[14.0, 14.0]]))[0]
    single_white = grid.to_pixel(np.array([[21.0, 20.0]]))[0]
    assert result.observed_road[paired_middle[1], paired_middle[0]] == 255
    assert result.observed_road[false_bridge[1], false_bridge[0]] == 0
    assert result.observed_white[single_white[1], single_white[0]] == 255
    assert result.stats['sources'][0]['groups'] == 2
    assert result.stats['sources'][0]['road_filled_groups'] == 1


def test_outer_road_trim_preserves_lane_when_link_exists_on_both_sides():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    road = grid.blank()
    # 두 차로 중심선 y=4, 7.5를 합쳐 만든 주행가능영역.
    draw_polyline(road, grid, np.array([[2.0, 4.0], [20.0, 4.0]]), 35)
    draw_polyline(road, grid, np.array([[2.0, 7.5], [20.0, 7.5]]), 35)
    lane = grid.blank()
    boundary = np.array([[3.0, 5.75], [19.0, 5.75]])
    draw_polyline(lane, grid, boundary, 1)

    result = trim_road_outside_observed_lanes(
        road, lane, grid,
        [{'points': boundary}],
        [np.array([[2.0, 4.0], [20.0, 4.0]]),
         np.array([[2.0, 7.5], [20.0, 7.5]])],
    )

    second_lane = grid.to_pixel(np.array([[10.0, 7.0]]))[0]
    assert result.image[second_lane[1], second_lane[0]] == 255
    assert result.image[
        grid.to_pixel(np.array([[10.0, 5.75]]))[0][1],
        grid.to_pixel(np.array([[10.0, 5.75]]))[0][0],
    ] == 255
    assert result.stats['removed_px'] == 0
    assert result.stats['excluded_lane_px'] == 0
    assert result.stats['internal_points_preserved'] > 0


def test_restore_trimmed_road_on_selected_link_only():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 240, 160)
    source = grid.blank()
    center = np.array([[2.0, 5.0], [20.0, 5.0]])
    draw_polyline(source, grid, center, 36)
    trimmed = source.copy()
    # 외곽 오판으로 차로 내부 일부가 길게 잘린 상황.
    cut = grid.blank()
    draw_polyline(cut, grid, np.array([[5.0, 5.8], [15.0, 5.8]]), 8)
    trimmed[cut > 0] = 0

    restored, mask, stats = restore_trimmed_road_on_links(
        trimmed, source, grid,
        [{'idx': 'test-link', 'points': center, 'width_m': 3.6}],
    )

    inside = grid.to_pixel(np.array([[10.0, 5.8]]))[0]
    outside = grid.to_pixel(np.array([[10.0, 7.2]]))[0]
    assert restored[inside[1], inside[0]] == 255
    assert mask[inside[1], inside[0]] == 255
    assert restored[outside[1], outside[0]] == 0
    assert stats['links'] == ['test-link']
    assert stats['restored_px'] > 0


def test_exclude_road_only_between_overlapping_observed_boundaries():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 220, 220)
    road = np.full((220, 220), 255, dtype=np.uint8)
    long_boundary = np.array([[8.0, 2.0], [8.0, 18.0]])
    short_boundary = np.array([[12.0, 6.0], [12.0, 14.0]])
    features = [
        {'points': long_boundary, 'source': '/data/long.csv',
         'side': 'LEFT', 'color': 'WHITE'},
        {'points': short_boundary, 'source': '/data/short.csv',
         'side': 'RIGHT', 'color': 'WHITE'},
    ]

    result, mask, stats = exclude_road_between_observed_boundaries(
        road, grid, features,
        [{'name': 'island', 'boundaries': [
            {'source': 'long.csv', 'side': 'LEFT'},
            {'source': 'short.csv', 'side': 'RIGHT'},
        ]}],
    )

    inside = grid.to_pixel(np.array([[10.0, 10.0]]))[0]
    before_overlap = grid.to_pixel(np.array([[10.0, 4.0]]))[0]
    assert result[inside[1], inside[0]] == 0
    assert mask[inside[1], inside[0]] == 255
    assert result[before_overlap[1], before_overlap[0]] == 255
    assert stats['removed_px'] > 0


def test_outer_road_trim_removes_only_side_without_adjacent_link():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    road = grid.blank()
    # 실제 링크 중심은 y=4 하나뿐인데 road가 외곽선(y=5.75) 너머로 0.75 m 돌출됐다.
    draw_polyline(road, grid, np.array([[2.0, 4.0], [20.0, 4.0]]), 50)
    lane = grid.blank()
    boundary = np.array([[3.0, 5.75], [19.0, 5.75]])
    draw_polyline(lane, grid, boundary, 1)

    result = trim_road_outside_observed_lanes(
        road, lane, grid,
        [{'points': boundary}],
        [np.array([[2.0, 4.0], [20.0, 4.0]])],
    )

    inside = grid.to_pixel(np.array([[10.0, 5.3]]))[0]
    outside = grid.to_pixel(np.array([[10.0, 6.2]]))[0]
    line = grid.to_pixel(np.array([[10.0, 5.75]]))[0]
    assert result.image[inside[1], inside[0]] == 255
    assert result.image[outside[1], outside[0]] == 0
    assert result.image[line[1], line[0]] == 0
    assert result.excluded_lane_mask[line[1], line[0]] == 255
    assert result.stats['removed_px'] > 0
    assert result.stats['excluded_lane_px'] > 0
    assert result.stats['outer_runs'] == 1


def test_outer_trim_cannot_remove_existing_link_core():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    road = grid.blank()
    # 관측선 외측 절단 polygon이 링크 중심을 덮는 극단적인 합류부 상황을 만든다.
    draw_polyline(road, grid, np.array([[2.0, 4.0], [20.0, 4.0]]), 50)
    lane = grid.blank()
    boundary = np.array([[3.0, 3.0], [19.0, 3.0]])
    draw_polyline(lane, grid, boundary, 1)

    result = trim_road_outside_observed_lanes(
        road, lane, grid,
        [{'points': boundary}],
        [np.array([[2.0, 4.0], [20.0, 4.0]])],
        link_probe_offset_m=1.0,
        link_tolerance_m=0.6,
        boundary_search_m=2.0,
        cut_depth_m=2.0,
        protect_link_core_width_m=1.0,
    )

    core = grid.to_pixel(np.array([[10.0, 4.0]]))[0]
    line = grid.to_pixel(np.array([[10.0, 3.0]]))[0]
    assert result.image[core[1], core[0]] == 255
    assert result.image[line[1], line[0]] == 0


def test_lane_overlapping_link_core_is_reclassified_as_internal():
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    road = grid.blank()
    centerline = np.array([[2.0, 4.0], [20.0, 4.0]])
    draw_polyline(road, grid, centerline, 30)
    lane = grid.blank()
    # 합류부에서 실측선이 유효 링크 중심 0.3 m 옆까지 접근한 상황이다.
    boundary = np.array([[3.0, 3.7], [19.0, 3.7]])
    draw_polyline(lane, grid, boundary, 1)

    result = trim_road_outside_observed_lanes(
        road, lane, grid,
        [{'points': boundary}],
        [centerline],
        link_probe_offset_m=0.3,
        link_tolerance_m=0.2,
        boundary_search_m=2.0,
        cut_depth_m=2.0,
        protect_link_core_width_m=1.0,
    )

    point = grid.to_pixel(np.array([[10.0, 3.7]]))[0]
    assert result.image[point[1], point[0]] == 255
    assert result.excluded_lane_mask[point[1], point[0]] == 0
    assert result.stats['link_core_lane_reclassified_px'] > 0


def test_per_lane_point_gap_override_connects_only_selected_lane(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    prior = grid.blank()
    csv_path = tmp_path / 'short_gap.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        # 두 색 모두 1.8 m 점프한다. 설정한 LEFT_YELLOW만 하나의 선분이 돼야 한다.
        writer.writerows([
            [0.0, 'LEFT', 2.0, 8.0, 'YELLOW'],
            [0.1, 'LEFT', 3.0, 8.0, 'YELLOW'],
            [0.2, 'LEFT', 4.8, 8.0, 'YELLOW'],
            [0.3, 'LEFT', 5.8, 8.0, 'YELLOW'],
            [0.0, 'RIGHT', 2.0, 4.0, 'WHITE'],
            [0.1, 'RIGHT', 3.0, 4.0, 'WHITE'],
            [0.2, 'RIGHT', 4.8, 4.0, 'WHITE'],
            [0.3, 'RIGHT', 5.8, 4.0, 'WHITE'],
        ])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path),
          'max_point_gap_by_lane': {'LEFT_YELLOW': 2.0}}],
        max_point_gap_m=1.5,
        keep_prior=False,
    )

    yellow = [f for f in result.features if f['color'] == 'YELLOW']
    white = [f for f in result.features if f['color'] == 'WHITE']
    assert len(yellow) == 1
    assert len(white) == 2
    assert result.stats['sources'][0]['max_point_gap_by_lane'] == {
        'LEFT_YELLOW': 2.0}


def test_segment_bridge_requires_matching_heading(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 400, 200)
    prior = grid.blank()
    csv_path = tmp_path / 'aligned_gap.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        # 첫 두 선분은 같은 +x 방향이고, 세 번째는 -x로 되돌아가므로 연결 금지다.
        writer.writerows([
            [0.0, 'RIGHT', 2.0, 4.0, 'WHITE'],
            [0.1, 'RIGHT', 3.0, 4.0, 'WHITE'],
            [1.0, 'RIGHT', 4.8, 4.0, 'WHITE'],
            [1.1, 'RIGHT', 5.8, 4.0, 'WHITE'],
            [2.0, 'RIGHT', 4.2, 4.0, 'WHITE'],
            [2.1, 'RIGHT', 3.2, 4.0, 'WHITE'],
        ])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path),
          'bridge_segment_gaps_by_lane': {'RIGHT_WHITE': 2.0},
          'bridge_max_heading_deg': 20.0}],
        max_time_gap_sec=0.5,
        keep_prior=False,
    )

    assert len(result.features) == 2
    assert len(result.features[0]['points']) == 4
    assert result.stats['sources'][0]['bridged_segments'] == 1


def test_manual_bridge_is_offset_drawn_and_saved_as_vector(tmp_path):
    grid = RasterGrid(np.array([0.0, 0.0], dtype=np.float32), 10.0, 300, 200)
    prior = grid.blank()
    csv_path = tmp_path / 'manual_bridge.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'side', 'lane_map_x', 'lane_map_y', 'lane_color'])
        writer.writerows([
            [0.0, 'RIGHT', 2.0, 4.0, 'WHITE'],
            [0.1, 'RIGHT', 3.0, 4.0, 'WHITE'],
            [1.0, 'RIGHT', 5.0, 4.0, 'WHITE'],
            [1.1, 'RIGHT', 6.0, 4.0, 'WHITE'],
        ])

    result = apply_observed_lane_overrides(
        prior, grid,
        [{'path': str(csv_path),
          'manual_bridges': [{
              'side': 'RIGHT', 'color': 'WHITE',
              'points': [[3.0, 4.0], [5.0, 4.0]],
          }]}],
        inward_offset_m=0.2,
        max_point_gap_m=1.5,
        keep_prior=False,
    )

    bridge = result.features[-1]
    midpoint = grid.to_pixel(np.array([[4.0, 4.2]]))[0]
    assert bridge['source_group'] == 'manual_bridge'
    assert bridge['points'][:, 1] == pytest.approx([4.2, 4.2])
    assert result.observed_white[midpoint[1], midpoint[0]] == 255
    assert result.stats['sources'][0]['manual_bridges'] == 1
