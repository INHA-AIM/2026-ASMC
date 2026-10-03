#!/usr/bin/env python3
"""Observe pinned build derivatives without changing classification branches."""
from pathlib import Path
import hashlib
import sys


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('diagnostic anchor drift: ' + old[:100])
    return text.replace(old, new, 1)


def generate(repo, pp, lf):
    hpath = pp/'include/patchwork/patchworkpp.h'
    h = hpath.read_text()
    h = once(h, '  std::vector<HeightPatch> height_patches;', '''  std::vector<unsigned> point_reasons;
  void ReviewReason(const std::vector<PointXYZ>& cloud, unsigned reason) {
    for (const auto& p : cloud) if (p.idx >= 0 && std::size_t(p.idx) < point_reasons.size())
      point_reasons[p.idx] = (point_reasons[p.idx] & ~4096u) | reason;
  }
  std::vector<HeightPatch> height_patches;''')
    hpath.write_text(h)
    path = pp/'src/patchworkpp.cpp'; text = path.read_text()
    text = once(text, '  height_patches.clear();', '  point_reasons.assign(cloud_in.rows(), 4096u);\n  height_patches.clear();')
    text = once(text, '          addCloud(cloud_nonground_, zone[ring_idx][sector_idx]);',
                '          ReviewReason(zone[ring_idx][sector_idx], 1);\n          addCloud(cloud_nonground_, zone[ring_idx][sector_idx]);')
    changes = [
        ('if (!is_upright) {', 'ReviewReason(regionwise_ground_, 8);'),
        ('} else if (!is_near_zone) {', 'ReviewReason(regionwise_ground_, 64);'),
        ('} else if (!is_heading_outside) {', 'ReviewReason(regionwise_ground_, 16);'),
        ('} else if (is_not_elevated || is_flat) {', 'ReviewReason(regionwise_ground_, (is_not_elevated ? 128 : 0) | (is_flat ? 256 : 0));'),
        ('          candidates.emplace_back(concentric_idx,', '          ReviewReason(regionwise_ground_, 32);\n          candidates.emplace_back(concentric_idx,')]
    for old, addition in changes:
        text = once(text, old, addition if 'candidates.emplace' in old else old+'\n          '+addition)
    text = text.replace('addCloud(cloud_ground_, candidate.regionwise_ground);',
                        'ReviewReason(candidate.regionwise_ground, 512);\n        addCloud(cloud_ground_, candidate.regionwise_ground);')
    text = text.replace('addCloud(cloud_nonground_, candidate.regionwise_ground);',
                        'ReviewReason(candidate.regionwise_ground, 1024);\n        addCloud(cloud_nonground_, candidate.regionwise_ground);')
    text = once(text, '          if (abs(distance) < params_.th_dist_v) {',
                '          if (abs(distance) < params_.th_dist_v) {\n            point_reasons[point.idx] = (point_reasons[point.idx] & ~4096u) | 2;')
    text = once(text, '\n          non_ground_dst.push_back(point);',
                '\n          point_reasons[point.idx] = (point_reasons[point.idx] & ~4096u) | 4;\n          non_ground_dst.push_back(point);')
    path.write_text(text)

    path = lf.parent/'ground_live_linefit_include/ground_segmentation/ground_segmentation.h'
    h = path.read_text()
    h = once(h, '  std::vector<unsigned> fit_visits, assignment_visits;',
             '  std::vector<unsigned> fit_visits, assignment_visits, point_reasons;\n  std::vector<unsigned> fit_reason_counts;')
    path.write_text(h)
    text = lf.read_text()
    text = once(text, '  assignment_visits.assign(cloud.size(), 0);',
                '  assignment_visits.assign(cloud.size(), 0);\n  point_reasons.assign(cloud.size(), 64);\n  fit_reason_counts.assign(5, 0);')
    text = once(text, '  assignCluster(segmentation);', '''  for (const auto& s : segments_) for (int k=0;k<5;++k) fit_reason_counts[k] += s.review_rejections[k];
  assignCluster(segmentation);''')
    text = once(text, '    ++assignment_visits[i];', '    ++assignment_visits[i];\n    point_reasons[i]=32;')
    text = once(text, '      // Search neighboring segments.', '''      point_reasons[i] = (fit_visits[segment_index] ? 0 : 128) | (dist < 0 ? 4 : 0);
      const bool own_line = dist >= 0;
      // Search neighboring segments.''')
    text = once(text, '      if (dist < params_.max_dist_to_line && dist != -1) {', '''      point_reasons[i] |= dist < 0 ? 8 : (dist < params_.max_dist_to_line ? (own_line ? 1 : 2) : 16);
      if (dist < params_.max_dist_to_line && dist != -1) {''')
    lf.write_text(text)
    vendor = repo/'src/asmc_lidar/vendor/linefit'
    expected = {'include/ground_segmentation/segment.h': '266bd5543e27fb5e2d158ad4cf0536c11aa158f915c8b56aa46d24e667e13d48',
                'src/segment.cc': '5ca76f01a02cb105f59137d34e976e57d7de9547195a438eff258a461f67a88c'}
    for name in ('include/ground_segmentation/segment.h', 'src/segment.cc'):
        path = vendor/name; data = path.read_text()
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected[name]:
            raise ValueError('linefit segment source drift: '+name)
        if name.endswith('.h'):
            data = once(data, 'class Segment {\npublic:', 'class Segment {\npublic:\n  unsigned review_rejections[5] = {0,0,0,0,0};')
            target = lf.parent/'ground_live_linefit_include/ground_segmentation/segment.h'
        else:
            data = once(data, '          // Add line until previous point as ground.', '''          review_rejections[0] += error > max_error_;
          review_rejections[1] += std::fabs(cur_line.first) > max_slope_ || (current_line_points.size() > 2 && std::fabs(cur_line.first) < min_slope_);
          review_rejections[2] += is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_;
          // Add line until previous point as ground.''')
            data = once(data, '          current_line_points.clear();', '''          review_rejections[3] += cur_point.d - current_line_points.back().d >= long_threshold_;
          review_rejections[4] += std::fabs(current_line_points.back().z - cur_ground_height) >= max_start_height_;
          current_line_points.clear();''')
            target = lf.parent/'ground_live_linefit_segment.cc'
        target.write_text(data)


if __name__ == '__main__':
    generate(*(Path(p) for p in sys.argv[1:]))
