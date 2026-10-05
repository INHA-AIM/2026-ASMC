#!/usr/bin/env python3
"""Build-only derivatives of the pinned vendor. Never edits the vendor checkout."""
import hashlib
from pathlib import Path
import sys

# Full digest, not a branch name: reject a different implementation.
CPP_SHA = 'f7037532eea5027994f672fbdca34be9b7be1033fd8d6f23cf6e41d0912f9b2a'
HEADER_SHA = '3803a5cd4c1064c71fa09f3388369e04ca912d18cdc2b8f52a796a6bbfc362f8'

def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('source anchor drift: ' + old[:100])
    return text.replace(old, new, 1)

def generate(repo, out):
    vendor = repo / 'src/asmc_lidar/vendor/patchwork'
    cpp_path = vendor / 'src/patchworkpp.cpp'
    hdr_path = vendor / 'include/patchwork/patchworkpp.h'
    for path, expected in ((cpp_path, CPP_SHA), (hdr_path, HEADER_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('pinned source SHA drift: ' + str(path))
    hdr = hdr_path.read_text()
    hdr = replace_once(hdr, 'struct Params', '''struct HeightPatch {
  int zone, ring, sector, count, seed_count;
  double z, nx, ny, nz, variance, elevation_threshold;
  bool accepted;
};
struct HeightUpdate {
  int ring, count;
  double before, after, threshold_before, threshold_after;
  const char* reason;
};
struct Params''')
    hdr = replace_once(hdr, '  double sensor_height;', '''  // H1 build-only policy; operating Params remains unchanged.
  int height_policy = 0;  // 0 original, 1 skip singleton, 2 fixed height
  bool height_trace = false;
  double sensor_height;''')
    hdr = replace_once(hdr, '  double getHeight()', '''  std::vector<HeightPatch> height_patches;
  std::vector<HeightUpdate> height_updates;
  friend struct HeightTestAccess;
  double getHeight()''')
    cpp = cpp_path.read_text()
    cpp = replace_once(cpp, '  clock_t t1_1 = clock();', '''  height_patches.clear();
  height_updates.clear();
  clock_t t1_1 = clock();''')
    anchor = '        if (is_upright && is_not_elevated && is_near_zone) {'
    cpp = replace_once(cpp, anchor, '''        if (params_.height_trace) height_patches.push_back({
          zone_idx, concentric_idx, sector_idx,
          int(zone[ring_idx][sector_idx].size()), int(regionwise_ground_.size()),
          ground_elevation, normal_(0), normal_(1), normal_(2), ground_flatness,
          is_near_zone ? params_.elevation_thr[concentric_idx] : 0.0,
          is_upright && is_not_elevated && is_near_zone});
''' + anchor)
    cpp = replace_once(cpp, '    if (update_elevation_[i].empty()) continue;', '''    const auto count = update_elevation_[i].size();
    const double before = params_.sensor_height;
    const double threshold_before = params_.elevation_thr[i];
    const bool singleton_skip = params_.height_policy != 0 && count < 2;
    if (count == 0 || singleton_skip) {
      if (params_.height_trace) height_updates.push_back({i, int(count), before,
        params_.sensor_height, threshold_before, params_.elevation_thr[i],
        count == 0 ? "empty_hold" : "singleton_hold"});
      continue;
    }''')
    cpp = replace_once(cpp, '      params_.sensor_height    = -update_mean;',
                       '      if (params_.height_policy != 2) params_.sensor_height = -update_mean;')
    anchor = '    int exceed_num = update_elevation_[i].size() - params_.max_elevation_storage;'
    cpp = replace_once(cpp, anchor, '''    if (params_.height_trace) height_updates.push_back({i, int(count), before,
      params_.sensor_height, threshold_before, params_.elevation_thr[i],
      i == 0 ? (params_.height_policy == 2 ? "fixed_height_threshold_updated" :
      (count == 1 ? "legacy_singleton_zero" : "height_updated")) : "threshold_updated"});
''' + anchor)
    (out / 'include/patchwork').mkdir(parents=True, exist_ok=True)
    (out / 'src').mkdir(exist_ok=True)
    (out / 'include/patchwork/patchworkpp.h').write_text(hdr)
    (out / 'src/patchworkpp.cpp').write_text(cpp)
    for name in ('plane_fit.h', 'types.h'):
        (out / 'include/patchwork' / name).write_bytes((vendor / 'include/patchwork' / name).read_bytes())
    (out / 'src/plane_fit.cpp').write_bytes((vendor / 'src/plane_fit.cpp').read_bytes())
    (out / 'LICENSE').write_bytes((vendor / 'LICENSE').read_bytes())

if __name__ == '__main__':
    generate(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
