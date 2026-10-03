#!/usr/bin/env python3
"""SHA-pinned build derivative; opt-in complete fit/assignment coverage."""
import hashlib
from pathlib import Path
import sys


def generate(source, output):
    expected = "6a85ddfe7ecbda1d49cbcb70db8420eb2c6570725dfeb65270d6f312f19e1624"
    if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
        raise ValueError("linefit vendored source changed; review derivative before rebuilding")
    text = source.read_text()
    header = source.parent.parent / 'include/ground_segmentation/ground_segmentation.h'
    if hashlib.sha256(header.read_bytes()).hexdigest() != 'b6059b0e482d5cc423ca5fe870c2fd5098dbe83b7df47004da2dd7e5c185b100':
        raise ValueError('linefit header changed')
    h = header.read_text().replace('  bool visualize;', '  bool visualize;\n  bool cover_all_work = false;')
    h = h.replace('public:\n', 'public:\n  std::vector<unsigned> fit_visits, assignment_visits;\n')
    target = output.parent / 'ground_live_linefit_include/ground_segmentation/ground_segmentation.h'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(h)
    for count in ('params_.n_segments', 'cloud_size'):
        for step in ('i', '(i+1)'):
            old = f'{count} / params_.n_threads * {step}'
            new = f'(params_.cover_all_work ? {count} * {step} / params_.n_threads : {old})'
            if text.count(old) != 1:
                raise ValueError('partition source drift: ' + old)
            text = text.replace(old, new)
    text = text.replace('  resetSegments();', '  resetSegments();\n  fit_visits.assign(params_.n_segments, 0);\n  assignment_visits.assign(cloud.size(), 0);')
    text = text.replace('    segments_[i].fitSegmentLines();', '    ++fit_visits[i];\n    segments_[i].fitSegmentLines();')
    text = text.replace('    Bin::MinZPoint point_2d =', '    ++assignment_visits[i];\n    Bin::MinZPoint point_2d =')
    start = text.index("void GroundSegmentation::insertPoints(const PointCloud& cloud) {")
    end = text.index("\nvoid GroundSegmentation::insertionThread", start)
    replacement = """void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  // Atomic members do not make the upstream compare/store of the (z,range)
  // pair atomic. Preserve input order for deterministic minima and ties.
  // Fitting/assignment coverage is separately controlled by cover_all_work.
  insertionThread(cloud, 0, cloud.size());
}
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text[:start]+replacement+text[end:])
    print("linefit source sha256="+hashlib.sha256(source.read_bytes()).hexdigest())


if __name__ == "__main__":
    generate(Path(sys.argv[1]), Path(sys.argv[2]))
