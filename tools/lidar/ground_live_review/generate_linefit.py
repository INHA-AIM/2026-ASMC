#!/usr/bin/env python3
"""Build derivative: stable ordered bin insertion, original fit/assign unchanged."""
import hashlib
from pathlib import Path
import sys


def generate(source, output):
    expected = "6a85ddfe7ecbda1d49cbcb70db8420eb2c6570725dfeb65270d6f312f19e1624"
    if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
        raise ValueError("linefit vendored source changed; review derivative before rebuilding")
    text = source.read_text()
    start = text.index("void GroundSegmentation::insertPoints(const PointCloud& cloud) {")
    end = text.index("\nvoid GroundSegmentation::insertionThread", start)
    replacement = """void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  // Atomic members do not make the upstream compare/store of the (z,range)
  // pair atomic. Preserve input order for deterministic minima and ties.
  // Line fitting and assignment still use the frozen four-thread policy.
  insertionThread(cloud, 0, cloud.size());
}
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text[:start]+replacement+text[end:])
    print("linefit source sha256="+hashlib.sha256(source.read_bytes()).hexdigest())


if __name__ == "__main__":
    generate(Path(sys.argv[1]), Path(sys.argv[2]))
