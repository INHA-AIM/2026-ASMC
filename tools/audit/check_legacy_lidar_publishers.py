#!/usr/bin/env python3
"""Fail when a launch file or launcher script starts drivable_filter_node.py.

Main (``PlannerObjectRosPublisher``) is the only ``/last_lidar`` publisher.
``src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py``
subscribes ``/lidar/tracked_objects_map`` and republishes ``last_lidar``.  It is
a manual cross-check of the C++ drivable filter and no launch file starts it;
run next to Main it would be a second ``/last_lidar`` publisher
(``won_tae_link_node/README.md``).  This check keeps it out of every launch XML
(``*.launch``, ``*.launch.xml``, rostest ``*.test``) and every shell launcher
(``*.sh``).  Mentions inside XML or shell comments are allowed.

The default scan uses ``git ls-files --cached --others --exclude-standard`` so
ignored build/devel trees are skipped; ``--no-git`` walks the directory instead.
A scan that finds no launch file at all fails closed.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
NODE_PATTERN = re.compile(r"drivable_filter_node", re.IGNORECASE)
LAUNCH_SUFFIXES = (".launch", ".launch.xml", ".test")
SHELL_SUFFIXES = (".sh",)
SKIPPED_DIRS = {".git", "build", "devel", "install", "log", "logs", "__pycache__"}
XML_COMMENT = re.compile(r"<!--.*?-->", re.DOTALL)
SHELL_COMMENT = re.compile(r"(^|\s)#.*$")


def is_launch(path: Path) -> bool:
    return path.name.endswith(LAUNCH_SUFFIXES)


def is_shell(path: Path) -> bool:
    return path.name.endswith(SHELL_SUFFIXES)


def candidate_files(root: Path, use_git: bool = True) -> List[Path]:
    """Launch XML and shell launchers below root, sorted."""
    names: Iterable[str] = ()
    listed = False
    if use_git:
        result = subprocess.run(
            ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
            cwd=root,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode == 0:
            names = [item.decode("utf-8") for item in result.stdout.split(b"\0") if item]
            listed = True
    if not listed:
        names = [
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_file() and not SKIPPED_DIRS.intersection(path.relative_to(root).parts)
        ]
    files = []
    for name in names:
        path = root / name
        if (is_launch(path) or is_shell(path)) and path.is_file():
            files.append(path)
    return sorted(files)


def strip_xml_comments(text: str) -> str:
    # Keep the line breaks of removed comments so reported line numbers stay exact.
    return XML_COMMENT.sub(lambda match: "\n" * match.group(0).count("\n"), text)


def strip_shell_comments(text: str) -> str:
    return "\n".join(SHELL_COMMENT.sub(r"\1", line) for line in text.split("\n"))


def violations_in(path: Path, text: str) -> List[Tuple[int, str]]:
    code = strip_xml_comments(text) if is_launch(path) else strip_shell_comments(text)
    found = []
    for number, line in enumerate(code.split("\n"), start=1):
        if NODE_PATTERN.search(line):
            found.append((number, line.strip()))
    return found


def check(root: Path, use_git: bool = True) -> Tuple[List[Path], List[str]]:
    """Return (scanned files, violation messages)."""
    scanned = candidate_files(root, use_git)
    errors: List[str] = []
    for path in scanned:
        text = path.read_text(encoding="utf-8", errors="replace")
        for number, line in violations_in(path, text):
            kind = "launch file" if is_launch(path) else "shell launcher"
            errors.append(
                f"{path.relative_to(root).as_posix()}:{number}: {kind} starts or references "
                f"drivable_filter_node (second /last_lidar publisher next to Main): {line}"
            )
    if not any(is_launch(path) for path in scanned):
        errors.append(f"no launch file found below {root}; refusing to pass an empty scan")
    return scanned, errors


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", type=Path, default=REPO_ROOT)
    parser.add_argument("--no-git", action="store_true", help="walk the directory instead of git ls-files")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    scanned, errors = check(root, use_git=not args.no_git)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    launches = sum(is_launch(path) for path in scanned)
    shells = sum(is_shell(path) for path in scanned)
    print(
        "legacy LiDAR publisher check passed: drivable_filter_node is started by no launch file "
        f"(launch={launches}, shell={shells})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
