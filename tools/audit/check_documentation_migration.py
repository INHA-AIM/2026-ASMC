#!/usr/bin/env python3
"""Validate documentation retention and compatibility redirects.

The default check only needs the current working tree, so it also works in a
shallow CI checkout.  ``--with-history`` additionally audits the pre-migration
Git tree recorded in the manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set
from urllib.parse import unquote


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
from tools.audit.documentation_links import anchors, links, local_target, split_target, without_link_destinations

DEFAULT_MANIFEST = REPO_ROOT / "config/docs/documentation_migration_v1.json"
RELOCATION_MANIFEST = REPO_ROOT / "config/docs/documentation_relocation_v2.json"
CENTRAL_DIRECTORIES = {"onboarding", "architecture", "operations", "contracts", "handoffs", "competition", "records"}
RECORD_AREAS = {'camera', 'lidar', 'bridge', 'planning', 'hd-map', 'repository', 'archive'}
NAVIGATION_PAGES = {'docs/README.md', 'docs/handoffs/README.md', 'docs/records/README.md'}


def repo_path(raw: str) -> Path:
    path = Path(raw)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"repository path must be relative and cannot contain '..': {raw}")
    return REPO_ROOT / path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def markdown_link_targets(text: str) -> Iterable[str]:
    for link in links(text):
        yield link.target


def tracked_markdown_files() -> Sequence[Path]:
    result = subprocess.run(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard", "--", "*.md"],
        cwd=REPO_ROOT,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode == 0:
        return [REPO_ROOT / item.decode("utf-8") for item in result.stdout.split(b"\0") if item]
    return [path for path in REPO_ROOT.rglob("*.md") if ".git" not in path.parts]


def git_bytes(revision: str, path: str) -> bytes:
    result = subprocess.run(
        ["git", "show", f"{revision}:{path}"],
        cwd=REPO_ROOT,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"cannot read {revision}:{path}: {message}")
    return result.stdout


def git_tree_paths(revision: str) -> List[str]:
    result = subprocess.run(
        ["git", "ls-tree", "-r", "-z", "--name-only", revision, "docs"],
        cwd=REPO_ROOT,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"cannot inspect {revision}: {message}")
    return [item.decode("utf-8") for item in result.stdout.split(b"\0") if item]


def validate_current_tree(manifest: dict) -> List[str]:
    errors: List[str] = []
    if manifest.get("schema_version") != 1:
        errors.append(f"unsupported schema_version: {manifest.get('schema_version')}")

    baseline = manifest.get("baseline", {})
    retention = manifest.get("retention", [])
    sources: Set[str] = set()

    for entry in retention:
        source = entry.get("source", "")
        if not source or source in sources:
            errors.append(f"retention source is missing or duplicated: {source!r}")
        sources.add(source)
        targets = entry.get("targets", [])
        if not targets:
            errors.append(f"retention entry has no target: {source}")
        for raw in targets:
            try:
                target = repo_path(raw)
            except ValueError as exc:
                errors.append(str(exc))
                continue
            if not target.is_file():
                errors.append(f"retention target does not exist: {raw} (from {source})")
        evidence_raw = entry.get("evidence", "")
        try:
            evidence = repo_path(evidence_raw)
        except ValueError as exc:
            errors.append(str(exc))
        else:
            if not evidence.is_file():
                errors.append(f"retention evidence does not exist: {evidence_raw}")

    declared_markdown = sum(entry.get("source", "").endswith(".md") for entry in retention)
    declared_text = sum(entry.get("source", "").endswith(".txt") for entry in retention)
    if declared_markdown != baseline.get("markdown_files"):
        errors.append(
            f"declared Markdown count mismatch: expected {baseline.get('markdown_files')}, "
            f"got {declared_markdown}"
        )
    if declared_text != baseline.get("generated_text_files"):
        errors.append(
            f"declared generated text count mismatch: expected {baseline.get('generated_text_files')}, "
            f"got {declared_text}"
        )

    for entry in manifest.get("frozen_current_files", []):
        raw = entry.get("path", "")
        try:
            path = repo_path(raw)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if not path.is_file():
            errors.append(f"frozen file does not exist: {raw}")
            continue
        actual = sha256(path)
        expected = entry.get("sha256", "")
        if actual != expected:
            errors.append(f"frozen file SHA mismatch: {raw}: expected {expected}, got {actual}")

    redirects = manifest.get("redirects", [])
    retired = manifest.get("retired_redirects", [])
    if manifest.get("redirect_policy", {}).get("state") == "removed" and redirects:
        errors.append("removed redirect policy cannot retain live compatibility pages")
    for entry in retired:
        if repo_path(entry["legacy"]).exists():
            errors.append(f"retired compatibility page reintroduced: {entry['legacy']}")
        if not repo_path(entry["target"]).is_file():
            errors.append(f"retired compatibility replacement missing: {entry['target']}")
    legacy_paths: Set[Path] = set()
    legacy_names: Set[str] = set()
    for entry in redirects:
        legacy_raw = entry.get("legacy", "")
        target_raw = entry.get("target", "")
        if legacy_raw in legacy_names:
            errors.append(f"duplicate compatibility redirect: {legacy_raw}")
        legacy_names.add(legacy_raw)
        if legacy_raw not in sources:
            errors.append(
                f"compatibility redirect was not present in the pre-migration inventory: {legacy_raw}"
            )
        try:
            legacy = repo_path(legacy_raw)
            target = repo_path(target_raw)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        legacy_paths.add(legacy.resolve())
        if not legacy.is_file():
            errors.append(f"compatibility stub does not exist: {legacy_raw}")
            continue
        if not target.is_file():
            errors.append(f"compatibility target does not exist: {target_raw}")
            continue
        text = legacy.read_text(encoding="utf-8")
        if "정본이 아닙니다" not in text or "호환 안내" not in text:
            errors.append(f"compatibility stub is missing its warning: {legacy_raw}")
        resolved_targets: List[Path] = []
        for raw_link in markdown_link_targets(text):
            if raw_link.startswith(("#", "http://", "https://", "mailto:")):
                continue
            path_part = raw_link.split("#", 1)[0].split("?", 1)[0]
            resolved_targets.append((legacy.parent / path_part).resolve())
        if target.resolve() not in resolved_targets:
            errors.append(
                f"compatibility stub does not link to declared target: {legacy_raw} -> {target_raw}"
            )

    actual_stubs: Set[Path] = set()
    for markdown in (REPO_ROOT / "docs").rglob("*.md"):
        text = markdown.read_text(encoding="utf-8", errors="replace")
        if "호환 안내" in text and "정본이 아닙니다" in text:
            actual_stubs.add(markdown.resolve())
    if actual_stubs != legacy_paths:
        undeclared = sorted(str(path.relative_to(REPO_ROOT)) for path in actual_stubs - legacy_paths)
        missing = sorted(str(path.relative_to(REPO_ROOT)) for path in legacy_paths - actual_stubs)
        errors.append(f"compatibility stub inventory mismatch: undeclared={undeclared}, missing={missing}")

    assets = manifest.get("unchanged_assets", [])
    if len(assets) != baseline.get("unchanged_binary_assets"):
        errors.append(
            f"declared unchanged asset count mismatch: expected "
            f"{baseline.get('unchanged_binary_assets')}, got {len(assets)}"
        )
    for raw in assets:
        try:
            asset = repo_path(raw)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if not asset.is_file():
            errors.append(f"unchanged asset does not exist: {raw}")

    # Current docs should link directly to canonical files.  Only the redirect
    # pages themselves may resolve through another compatibility page.
    anchor_cache = {}
    for markdown in tracked_markdown_files():
        if not markdown.is_file() or markdown.resolve() in legacy_paths:
            continue
        text = markdown.read_text(encoding="utf-8", errors="replace")
        for raw_link in markdown_link_targets(text):
            if not local_target(raw_link):
                continue
            path_part, _, fragment = split_target(raw_link)
            resolved = (markdown.parent / path_part).resolve() if path_part else markdown.resolve()
            try:
                resolved.relative_to(REPO_ROOT)
            except ValueError:
                errors.append(
                    f"local Markdown link escapes repository root: "
                    f"{markdown.relative_to(REPO_ROOT)} -> {raw_link}"
                )
                continue
            if not resolved.exists():
                errors.append(
                    f"broken local Markdown link: "
                    f"{markdown.relative_to(REPO_ROOT)} -> {raw_link}"
                )
                continue
            if resolved in legacy_paths:
                errors.append(
                    f"active Markdown links through compatibility stub: "
                    f"{markdown.relative_to(REPO_ROOT)} -> {raw_link}"
                )
            if fragment and resolved.is_file() and resolved.suffix == '.md':
                if resolved not in anchor_cache:
                    anchor_cache[resolved] = anchors(resolved.read_text(encoding='utf-8'))
                if fragment not in anchor_cache[resolved]:
                    errors.append(f"broken Markdown anchor: {markdown.relative_to(REPO_ROOT)} -> {raw_link}")

    empty_dirs = [path for path in (REPO_ROOT / "docs").rglob("*") if path.is_dir() and not any(path.iterdir())]
    for path in empty_dirs:
        errors.append(f"empty documentation directory: {path.relative_to(REPO_ROOT)}")

    actual_directories = {p.name for p in (REPO_ROOT / 'docs').iterdir() if p.is_dir()}
    if actual_directories != CENTRAL_DIRECTORIES:
        errors.append(f"central documentation directories differ: {sorted(actual_directories)}")
    for path in (REPO_ROOT / 'docs').rglob('*.md'):
        relative = path.relative_to(REPO_ROOT / 'docs')
        if len(relative.parts) > 3:
            errors.append(f"documentation exceeds two directory levels: {relative}")
        if len(relative.parts) == 3 and relative.parts[0] != 'records':
            errors.append(f"non-record Markdown must be directly inside its category: {relative}")
        if len(relative.parts) == 1 and path.name != 'README.md':
            errors.append(f"unexpected root documentation page: {relative}")
        if path.name != 'README.md' and not re.fullmatch(r'[a-z0-9][a-z0-9.-]*\.md', path.name):
            errors.append(f"documentation filename violates lowercase/hyphen convention: {relative}")
    for path in (REPO_ROOT/'docs/records').iterdir():
        if path.is_dir() and path.name not in RECORD_AREAS:
            errors.append(f"unrecognized record category: {path.name}")

    return errors


def validate_history(manifest: dict) -> List[str]:
    errors: List[str] = []
    baseline = manifest["baseline"]
    revision = baseline["commit"]
    try:
        tree_paths = git_tree_paths(revision)
    except RuntimeError as exc:
        return [str(exc)]

    markdown_paths = sorted(path for path in tree_paths if path.endswith(".md"))
    text_paths = sorted(path for path in tree_paths if path.endswith(".txt"))
    retention = manifest["retention"]
    declared_markdown = sorted(entry["source"] for entry in retention if entry["source"].endswith(".md"))
    declared_text = sorted(entry["source"] for entry in retention if entry["source"].endswith(".txt"))

    if markdown_paths != declared_markdown:
        missing = sorted(set(markdown_paths) - set(declared_markdown))
        extra = sorted(set(declared_markdown) - set(markdown_paths))
        errors.append(f"baseline Markdown inventory mismatch: missing={missing}, extra={extra}")
    if text_paths != declared_text:
        missing = sorted(set(text_paths) - set(declared_text))
        extra = sorted(set(declared_text) - set(text_paths))
        errors.append(f"baseline text inventory mismatch: missing={missing}, extra={extra}")

    markdown_blobs = [git_bytes(revision, path) for path in markdown_paths]
    text_blobs = [git_bytes(revision, path) for path in text_paths]
    actual_values = {
        "markdown_files": len(markdown_paths),
        "markdown_lines": sum(len(blob.splitlines()) for blob in markdown_blobs),
        "markdown_bytes": sum(len(blob) for blob in markdown_blobs),
        "generated_text_files": len(text_paths),
        "generated_text_lines": sum(len(blob.splitlines()) for blob in text_blobs),
        "generated_text_bytes": sum(len(blob) for blob in text_blobs),
    }
    for key, actual in actual_values.items():
        if baseline.get(key) != actual:
            errors.append(f"baseline {key} mismatch: expected {baseline.get(key)}, got {actual}")

    for entry in retention:
        if entry['disposition'] == 'corrected_snapshot_frozen_before_v2':
            original = git_bytes(revision, entry['source'])
            snapshot = entry['corrected_snapshot']
            corrected = git_bytes(snapshot['commit'], snapshot['path'])
            if hashlib.sha256(original).hexdigest() != entry['historical_source_sha256']:
                errors.append(f"historical source SHA mismatch: {entry['source']}")
            if hashlib.sha256(corrected).hexdigest() != snapshot['sha256'] or repo_path(entry['targets'][0]).read_bytes() != corrected:
                errors.append(f"pre-existing corrected snapshot changed: {entry['source']}")
            if not repo_path(entry['correction_evidence']).is_file():
                errors.append(f"pre-existing correction evidence missing: {entry['source']}")
            continue
        if not entry["disposition"].startswith("byte_identical"):
            continue
        source_bytes = git_bytes(revision, entry["source"])
        target = repo_path(entry["targets"][0])
        if target.read_bytes() != source_bytes:
            errors.append(f"byte-identical retention changed: {entry['source']} -> {entry['targets'][0]}")

    assets = manifest.get("unchanged_assets", [])
    if baseline.get("unchanged_binary_assets") != len(assets):
        errors.append(
            "unchanged asset count mismatch: "
            f"expected {baseline.get('unchanged_binary_assets')}, declared {len(assets)}"
        )
    for raw in assets:
        target = repo_path(raw)
        if not target.is_file():
            errors.append(f"unchanged asset does not exist: {raw}")
            continue
        if target.read_bytes() != git_bytes(revision, raw):
            errors.append(f"unchanged asset differs from baseline: {raw}")

    return errors


def relocation_body(text: str) -> str:
    # Only explicitly added navigation blocks may differ during this move.
    text = re.sub(r'<!-- documentation-structure-v2:start -->.*?<!-- documentation-structure-v2:end -->\n', '', text, flags=re.S)
    return without_link_destinations(text)


def relocated_document_paths(text: str, inventory: dict) -> str:
    replacements = {entry['source']: entry['target'] for entry in inventory['entries']
                    if entry['action'] in {'move', 'remove_redirect'}}
    if not replacements:
        return text
    pattern = re.compile('|'.join(re.escape(x) for x in sorted(replacements, key=len, reverse=True)))
    return pattern.sub(lambda match: replacements[match[0]], text)


def validate_relocation(inventory: dict) -> List[str]:
    """One-time before/after proof; deliberately excluded from default CI."""
    errors = []
    if inventory.get('schema_version') != 2:
        return ['unsupported relocation inventory schema']
    sources, targets = set(), set()
    revision = inventory['base_commit']
    for entry in inventory['entries']:
        source, target = entry['source'], entry['target']
        if source in sources:
            errors.append(f'duplicate relocation source: {source}')
        sources.add(source)
        original = git_bytes(revision, source)
        if hashlib.sha256(original).hexdigest() != entry['source_sha256']:
            errors.append(f'relocation source SHA mismatch: {source}')
        if entry['action'] == 'remove_redirect':
            if repo_path(source).exists() or not repo_path(target).is_file():
                errors.append(f'invalid redirect retirement: {source} -> {target}')
            if '호환 안내'.encode('utf-8') not in original:
                errors.append(f'non-redirect deletion refused: {source}')
            continue
        if target in targets:
            errors.append(f'duplicate relocation destination: {target}')
        targets.add(target)
        path = repo_path(target)
        if not path.is_file():
            errors.append(f'relocation target missing: {target}')
            continue
        current = path.read_bytes()
        if entry['action'] == 'update_documentation_paths':
            equal = current.decode('utf-8') == relocated_document_paths(original.decode('utf-8'), inventory)
        elif entry['frozen'] or path.suffix != '.md':
            equal = current == original
        else:
            normalize = relocation_body if source in NAVIGATION_PAGES else without_link_destinations
            equal = normalize(current.decode('utf-8')) == without_link_destinations(original.decode('utf-8'))
        if not equal:
            errors.append(f'non-navigation content changed during relocation: {source} -> {target}')
        if source != target and repo_path(source).exists():
            errors.append(f'old relocation source remains: {source}')
    original_docs = set(git_tree_paths(revision))
    declared_docs = {x for x in sources if x.startswith('docs/')}
    if original_docs != declared_docs:
        errors.append(f'relocation inventory incomplete: missing={sorted(original_docs-declared_docs)}, extra={sorted(declared_docs-original_docs)}')
    for path in (REPO_ROOT/'docs').rglob('*'):
        if path.is_file() and path.relative_to(REPO_ROOT).as_posix() not in targets | set(inventory.get('new_files', [])):
            errors.append(f'undeclared new central document: {path.relative_to(REPO_ROOT)}')
    return errors


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--with-history",
        action="store_true",
        help="also verify the recorded pre-migration commit and byte-identical moves",
    )
    parser.add_argument('--verify-relocation', action='store_true',
                        help='one-time v2 source/target body-preservation check (requires base Git history)')
    args = parser.parse_args(argv)

    manifest = json.loads(args.config.read_text(encoding="utf-8"))
    errors = validate_current_tree(manifest)
    if args.with_history:
        errors.extend(validate_history(manifest))
    if args.verify_relocation:
        errors.extend(validate_relocation(json.loads(RELOCATION_MANIFEST.read_text())))

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    retention = manifest["retention"]
    markdown_count = sum(entry["source"].endswith(".md") for entry in retention)
    text_count = sum(entry["source"].endswith(".txt") for entry in retention)
    history = "verified" if args.with_history else "not requested"
    print(
        "documentation migration check passed: "
        f"markdown={markdown_count}, generated_text={text_count}, "
        f"redirects={len(manifest['redirects'])}, frozen={len(manifest['frozen_current_files'])}, "
        f"history={history}, relocation={'verified' if args.verify_relocation else 'not requested'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
