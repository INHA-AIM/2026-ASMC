"""Portable data paths with optional, machine-local volume protection.

Without a policy, the default is ``<checkout-parent>/data`` on the host and
``/data`` in the standard container. No particular drive, free-space minimum,
private management script or external dataset is required on another team PC.
"""
import json
import os
from pathlib import Path
import re
import shutil


def _read_policy(path):
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        raise RuntimeError("Cannot read local storage configuration: {}".format(path)) from error
    return value


def _local_policy():
    explicit = os.environ.get("ASMC_STORAGE_WORKSPACE")
    candidates = [Path(explicit).expanduser()] if explicit else Path(__file__).resolve().parents
    for parent in candidates:
        policy = parent / ".storage/policy.json"
        if policy.is_file():
            return _read_policy(policy)
    location = os.environ.get("ASMC_DATA_LOCATIONS")
    if location:
        path = Path(location).expanduser()
        if not path.is_file():
            raise RuntimeError("Configured ASMC_DATA_LOCATIONS is unavailable: {}".format(path))
        contract = _read_policy(path).get("storage_contract")
        if explicit and not contract:
            raise RuntimeError("Configured storage workspace/locations have no storage policy")
        return contract
    if explicit:
        raise RuntimeError("Configured storage workspace has no policy: {}".format(explicit))
    return None


def _in_container():
    return Path("/.dockerenv").is_file()


def default_data_root():
    for name in ("ASMC_STORAGE_DATA", "ASMC_DATA"):
        if os.environ.get(name):
            return os.path.expanduser(os.environ[name])
    if _in_container() and Path("/data").is_dir():
        return "/data"
    policy = _local_policy()
    if policy:
        return str(Path(policy["storage_root"]) / "data")
    if os.environ.get("AIM_PROJECT"):
        return str(Path(os.environ["AIM_PROJECT"]).expanduser() / "data")
    # .git can be either a directory (clone) or a file (worktree).
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists():
            return str(parent.parent / "data")
    raise RuntimeError("Set ASMC_DATA explicitly when running outside a checkout")


def _unescape(value):
    return re.sub(r"\\([0-7]{3})", lambda match: chr(int(match[1], 8)), value)


def _mounts():
    for line in Path("/proc/self/mountinfo").read_text().splitlines():
        left, right = line.split(" - ", 1)
        fields, tail = left.split(), right.split()
        yield _unescape(fields[4]), tail[0], _unescape(tail[1]), fields[5].split(",")


def _mount_point(path):
    target = Path(path).resolve()
    matches = [entry[:3] for entry in _mounts()
               if target == Path(entry[0]) or Path(entry[0]) in target.parents]
    if not matches:
        raise RuntimeError("Cannot confirm output mount")
    return max(matches, key=lambda item: len(item[0]))


def _writable(point):
    return any(entry[0] == point and "rw" in entry[3] for entry in _mounts())


def _inside(path, root):
    return path == root or root in path.parents


def _host_volume(policy):
    archive = Path(policy["storage_root"]).expanduser()
    if not archive.is_dir():
        raise RuntimeError("Configured storage is unavailable; refusing a fallback directory")
    expected = policy["expected_mount"]
    if _mount_point(archive) != (expected["point"], expected["type"], expected["source"]):
        raise RuntimeError("Configured storage volume is not mounted; refusing a shadow directory")
    return archive.resolve()


def _check_target(target, policy, allowed_container_mounts):
    point = _mount_point(target)[0]
    if _in_container() and point not in allowed_container_mounts:
        raise RuntimeError("Output is not in a configured writable container mount")
    if not _writable(point):
        raise RuntimeError("Output mount is read-only")
    ancestor = target
    while not ancestor.exists():
        ancestor = ancestor.parent
    minimum = policy.get("minimum_free_bytes", 50 * 2**30)
    if shutil.disk_usage(ancestor).free < minimum:
        raise RuntimeError("Storage is below its configured minimum free space")


def validate_output(data_root, category, run_id=None):
    """Validate before mkdir. No policy means no machine-specific restriction."""
    category = Path(category)
    if category.is_absolute() or ".." in category.parts:
        raise RuntimeError("Collection category cannot escape its data root")
    if run_id is not None and (not str(run_id) or Path(str(run_id)).name != str(run_id)
                               or str(run_id) in (".", "..")):
        raise RuntimeError("Run ID cannot escape its output category")
    target = Path(data_root).expanduser() / category
    if run_id is not None:
        target = target / str(run_id)
    target = target.resolve()
    if os.environ.get("ASMC_STORAGE_UNMANAGED_OUTPUT") == "1":
        return str(target)  # Explicit unit-fixture escape, not an operational mode.
    policy = _local_policy()
    if not policy:
        return str(target)
    archive = Path(policy["storage_root"]).expanduser().resolve()
    if _in_container():
        if not (_inside(target, Path("/data")) or _inside(target, archive / "data")):
            raise RuntimeError("Collection output must stay in mounted data storage")
        mounts = ("/data", str(archive))
    else:
        archive = _host_volume(policy)
        if not _inside(target, archive / "data"):
            raise RuntimeError("Collection output must resolve inside configured data storage")
        mounts = ()
    _check_target(target, policy, mounts)
    return str(target)


def default_artifact_root(repo_root):
    """Keep repo/artifacts for ordinary clones; managed profiles opt in."""
    policy = _local_policy()
    if not policy:
        return Path(repo_root) / "artifacts"
    if _in_container():
        for name in ("/learning/training_runs", "/artifacts"):
            path = Path(name)
            if path.is_dir() and _mount_point(path)[0] == name and _writable(name):
                return path
        raise RuntimeError("No configured writable artifact output mount")
    return Path(policy["storage_root"]).expanduser() / "artifacts"


def validate_bulk_output(path):
    """Guard training/export outputs without creating any directories."""
    target = Path(path).expanduser().resolve()
    if os.environ.get("ASMC_STORAGE_UNMANAGED_OUTPUT") == "1":
        return str(target)
    policy = _local_policy()
    if not policy:
        return str(target)
    archive = Path(policy["storage_root"]).expanduser().resolve()
    if _in_container():
        mounts = ("/data", "/artifacts", "/learning/training_runs", str(archive))
    else:
        archive = _host_volume(policy)
        if not _inside(target, archive):
            raise RuntimeError("Bulk output must stay inside configured storage")
        mounts = ()
    _check_target(target, policy, mounts)
    return str(target)
