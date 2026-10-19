"""Per-PC native input locations, separate from immutable capture provenance.

The shipped config only describes the conventional MORAI installation. No
external archive/AIM drive is required or auto-mounted. A private config can
also carry storage_contract for the opt-in output guard in storage_paths.
"""
import json
import os
from pathlib import Path


def load_locations():
    configured = os.environ.get('ASMC_DATA_LOCATIONS')
    if configured:
        path = Path(configured).expanduser()
    else:
        path = Path(__file__).resolve().parents[2] / 'config/data_locations.json'
        if not path.is_file():
            # Installed catkin Python modules live separately from package share.
            import rospkg
            path = Path(rospkg.RosPack().get_path('data_collection')) / 'config/data_locations.json'
    try:
        with path.open() as stream:
            locations = json.load(stream)
    except (OSError, ValueError) as error:
        raise ValueError('Cannot read ASMC data locations: ' + str(path)) from error
    if not isinstance(locations, dict) or locations.get('schema_version') != 1:
        raise ValueError('unsupported data location schema: ' + str(path))
    overrides = {
        'capture_directory_windows': 'ASMC_CAPTURE_DIR_WINDOWS',
        'capture_directory_wsl': 'ASMC_SENSOR_ROOT',
        'capture_archive_wsl': 'ASMC_CAPTURE_ARCHIVE',
        'aim_directory_wsl': 'ASMC_AIM_ROOT',
    }
    for key in ('capture_directory_windows', 'capture_directory_wsl',
                'capture_archive_wsl', 'aim_directory_wsl', 'aim_legacy_directory_wsl'):
        environment = overrides.get(key)
        value = os.environ.get(environment, locations.get(key, '')) if environment else locations.get(key, '')
        if not isinstance(value, str):
            raise ValueError('Data location must be a string: ' + key)
        # Private config paths may be relative to that config, not the caller's cwd.
        if key.endswith('_wsl') and value:
            candidate = Path(value).expanduser()
            if not candidate.is_absolute():
                candidate = (Path.cwd() if environment and environment in os.environ else path.parent) / candidate
            value = str(candidate)
        locations[key] = value
    return locations


def capture_directory_windows(configured=''):
    """The path is interpreted on the simulator PC, not on the ROS host."""
    if configured:
        return str(configured)
    value = load_locations()['capture_directory_windows']
    if not value:
        raise ValueError('Set morai_file_dir or capture_directory_windows for this simulator PC')
    return value


def resolve_sensor_root(configured, records):
    """Select one native root for a run; never blend two capture sessions.

    Explicit --sensor-root remains authoritative. Automatic lookup supports new
    conventional captures and an optional per-PC archive. Ambiguous names
    require a root even when the duplicate is after the first few records.
    A source root is read-only here; it is never created or mounted.
    """
    if configured is not None or os.environ.get('ASMC_SENSOR_ROOT'):
        value = configured if configured is not None else os.environ['ASMC_SENSOR_ROOT']
        root = Path(value).expanduser()
        if not str(value).strip() or not root.is_dir():
            raise ValueError('Explicit SensorData root is unavailable: ' + str(root))
        return root
    locations = load_locations()
    candidates = []
    probes = list(records)
    if not probes:
        raise ValueError('Cannot select a Capture root without successful records')
    for record in probes:
        name = str(record['custom_name'])
        if not name or name in ('.', '..') or '/' in name or '\\' in name:
            raise ValueError('Capture custom_name must be a single filename component')
    checked = []
    for key in ('capture_directory_wsl', 'capture_archive_wsl'):
        if not locations[key]:
            continue  # Empty optional roots must not become Path('.') lookups.
        root = Path(locations[key]).expanduser().resolve()
        if root in checked:
            continue  # A symlink alias to the same source is not a second run.
        checked.append(root)
        if not root.is_dir():
            continue
        found = any((root / ('CAMERA_' + str(camera)) /
                     (str(record['custom_name']) + '_Intensity.png')).is_file()
                    for record in probes for camera in (1, 2, 3, 4))
        if found and root not in candidates:
            candidates.append(root)
    if len(candidates) != 1:
        raise ValueError('Capture root {}: pass --sensor-root explicitly; checked {}'.format(
            'ambiguous' if candidates else 'not found',
            [str(root) for root in checked]))
    return candidates[0]
