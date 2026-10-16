"""Current storage locations, separate from immutable capture provenance."""
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
    with path.open() as stream:
        locations = json.load(stream)
    if locations['schema_version'] != 1:
        raise ValueError('unsupported data location schema: ' + str(path))
    return locations


def capture_directory_windows(configured=''):
    """Keep custom capture names, but never rely on an empty-directory fallback."""
    return str(configured or load_locations()['capture_directory_windows'])


def resolve_sensor_root(configured, records):
    """Select one native root for a run; never blend two capture sessions.

    Explicit --sensor-root remains authoritative. Automatic lookup supports new
    default captures and the relocated archive. Ambiguous names require a root.
    """
    if configured is not None:
        return Path(configured).expanduser()
    locations = load_locations()
    candidates = []
    probes = list(records)[:16]
    for key in ('capture_directory_wsl', 'capture_archive_wsl'):
        root = Path(locations[key])
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
            [locations['capture_directory_wsl'], locations['capture_archive_wsl']]))
    return candidates[0]
