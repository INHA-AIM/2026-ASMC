"""Pure protocol and source-stamp checks for the frame-boundary experiment."""
import math


def jobs(config, suite):
    if suite not in ('state', 'cost', 'diagnostic'):
        raise ValueError('unknown experiment suite')
    result = []
    for repeat in range(config['conditions']['repeats'] if suite != 'diagnostic' else 1):
        for bag in config['bags']:
            profiles = list(config['profiles']) if suite == 'state' else ['common_roll_pitch']
            profiles = profiles[repeat % len(profiles):] + profiles[:repeat % len(profiles)]
            for profile in profiles:
                for mode in config['gps_modes'] if suite != 'cost' else ['normal']:
                    for subscribers in config['subscriber_modes'] if suite == 'cost' else ['both']:
                        result.append(dict(phase='realtime', suite=suite, bag_id=bag,
                            profile=profile, gps_mode=mode, subscribers=subscribers, repeat=repeat+1))
    return result


def stamp_table(rows):
    result = {}
    previous = 0
    for row in rows:
        stamp = row['stamp_ns']
        if stamp <= previous or stamp in result:
            raise ValueError('zero, duplicate or regressing observation stamp')
        if row['status'] in (3, 4) and row['objects']:
            raise ValueError('invalid/pose-unavailable result contains objects')
        if row['status'] == 2 and row['objects']:
            raise ValueError('observed-empty result contains objects')
        if row['status'] == 1 and not row['objects']:
            raise ValueError('observed result contains no objects')
        if row['status'] not in (1, 2, 3, 4):
            raise ValueError('unknown observation status')
        for obj in row['objects']:
            values = obj['center'] + [v for p in obj['corners'] for v in p] + obj['dimensions']
            if not all(math.isfinite(v) for v in values) or any(v < 0 for v in obj['dimensions']):
                raise ValueError('invalid geometry')
            if obj['miss_count'] or len(obj['corners']) != 8:
                raise ValueError('predicted or incomplete geometry in observed output')
        result[stamp] = row
        previous = stamp
    return result


def compare_local(reference, candidate):
    """Strict same-source comparison; missing frames are retained as failures.

    This is not an independent accuracy metric or an ID-count tracking score.
    For realtime scheduling differences a separate geometric correspondence
    review is needed; this function never silently relaxes its strict result.
    """
    old, new = stamp_table(reference), stamp_table(candidate)
    common = old.keys() & new.keys()
    differences = []
    for stamp in sorted(common):
        a, b = old[stamp], new[stamp]
        fields = ('frame_id', 'status', 'stamp_provenance', 'objects')
        if any(a[k] != b[k] for k in fields):
            differences.append(stamp)
    missing, extra = sorted(old.keys()-new.keys()), sorted(new.keys()-old.keys())
    return dict(common_frames=len(common), reference_only_ns=missing, candidate_only_ns=extra,
        changed_ns=differences, exact=not missing and not extra and not differences)


def map_gate(rows, gps_rows, lidar_rows=None):
    """Check output pose age, invalid barriers and three-sample recovery.

    Observer reception times are not the Main callback times. A row near a
    delivery-order boundary remains an explicit review item, never an approval.
    """
    outputs = stamp_table(rows)
    receipts = {r['stamp_ns']//1000000*1000000:r['arrival'] for r in (lidar_rows or [])}
    violations = []
    for stamp, row in outputs.items():
        receipt = receipts.get(stamp,row['arrival'])
        samples = [s for s in gps_rows if s['stamp_ns'] <= stamp and s['arrival'] <= receipt]
        samples.sort(key=lambda s: s['arrival'])
        latest, consecutive, last_source = None, 0, 0
        for sample in samples:
            if sample['stamp_ns'] <= last_source:
                continue
            if last_source and sample['stamp_ns']-last_source > 300000000:
                consecutive = 0
            good = sample['status'] != 0 and all(math.isfinite(sample[k]) for k in
                ('latitude', 'longitude', 'altitude')) and (sample['latitude'] or sample['longitude'])
            consecutive = consecutive+1 if good else 0
            last_source, latest = sample['stamp_ns'], sample
        available = latest is not None and consecutive >= 3 and stamp-last_source <= 300000000
        if row['status'] in (1, 2) and not available:
            violations.append(dict(stamp_ns=stamp, reason='map_output_without_causal_valid_pose',
                gps_age_ms=(stamp-last_source)/1e6 if latest else None, recovery_samples=consecutive))
    return dict(checked_frames=len(outputs), violations=violations,
        input_receipt_reference_available=lidar_rows is not None,
        observer_boundary_review_required=True)
