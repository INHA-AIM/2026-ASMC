"""Pure B0 measurement analysis; missing evidence never becomes a passing zero."""
import math

try:
    from .lifecycle_measurements import percentile, rate_report, ten_hz_pass
    from .fusion_log_accounting import parse_fusion_observations
except ImportError:
    from lifecycle_measurements import percentile, rate_report, ten_hz_pass
    from fusion_log_accounting import parse_fusion_observations


def quantiles(values):
    finite = [float(v) for v in values if v is not None and math.isfinite(v)]
    return dict(samples=len(finite), p50=percentile(finite, 50),
                p95=percentile(finite, 95), p99=percentile(finite, 99),
                maximum=max(finite) if finite else None)


def output_report(rows, start, end, cutoff=0, expected_seconds=None):
    if end <= start:
        return dict(complete=False, messages=0, reason='missing_observation_window')
    rate = rate_report([(r['arrival'], r['stamp_ns'], r['valid']) for r in rows], start, end, cutoff)
    kept = [r for r in rows if start <= r['arrival'] < end and r['stamp_ns'] > cutoff]
    rate.update(complete=expected_seconds is None or end-start+1e-6 >= expected_seconds,
                duration_sec=end-start,
                valid_ratio=rate['valid_messages']/rate['messages'] if rate['messages'] else None,
                source_age_ms=quantiles([r.get('source_age_ms') for r in kept]),
                input_to_output_wall_ms=quantiles([r.get('wall_latency_ms') for r in kept]),
                input_anchor_missing=sum(r.get('input_anchor') == 'missing' for r in kept),
                input_anchor_ambiguous=sum(r.get('input_anchor') == 'ambiguous' for r in kept),
                negative_source_age=sum(r.get('source_age_ms', 0) < 0 for r in kept))
    return rate


def output_gate(name, row, lifecycle=True):
    if not row.get('complete') or not row.get('messages'):
        return False
    if row.get('duplicate_stamps') or row.get('reversed_stamps'):
        return False
    if name == 'yolo':
        return row['hz'] >= 18.0
    if lifecycle:
        return ten_hz_pass(row)
    # Startup ACTIVE bag trials are not lifecycle-acceptance trials.
    return bool(row['five_second_hz']) and all(hz >= 9.5 for hz in row['five_second_hz']) and row['max_gap_sec'] <= .25


def resource_report(samples):
    gpu = [s['gpu'] for s in samples if s.get('gpu') is not None]
    nodes = sorted({name for s in samples for name in s.get('nodes', {})})
    return dict(samples=len(samples),
                gpu_available=bool(gpu),
                gpu_missing_samples=sum(s.get('gpu') is None for s in samples),
                gpu_utilization_percent=quantiles([s['utilization_percent'] for s in gpu]),
                gpu_memory_mib=quantiles([s['memory_used_mib'] for s in gpu]),
                gpu_process_memory='unavailable_not_inferred_from_global_memory',
                nodes={name: dict(
                    cpu_percent=quantiles([s['nodes'][name].get('cpu_percent') for s in samples if name in s.get('nodes', {})]),
                    rss_mib=quantiles([s['nodes'][name]['rss_mib'] for s in samples if name in s.get('nodes', {})])) for name in nodes})


def fusion_report(screen_logs, rosout_events, inputs, tracks):
    intervals, total, complete = parse_fusion_observations(screen_logs, rosout_events)
    # Main truncates raw source ns to milliseconds before processing.
    raw = {r['stamp_ns']//1000000*1000000 for r in inputs}
    published = [r['stamp_ns'] for r in tracks]
    unique = set(published)
    return dict(intervals=intervals, totals=total, accounting_complete=complete,
                offset_p95_max_ms=max((r['p95_ms'] for r in intervals), default=None),
                raw_lidar_frames=len(inputs), raw_lidar_stamps_ns=sorted(raw),
                published_processed_lidar_stamps_ns=sorted(unique),
                processed_observation_source='unfiltered tracked_objects_map: once per successfully processed stamp',
                unpublished_raw_lidar_stamps_ns=sorted(raw-unique),
                # Trace OFF cannot separate selection skips from pose rejection.
                skip_timeout_reasons='unavailable_without_trace_diagnostic',
                duplicate_processed_stamps=len(published)-len(unique),
                pending_end=({k: total[k] for k in ('pending_lidar', 'pending_yolo')} if total else None))


def gps_phase_report(inputs, tracks, relays, bag_start_ns, gap_start=8., gap_end=14.):
    def phase(t):
        return 'before' if t < gap_start else 'gap' if t < gap_end else 'after'
    result = {}
    for label in ('before', 'gap', 'after'):
        result[label] = dict(
            sensor_inputs={name: sum(phase((r['ros_ns']-bag_start_ns)/1e9) == label for r in rows)
                           for name, rows in inputs.items()},
            tracked_frames=sum(phase((r['ros_ns']-bag_start_ns)/1e9) == label for r in tracks),
            gps_received=sum(phase(r['elapsed_sec']) == label for r in relays),
            gps_delivered=sum(phase(r['elapsed_sec']) == label and r['delivered'] for r in relays))
    after = [r for r in tracks if r['ros_ns'] >= bag_start_ns+int(gap_end*1e9)]
    result['first_tracked_after_restore_sec'] = ((after[0]['ros_ns']-bag_start_ns)/1e9-gap_end if after else None)
    return result
