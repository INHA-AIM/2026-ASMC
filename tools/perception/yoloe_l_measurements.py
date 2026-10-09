"""First-run feasibility gates, distinct from the existing 18Hz operational gate."""
from b0_measurements import output_report


def view_gate(measurement):
    windows=measurement.get('five_second_hz',[])
    p95=measurement.get('source_age_ms',{}).get('p95')
    p99=measurement.get('source_age_ms',{}).get('p99')
    checks=dict(complete=bool(measurement.get('complete')),
                fresh=measurement.get('duplicate_stamps',1)==0 and measurement.get('reversed_stamps',1)==0,
                full_windows=bool(windows) and all(v>=10 for v in windows),
                latency=p95 is not None and p99 is not None and p95<=150 and p99<=200,
                gap=measurement.get('max_gap_sec',float('inf'))<=.25,
                valid=measurement.get('valid_ratio')==1,
                nonnegative_source_age=measurement.get('negative_source_age',1)==0,
                source_anchors=measurement.get('input_anchor_missing',1)==0 and measurement.get('input_anchor_ambiguous',1)==0)
    return dict(passed=all(checks.values()),checks=checks,
                twenty_hz_met=bool(windows) and all(v>=20 for v in windows),
                operational_18hz_met=bool(windows) and all(v>=18 for v in windows))


def first_report(report):
    yolo={k:view_gate(v) for k,v in report.get('measurements',{}).items() if k.startswith('yolo_')}
    other={k:v for k,v in report.get('checks',{}).items() if not k.startswith('yolo_')}
    peak=report.get('resources',{}).get('gpu_memory_mib',{}).get('maximum')
    headroom=peak is not None and peak<=8188*.85
    return dict(views=yolo, all_three_views_present=len(yolo)==3,
                passed=len(yolo)==3 and all(v['passed'] for v in yolo.values()) and all(other.values()) and not report.get('failures'),
                other_checks=other, vram_headroom_target_met=headroom,whole_gpu_peak_mib=peak,
                failures=report.get('failures',[]),old_18hz_gate_report=report.get('passed'))
