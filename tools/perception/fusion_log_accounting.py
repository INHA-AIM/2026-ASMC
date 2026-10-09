"""Read every fusion interval, including its shutdown tail, without ROS."""
import re


def parse_fusion_observations(screen_logs, rosout_events):
    """Use intact ROS Log messages for intervals; stdout owns shutdown totals.

    roslaunch can interleave stdout/stderr chunks in its screen forwarding.
    Do not repair a truncated interval or substitute its counts with totals.
    A missing/duplicate ROS event still fails exact interval reconciliation.
    """
    intervals = [row['message'] for row in rosout_events
                 if row['name'] == '/Main' and row['message'].startswith('Fusion sync 5s:')]
    shutdown = re.findall(r'Fusion sync (?:tail|total):[^\r\n]*', screen_logs)
    return parse_fusion_logs('\n'.join(intervals + shutdown))


def parse_fusion_logs(logs):
    pattern = (r"Fusion sync (5s|tail): matched=(\d+) dropped=(\d+) "
               r"offset median/p95/max=([\d.]+)/([\d.]+)/([\d.]+) ms")
    intervals = [{"kind": kind, "matched": int(m), "dropped": int(d),
                  "median_ms": float(median), "p95_ms": float(p95), "max_ms": float(maximum)}
                 for kind, m, d, median, p95, maximum in re.findall(pattern, logs)]
    totals = re.findall(r"Fusion sync total: matched=(\d+) dropped=(\d+)(?: fresh=(\d+) reused=(\d+))? pending lidar/yolo=(\d+)/(\d+)", logs)
    total = ({"matched": int(totals[0][0]), "dropped": int(totals[0][1]),
              "pending_lidar": int(totals[0][4]), "pending_yolo": int(totals[0][5])}
             if len(totals) == 1 else None)
    if total is not None and totals[0][2]:
        total.update(fresh=int(totals[0][2]), reused=int(totals[0][3]))
    complete = (total is not None and sum(row["kind"] == "tail" for row in intervals) == 1
                and all(sum(row[key] for row in intervals) == total[key] for key in ("matched", "dropped")))
    if total is not None and 'fresh' in total:
        complete = complete and total['fresh']+total['reused']==total['matched']
    return intervals, total, complete
