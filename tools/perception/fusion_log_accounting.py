"""Read every fusion interval, including its shutdown tail, without ROS."""
import re


def parse_fusion_logs(logs):
    pattern = (r"Fusion sync (5s|tail): matched=(\d+) dropped=(\d+) "
               r"offset median/p95/max=([\d.]+)/([\d.]+)/([\d.]+) ms")
    intervals = [{"kind": kind, "matched": int(m), "dropped": int(d),
                  "median_ms": float(median), "p95_ms": float(p95), "max_ms": float(maximum)}
                 for kind, m, d, median, p95, maximum in re.findall(pattern, logs)]
    totals = re.findall(r"Fusion sync total: matched=(\d+) dropped=(\d+) pending lidar/yolo=(\d+)/(\d+)", logs)
    total = ({"matched": int(totals[0][0]), "dropped": int(totals[0][1]),
              "pending_lidar": int(totals[0][2]), "pending_yolo": int(totals[0][3])}
             if len(totals) == 1 else None)
    complete = (total is not None and sum(row["kind"] == "tail" for row in intervals) == 1
                and all(sum(row[key] for row in intervals) == total[key] for key in ("matched", "dropped")))
    return intervals, total, complete
