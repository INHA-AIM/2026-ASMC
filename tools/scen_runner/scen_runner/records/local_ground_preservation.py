"""Compare native replay near one known static fixture, without changing tracks.

The shared1m association gate is for this offline audit only, not a detector
metric or operational matching policy. A missing/invalid map remains missing.
"""

import argparse
import html
import json
import math
from pathlib import Path
from .analyze_local import quantiles


def build(root, scene):
    source = json.loads(Path(scene).read_text())
    fixture = source["replay"]["tunnel"]["baseline"][0]["record"]["definition"]
    target = fixture["pose"]
    result = dict(fixture=fixture, association_gate_m=1.0, modes={})
    series = {}
    for mode in ("baseline", "tight"):
        rows = [
            json.loads(l) for l in (root / (mode + ".jsonl")).read_text().splitlines()
        ]
        samples, counts, ground, total, height, points = [], [], [], [], [], []
        for row in rows:
            if row.get("processed"):
                counts.append(row["counts"]["ground_kept"])
                ground.append(row["timings_ms"]["ground"])
                total.append(row["timings_ms"]["total"])
            objects = row.get("typed", {}).get("map", {}).get("objects", [])
            nearest = min(
                objects,
                key=lambda o: math.hypot(
                    o["center"][0] - target["x"], o["center"][1] - target["y"]
                ),
                default=None,
            )
            distance = (
                math.hypot(
                    nearest["center"][0] - target["x"],
                    nearest["center"][1] - target["y"],
                )
                if nearest
                else None
            )
            matched = nearest is not None and distance <= 1.0
            samples.append(
                dict(
                    stamp_ns=row["stamp_ns"],
                    map_valid=row.get("map", {}).get("valid", False),
                    matched=matched,
                    distance_m=distance,
                    object=nearest if matched else None,
                )
            )
            if matched:
                height.append(nearest["height"])
                points.append(nearest["points"])
        series[mode] = samples
        result["modes"][mode] = dict(
            scans=len(rows),
            matched=sum(s["matched"] for s in samples),
            map_invalid=sum(not s["map_valid"] for s in samples),
            object_height_m=quantiles(height),
            object_points=quantiles(points),
            ground_kept=quantiles(counts),
            ground_ms=quantiles(ground),
            total_ms=quantiles(total),
        )
    result["new_misses"] = [
        a["stamp_ns"]
        for a, b in zip(series["baseline"], series["tight"])
        if a["matched"] and not b["matched"]
    ]
    if [s["stamp_ns"] for s in series["baseline"]] != [
        s["stamp_ns"] for s in series["tight"]
    ]:
        raise ValueError("different input scans")
    with (root / "preservation.json").open("x") as f:
        json.dump(dict(summary=result, scans=series), f, indent=2)
    body = "".join(
        f'<tr><td>{m}</td><td>{r["matched"]}/{r["scans"]}</td><td>{r["object_points"]["p50"]}</td><td>{r["object_height_m"]["p50"]:.3f}</td><td>{r["ground_ms"]["p95"]:.2f}</td><td>{r["total_ms"]["p95"]:.2f}</td></tr>'
        for m, r in result["modes"].items()
    )
    with (root / "index.html").open("x") as f:
        f.write(
            """<!doctype html><meta charset="utf-8"><title>LOCAL-2C 실제 barrier 보존 대조</title><style>body{font:17px system-ui;background:#132333;color:#eee;max-width:1100px;margin:25px auto;padding:20px}a{color:#78ddff}td,th{padding:10px;border-bottom:1px solid #456}pre{white-space:pre-wrap}</style><h1>실제 barrier: 지면 fitting 대조</h1>
<p>기존LOCAL-1 drive13/15 bag은 원시 점군이 없어 재실행0scan이었습니다. 이를 통과로 세지 않았습니다. 같은 left-barrier 배치를 새로 재현하고3km/h 상한·5m 접근에서 raw pointcloud와 판단을 기록했습니다. 접촉0이며 장애물을 통과·회피한 수락은 아닙니다.</p>
<p>시나리오의 정적PE_Firewall_Orange 위치에 가장 가까운 발행 map 객체를 두 방법 모두 같은1m 한계로 찾았습니다. simulator 위치는 평가에만 쓰며 local 제어 입력이 아닙니다. 다른 객체와 잘못 연결될 가능성·지도 위치 오차를 포함하므로 전체 재현율로 해석하지 않습니다.</p>
<table><tr><th>설정</th><th>근처 객체가 발행된scan</th><th>점 수 중앙</th><th>높이 중앙m</th><th>지면 p95ms</th><th>전체 p95ms</th></tr>"""
            + body
            + f"""</table><p>기존에는 있었지만 후보에서 없어진scan: {len(result['new_misses'])}. 높이·점 수가 증가할 수 있으며 바이트 동등성이나 모든 장애물 보존을 뜻하지 않습니다. 낮은 노면 오검출을 제거하는 대신 다른 벽면 잔류점과 군집 수가 증가하는 경우도 있어 기본 승격은 보류했습니다.</p>
<p><a href="../local2c-20261028-barrier01/local-gallery/index.html">정확한 RGB·semantic·경로</a> · <a href="../local2c-20261028-audit01/ground/index.html">노면 오검출 원인</a> · <a href="preservation.json">전체scan·시각·수치</a></p><pre>"""
            + html.escape(json.dumps(result, ensure_ascii=False, indent=2))
            + "</pre>"
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("scene", type=Path)
    a = p.parse_args()
    build(a.root, a.scene)
