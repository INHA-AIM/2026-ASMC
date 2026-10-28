"""LOCAL rebase/ground/replay evidence from already registered immutable inputs.

This report does not infer live success or wall/obstacle GT from object size.
"""
import argparse
import html
import json
import math
from pathlib import Path
from .analyze_local import quantiles


def build(root, scene):
    fixture = json.loads(scene.read_text())["replay"]["tunnel"]["baseline"][0]["record"]["definition"]
    target = fixture["pose"]
    rows = {mode: [json.loads(line) for line in (root / (mode + ".jsonl")).read_text().splitlines()]
            for mode in ("baseline", "tight")}
    if [r["stamp_ns"] for r in rows["baseline"]] != [r["stamp_ns"] for r in rows["tight"]]:
        raise ValueError("different replay input scans")
    matches, summary = {}, {}
    for mode, scans in rows.items():
        matched, heights, local = [], [], []
        for row in scans:
            candidates = row.get("typed", {}).get("map", {}).get("objects", [])
            nearest = min(candidates, key=lambda o: math.hypot(o["center"][0]-target["x"], o["center"][1]-target["y"]), default=None)
            good = nearest is not None and math.hypot(nearest["center"][0]-target["x"], nearest["center"][1]-target["y"]) <= 1.
            matched.append(good)
            if good:
                heights.append(nearest["height"])
            local.extend(row.get("typed", {}).get("local", {}).get("objects", []))
        matches[mode] = matched
        summary[mode] = dict(scans=len(scans), matched=sum(matched),
            map_invalid=sum(not r.get("map", {}).get("valid", False) for r in scans),
            ground_ms=quantiles([r["timings_ms"]["ground"] for r in scans if r.get("processed")]),
            total_ms=quantiles([r["timings_ms"]["total"] for r in scans if r.get("processed")]),
            object_height_m=quantiles(heights), local_objects=len(local),
            local_length_over_4m=sum(o["length"]>4 for o in local))
    report = dict(fixture=fixture, offline_association_gate_m=1., modes=summary,
                  new_misses=[r["stamp_ns"] for i,r in enumerate(rows["baseline"])
                              if matches["baseline"][i] and not matches["tight"][i]],
                  scope="same latest-main configuration; fit band only differs; size count is not wall GT")
    with (root / "ground-summary.json").open("x") as f:
        json.dump(report, f, indent=2)
    table = ''.join(f'<tr><td>{m}</td><td>{s["matched"]}/{s["scans"]}</td><td>{s["map_invalid"]}</td><td>{s["local_objects"]}</td><td>{s["local_length_over_4m"]}</td><td>{s["total_ms"]["p95"]:.2f}</td></tr>' for m,s in summary.items())
    cards = ''
    for name in ('empty-replay', 'barrier-replay'):
        data = json.loads((root/name/'summary.json').read_text())
        cards += f'<h2>{html.escape(name)}</h2><a href="{name}/local-gallery/index.html">동일 source 영상·노면·경로·실패 사례</a><pre>{html.escape(json.dumps(data,ensure_ascii=False,indent=2))}</pre>'
    with (root/'index.html').open('x') as f:
        f.write('''<!doctype html><meta charset="utf-8"><title>LOCAL-2D · 리베이스 이후 오프라인 검증</title>
<style>body{font:17px system-ui;background:#132333;color:#eef;max-width:1200px;margin:25px auto;padding:20px}a{color:#78ddff}td,th{padding:10px;border-bottom:1px solid #456}pre{white-space:pre-wrap}p{line-height:1.6}</style>
<h1>LOCAL-2D · 최신 main 기반 준비와 오프라인 검증</h1>
<p>main be20138a와 LOCAL을 리베이스한 별도 컨테이너에서 검사했습니다. NORMAL·기존 음영 제어와 운영 컨테이너를 바꾸지 않았습니다. 실제 회피·복귀 수락은 아직 아닙니다.</p>
<h2>지면 fitting 0.10m 후보</h2><p>최신 main 군집 설정을 동일하게 적용한 117scan 재생입니다. 실제 barrier 근처의 객체를 두 방법 모두 같은 1m 기준으로 연결했습니다. 이는 전체 객체 재현율이나 독립 GT 정확도가 아닙니다. 원본 긴 노면 군집 262점은 제거되고 양의 높이 군집 1,711점은 보존됐습니다.</p>
<table><tr><th>설정</th><th>barrier 근처 검출</th><th>map 무효 scan</th><th>local 객체 합계</th><th>길이&gt;4m (진단만)</th><th>처리 p95 ms</th></tr>'''+table+'''</table><p>길이 수량은 벽/장애물 정답이 아니며 길이로 제거하지 않습니다. local 증가는 다른 잔류점도 생길 수 있음을 뜻합니다. 낮은 barrier 한 사례를 통과했지만 다양한 낮은 실물과 벽 혼입의 완전한 수락은 아닙니다.</p>
<a href="ground-summary.json">지면 수치·입력 시각</a> · <a href="ground-probe.json">실패 점군 native probe</a>
<h2>MPC 공백과 경로 비용</h2><p>미래 관측으로 계산하지 않고 실제 명령 이력을 경과시간으로 재구성합니다. 계산 재개 시 이전 solver 초기값만 폐기합니다. 350ms freshness·150ms 대기·250ms selector 한계는 유지합니다. 후보는 거리 적분으로 비교하고 이전 경로도 현재 충돌 및 운동학 검사에 통과해야 합니다.</p>
<p>다음 replay는 기록된 ego 이동·인지 출력의 오프라인 대조입니다. 새 경로에 따라 ego가 달라지지 않으므로 충돌 없이 회피했다는 근거로 쓰지 않습니다. 지면 후보의 새 LiDAR 결과로 입력 bag을 덮어쓰지도 않았습니다. 시작의 motion 대기, 오래된 객체 입력, clock 재동기화와 250ms 초과 공백을 통계에서 숨기지 않았습니다.</p>'''+cards+'''<p>다음 실제 시험: 실행 소유권 확보 → 새 LiDAR 설정으로 빈 터널 10km/h → 좌우 단일 장애물 각3회 회피·복귀. 현재 다른 작업의 실행과 인지 생산자를 유지한 상태라 이 폐루프 시험은 미실행입니다.</p>''')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('scene',type=Path)
    a=p.parse_args();build(a.root,a.scene)
