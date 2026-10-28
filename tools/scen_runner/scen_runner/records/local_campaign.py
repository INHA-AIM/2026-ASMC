"""Read recorded trials and publish a standalone comparison in a new run.

No ROS dependency, no source-run writes. A chart is not autonomous acceptance.
"""

import argparse
from collections import Counter
import html
import json
import os
from pathlib import Path
from .analyze_local import quantiles


def rows(path):
    return (
        [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []
    )


def summarize(root):
    events = [
        r for r in rows(root / "events.jsonl") if r.get("event") == "native_sample"
    ]
    if not events:
        raise ValueError("native drive interval unavailable: " + str(root))
    begin, end = events[0]["ros_time"], events[-1]["ros_time"]
    plans = [
        r for r in rows(root / "decisions.jsonl") if begin <= r["decision_stamp"] <= end
    ]
    control = [
        r for r in rows(root / "control.jsonl") if begin <= r["decision_stamp"] <= end
    ]
    result = json.loads((root / "trial-result.json").read_text())
    valid = [r for r in plans if r["valid"]]
    following = (
        json.loads((root / "following-analysis.json").read_text())
        if (root / "following-analysis.json").exists()
        else {}
    )
    command_times = [r["decision_stamp"] for r in control if r.get("command_published")]
    result.update(
        planning_ms=quantiles([r["processing_ms"] for r in plans]),
        alignment_ms=quantiles([r.get("alignment_ms", 0) for r in plans]),
        selected_length_m=quantiles(
            [
                r.get("selected_length_m", r["path"][-1][0] if r["path"] else 0)
                for r in valid
            ]
        ),
        control_reasons=dict(Counter(r["reason"] for r in control)),
        plan_reasons=dict(Counter(r["reason"] for r in plans)),
        hard_brake_ticks=sum(r.get("brake", 0) > 0.9 for r in control),
        command_gap_ms=quantiles(
            [(b - a) * 1000 for a, b in zip(command_times, command_times[1:])]
        ),
        mpc_ms=quantiles([r["processing_ms"] for r in control if r["reason"] == "ok"]),
        following=following,
        series={
            "speed": [[r["ros_time"] - begin, r["ego"]["speed_kmh"]] for r in events],
            "path": [
                [
                    r["decision_stamp"] - begin,
                    r.get("selected_length_m", r["path"][-1][0] if r["path"] else 0),
                ]
                for r in plans
            ],
            "horizon": [
                [r["decision_stamp"] - begin, r.get("search_horizon_m", 0)]
                for r in plans
            ],
            "cap": [
                [r["decision_stamp"] - begin, r.get("selected_kph", 0)] for r in plans
            ],
            "brake": [
                [r["decision_stamp"] - begin, r.get("brake", 0) * 20] for r in control
            ],
        },
        failures=[
            dict(
                time_s=r["decision_stamp"] - begin,
                sequence=r["plan_sequence"],
                reason=r["reason"],
            )
            for r in control
            if r["reason"]
            not in ("ok", "plan_replacement_pending", "motion_sample_pending")
        ],
    )
    return result


def build(output, roots):
    output = Path(output)
    if not any((p / "registration.json").is_file() for p in output.parents):
        raise ValueError("output must be within a registered run")
    output.mkdir(exist_ok=False)
    completed = [r for r in roots if (Path(r) / "trial-result.json").is_file()]
    aborted = [
        dict(run_id=Path(r).name, **json.loads((Path(r) / "aborted.json").read_text()))
        for r in roots
        if (Path(r) / "aborted.json").is_file()
    ]
    if len(completed) + len(aborted) != len(roots):
        raise ValueError("unfinished or unidentified trial: not silently omitted")
    results = [summarize(Path(r)) for r in completed]
    for result, root in zip(results, completed):
        stability = Path(root) / "path-stability.json"
        if stability.exists():
            result["path_stability"] = json.loads(stability.read_text())
        result["link"] = os.path.relpath(
            Path(root) / "local-gallery/index.html", output
        )
    (output / "aborted.json").write_text(
        json.dumps(aborted, ensure_ascii=False, indent=2)
    )
    (output / "results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2)
    )
    data = json.dumps(results, ensure_ascii=False).replace("<", "\\u003c")
    page = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>LOCAL-2B 경로 안정화 · 주행/실패 대조</title>
<style>body{background:#101a24;color:#e9f3fb;font:16px system-ui;margin:24px auto;max-width:1250px;padding:0 16px}a{color:#61dcff}select{padding:10px;font-size:16px;max-width:100%}svg{width:100%;background:#142432}table{border-collapse:collapse;width:100%;margin:18px 0}td,th{text-align:left;padding:8px;border-bottom:1px solid #405064}pre{white-space:pre-wrap}article{padding:16px;background:#182d3b;margin:16px 0}</style>
<h1>LOCAL-2B 경로·속도·실패 검수</h1>
<p>실제 주행 구간만 집계합니다. 시작 준비·정차 입력도 원본 로그에 보존했습니다. 횡오차는 family 내부 일관성이며 독립 차선 GT가 아닙니다. 빈 터널 시험은 장애물 회피·전체 터널·25km/h 수락을 의미하지 않습니다.</p>
<p>파랑: 실측 속도(km/h), 청록: 선택 경로(m), 점선: 탐색 한계(m), 노랑: 속도 상한(km/h), 빨강: 브레이크×20. 세로축은 비교를 위한 공통 눈금입니다.</p>
<p><a href="aborted.json">시작 전 차단/준비 실패도 확인</a></p>__SUPPLEMENTS__
<select id="pick"></select><svg id="plot" viewBox="0 0 1100 340" role="img" aria-label="시간에 따른 속도와 경로 길이"></svg>
<div id="detail"></div><h2>시도별 비교</h2><table id="comparison"></table>
<p>갤러리는 정확한 source stamp의 RGB·노면에 후보·선택 경로·차체 sweep을 표시합니다. 청회색 근거리 셀은 과거 실제 관측 노면이며 현재 새 추론이 아닙니다.</p><a href="results.json">전체 수치·실패 계획 번호</a>
<script>const data=__DATA__;
const esc=x=>String(x??'-').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=x=>x==null?'-':Number(x).toFixed(2);
pick.innerHTML=data.map((r,i)=>`<option value="${i}">${esc(r.run_id)} · 설정 ${r.cap_kph}km/h</option>`).join('');
function draw(){const r=data[+pick.value], seconds=Math.max(1,...r.series.speed.map(p=>p[0]));const X=t=>50+t/seconds*1010,Y=v=>300-v/30*270;let s='';
for(let v=0;v<=30;v+=5)s+=`<path d="M50 ${Y(v)}H1060" stroke="#354a59"/><text x="5" y="${Y(v)}" fill="#bacdda">${v}</text>`;
for(const [k,color,dash] of [['speed','#54b9ff',''],['path','#40e4ba',''],['horizon','#80aaa5','6 4'],['cap','#ffe081',''],['brake','#ff7777','']]) s+=`<polyline fill="none" stroke="${color}" stroke-width="2" stroke-dasharray="${dash}" points="${r.series[k].map(p=>X(p[0])+','+Y(p[1])).join(' ')}"/>`;
s+=`<text x="50" y="328" fill="white">0s</text><text x="990" y="328" fill="white">${num(seconds)}s</text>`;plot.innerHTML=s;
detail.innerHTML=`<article><a href="${esc(r.link)}">RGB·노면·선택/탈락 경로·실패 갤러리 열기</a><p>접촉 ${r.native_contacts.length}, ${num(r.travel_m)}m, 급제동 명령 ${r.hard_brake_ticks}tick · 계산 p95 ${num(r.planning_ms.p95)}ms · MPC p95 ${num(r.mpc_ms.p95)}ms · 명령 공백 최대 ${num(r.command_gap_ms.max)}ms</p><pre>${esc(JSON.stringify({계획:r.plan_reasons,제어:r.control_reasons,실패:r.failures},null,2))}</pre></article>`;}
comparison.innerHTML='<tr><th>기록</th><th>설정/실측 중앙속도</th><th>횡오차 p95</th><th>급제동 명령 tick</th><th>경로 p50</th><th>접촉</th></tr>'+data.map(r=>`<tr><td>${esc(r.run_id)}</td><td>${r.cap_kph} / ${num(r.following.actual_speed_kph?.p50)}km/h</td><td>${num((r.following.home_error_abs_m?.p95??NaN)*100)}cm</td><td>${r.hard_brake_ticks}</td><td>${num(r.selected_length_m.p50)}m</td><td>${r.native_contacts.length}</td></tr>`).join('');pick.onchange=draw;pick.value=data.length-1;draw();</script>"""
    supplements = "".join(
        '<p><a href="'
        + html.escape(os.path.relpath(Path(r) / "local-gallery-v2/index.html", output))
        + '">'
        + html.escape(Path(r).name)
        + " 추가 실패 검수</a></p>"
        for r in roots
        if (Path(r) / "local-gallery-v2/index.html").is_file()
    )
    (output / "index.html").write_text(
        page.replace("__DATA__", data).replace("__SUPPLEMENTS__", supplements),
        encoding="utf-8",
    )
    print(output / "index.html")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("roots", nargs="+", type=Path)
    a = p.parse_args()
    build(a.output, a.roots)
