"""LOCAL-2C fixed-input reference comparison and all attempted drive evidence.

Read-only source runs. Output belongs to a new registered run; no success-only
sampling and no inference of full-course acceptance from a short speed chart.
"""

import argparse
import html
import json
import os
from pathlib import Path


def build(output, baseline, roots):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    output = Path(output)
    audit = output.parent
    if not (audit / "registration.json").is_file():
        raise ValueError("registered output required")
    output.mkdir(exist_ok=False)
    old = json.loads(Path(baseline).read_text())
    data, incomplete = [], []
    for root in map(Path, roots):
        path = root / "speed-analysis.json"
        if not path.exists():
            incomplete.append(dict(run_id=root.name, reason="no_native_speed_analysis"))
            continue
        row = json.loads(path.read_text())
        result = json.loads((root / "trial-result.json").read_text())
        following = json.loads((root / "following-analysis.json").read_text())
        provenance = json.loads((root / "runtime-provenance.json").read_text())
        gallery = root / "local-gallery/index.html"
        if not gallery.is_file():
            gallery = root / "local-gallery-v2/index.html"
        row.update(
            travel_m=result["travel_m"],
            contacts=result["native_contacts"],
            home_error_p95_m=following["home_error_abs_m"]["p95"],
            options=provenance["options"],
            link=os.path.relpath(gallery, output),
        )
        data.append(row)
    (output / "results.json").write_text(
        json.dumps(dict(trials=data, unavailable=incomplete), indent=2)
    )
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    # Native C++ Plan: same free raster, initial4.3m/s, acceleration0, cap15km/h.
    for name, label in (
        ("before", "Before: scalar cap + mean acceleration"),
        ("after", "After: physical endpoint state"),
    ):
        rows = np.loadtxt(audit / f"speed-native-{name}.csv", delimiter=",")
        row = next(r for r in rows if abs(r[0] - 4.3) < 1e-6 and abs(r[1]) < 1e-6)
        axes[0].plot(
            [0, 0.1, 0.5, 1.0, 2.0, 2.5],
            [row[0] * 3.6] + [row[i] * 3.6 for i in (5, 7, 9, 11, 13)],
            "o-",
            label=label,
        )
    axes[0].axhline(15, c="#777", ls="--")
    axes[0].axvspan(1.5, 2.5, color="#999", alpha=.12)
    axes[0].text(1.55, 15.2, "Remaining-distance braking\n(expected in both)", fontsize=9)
    axes[0].set(
        title="Same native input: speed reference (not closed-loop)",
        xlabel="trajectory time [s]",
        ylabel="km/h",
    )
    axes[0].legend()
    axes[0].grid()
    # Display a fixed chosen repeat, while the table includes every attempt.
    chosen = next(r for r in data if r["run_id"].endswith("drive05"))
    for row, label in ((old, "LOCAL-2B drive16"), (chosen, "LOCAL-2C drive05")):
        axes[1].plot(
            [s["t"] for s in row["series"]],
            [s["speed_kph"] for s in row["series"]],
            label=label,
        )
    axes[1].axhline(15, c="#777", ls="--")
    axes[1].set(
        title="Separate65m repeats: cap15km/h (not identical sensor input)",
        xlabel="native drive elapsed [s]",
        ylabel="km/h",
    )
    axes[1].legend()
    axes[1].grid()
    fig.tight_layout()
    fig.savefig(output / "speed.png", dpi=140)
    plt.close(fig)
    table = []
    for r in data:
        fit = "0.10m 후보" if r["options"].get("lidar_config") else "기존"
        table.append(
            f'<tr><td><a href="{html.escape(r["link"])}">{html.escape(r["run_id"])}</a></td>'
            f'<td>{r["requested_cap_kph"]:g} / {fit}</td><td>{r["travel_m"]:.2f}</td>'
            f'<td>{r["steady_cap_error_kph"]["p95"]:.3f}</td><td>{100*r["home_error_p95_m"]:.2f}</td>'
            f'<td>{r["hard_brake_ticks"]}</td><td>{r["command_gap_ms"]["max"]:.1f}</td><td>{len(r["contacts"])}</td></tr>'
        )
    (output / "index.html").write_text(
        """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LOCAL-2C 속도 연속성·지면 OBB 검수</title><style>body{font:17px system-ui;background:#132233;color:#eee;max-width:1250px;margin:24px auto;padding:16px}a{color:#78ddff}p{line-height:1.6}img{max-width:100%}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:8px;border-bottom:1px solid #456;text-align:left}.scroll{overflow:auto}</style>
<h1>LOCAL-2C 속도 연속성·지면 OBB 검수</h1>
<p>25km/h 달성 대신10·15km/h의 빈 구간 추종을 대조했습니다. 실제 접촉·원본 실패를 모두 보존했습니다. 짧은65m 주행이며 전체 터널·장애물 회피·운영 승격 수락이 아닙니다.</p>
<p><a href="../campaign/index.html">모든 시도: 속도·선택 경로·제동·실패 RGB</a> · <a href="../ground/index.html">노면 OBB의 실제 점·지면 평면 대조</a> · <a href="../campaign/aborted.json">시작 전 실패</a> · <a href="results.json">원본 집계</a></p>
<p>LOCAL의 물리 궤적에 scalar 속도 cap을 바로 넣고 평균 가속을 끝점 상태로 재사용하던 처리를 수정했습니다. 기존 NORMAL 속도 정책은 유지합니다. 아래 첫 그래프는 동일 입력의 native 계산, 둘째는 서로 다른 실제 반복입니다.0.5초의 불필요한 속도 저하가 줄었으며, 약1.5초 이후는 두 방식 모두 경로 끝의 정지거리 제약으로 감속합니다.</p><img src="speed.png" alt="동일 입력 속도 기준과 실제 반복 속도">
<div class="scroll"><table><tr><th>기록/갤러리</th><th>상한km/h / 지면 fitting</th><th>거리m</th><th>속도편차 p95 km/h</th><th>family 횡오차 p95 cm</th><th>급제동 tick</th><th>명령 공백 최대ms</th><th>접촉</th></tr>"""
        + "".join(table)
        + """</table></div>
<p>속도편차는 실제 주행 시작10초 이후의 고정 구간이며 실패·회복도 포함합니다. 짧은 구간은 정상 순항과 분리해서 검토하십시오. family 횡오차는 내부 일관성이고 독립 차선 GT가 아닙니다.</p>
<p>drive01은 selector 실행파일 준비 실패로 출발하지 않았습니다. drive02의 지연/200.9ms 계획 교체, drive06/11의 노면 OBB 급제동, drive09의67µs 미래 상태, drive10의입력 지연도 숨기지 않았습니다. 최초 근거리 보조 구간은3km/h로 제한되며 고속 완전 관측 수락이 아닙니다.</p>
<p>미래67µs 상태는 다음 시각을 발행 없이 잠시 기다리도록 수정하고 독립 ROS clock으로 재현했습니다. 미래 상태로 계산하지 않으며 지속 중단은 실패합니다. LiDAR fitting0.10m는 별도 비교 설정으로만 사용했습니다. 기본 지면 설정이나 길이/첫 관측 기반 객체 삭제는 변경하지 않았습니다.</p>
<p>다음은 낮은 실제 장애물과 노면 잔여점의 지면 근거 비교입니다. planner에서 모든 긴 객체·신규 객체를 무시하면 실제 barrier/truck도 놓칠 수 있습니다. <a href="../../local2c-20261028-ground02/index.html">실제 barrier 보존 대조·자료 제한</a></p>""",
        encoding="utf-8",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("baseline", type=Path)
    p.add_argument("roots", nargs="+", type=Path)
    a = p.parse_args()
    build(a.output, a.baseline, a.roots)
