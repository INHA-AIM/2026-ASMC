"""Plot existing native ground-fit audit results without running a new detector.

Input points are LEVELED; base_from_input is used once for the top view. Never
label below-plane distance as ground truth or modify the frozen source run.
"""

import argparse
import html
import json
from pathlib import Path
import numpy as np


def build(source, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if output.exists():
        raise FileExistsError(output)
    output.mkdir()
    old = json.loads((source / "replay16-focus.json").read_text())
    audit = output.parent
    before = json.loads((audit / "ground-band-0.json").read_text())
    after = json.loads((audit / "ground-band-.1.json").read_text())
    xyz = np.genfromtxt(
        source / "stages16/1793189383321630716.csv", delimiter=",", names=True
    )
    points = np.c_[xyz["x"], xyz["y"], xyz["z"]]
    transform = np.asarray(old["base_from_input"]).reshape(4, 4)
    base = points @ transform[:3, :3].T + transform[:3, 3]
    stats = []
    for i in (3, 4, 5, 6):
        pick = xyz["cluster"] == i

        def distance(model):
            c = np.asarray(model["coefficients"])
            return points[pick] @ c[:3] + c[3]

        stats.append(
            dict(
                cluster=i,
                samples=int(pick.sum()),
                base_z_span_m=float(np.ptp(base[pick, 2])),
                old_signed_distance_m=np.percentile(
                    distance(before), [0, 50, 100]
                ).tolist(),
                new_signed_distance_m=np.percentile(
                    distance(after), [0, 50, 100]
                ).tolist(),
                kept_after=next(c["kept"] for c in after["clusters"] if c["id"] == i),
            )
        )
    (output / "evidence.json").write_text(
        json.dumps(dict(before=before, after=after, clusters=stats), indent=2)
    )
    fig, axes = plt.subplots(1, 2, figsize=(12, 7))
    selected = (base[:, 0] > 0) & (base[:, 0] < 16) & (abs(base[:, 1]) < 6)
    axes[0].scatter(-base[selected, 1], base[selected, 0], s=2, c="#bbbbbb")
    for i, c in zip((3, 4, 5, 6), ("#d84f38", "#e69700", "#a04cbe", "#dc285c")):
        pick = xyz["cluster"] == i
        axes[0].scatter(-base[pick, 1], base[pick, 0], s=8, c=c, label=f"cluster {i}")
        axes[1].scatter(points[pick, 0], points[pick, 2], s=6, c=c)
    axes[0].set(
        xlabel="screen right = vehicle right [m]",
        ylabel="forward base_link [m]",
        title="Original floor-strip clusters (base_link)",
    )
    axes[0].set_aspect("equal")
    axes[0].legend()
    x = np.linspace(4, 15, 100)
    y = -1.7
    for model, color, label in (
        (before, "#c72b2b", "Original fit band0.4m"),
        (after, "#187044", "Audit fit band0.1m"),
    ):
        a, b, c, d = model["coefficients"]
        z = -(a * x + b * y + d) / c
        axes[1].plot(x, z, c=color, label=label)
    axes[1].set(
        xlabel="leveled sensor x [m]",
        ylabel="leveled sensor z [m]",
        title="Plane section at y=-1.7m (not independent GT)",
    )
    axes[1].legend()
    axes[1].grid()
    fig.tight_layout()
    fig.savefig(output / "planes.png", dpi=150)
    plt.close(fig)
    rows = "".join(
        f'<tr><td>{r["cluster"]}</td><td>{r["samples"]}</td><td>{r["base_z_span_m"]:.3f}</td><td>{r["old_signed_distance_m"][1]:.3f}</td><td>{r["new_signed_distance_m"][1]:.3f}</td><td>{r["kept_after"]}</td></tr>'
        for r in stats
    )
    (output / "index.html").write_text(
        """<!doctype html><meta charset="utf-8"><title>LOCAL-2C 지면 잔여 OBB 원인</title><style>body{font:17px system-ui;max-width:1200px;margin:30px auto;background:#172333;color:#eee;padding:20px}a{color:#70cfff}img{width:100%}td,th{padding:8px;text-align:left}p{line-height:1.6}</style><h1>노면의 긴 OBB: 길이 필터 대신 지면 추정 대조</h1>
<p>실제 실패 scan1793189383321630716. 이때 터널 필터는 실행되지 않았습니다. 넓은0.4m fitting 합의 범위로 추정한 평면 아래의 노면 점이 절댓값 거리 제거를 통과했습니다. 차량 좌표에서는 높이 변화가 작은 점들이 수평화 좌표의 OBB로 묶였습니다.</p>
<p>빨강/초록 평면은 독립 GT가 아니라 동일 입력의 두 native 추정값입니다. 제거 범위0.30–0.40m는 유지하고 fitting만0.10m로 분리한 비교입니다. 단순 길이 제한·첫 관측 무시·모든 평면 아래 점 삭제는 추가하지 않았습니다.</p><img src="planes.png"><table><tr><th>원본 군집</th><th>점 수</th><th>base Z범위(m)</th><th>기존 평면 잔차 중앙(m)</th><th>후보 잔차 중앙(m)</th><th>후보 잔류 점</th></tr>"""
        + rows
        + """</table>
<p>네 군집의262점은 모두 제거됐고 같은 scan의 양의 높이 군집7–12의1,711점은 보존됐습니다. 이 수치를 실제 모든 장애물 재현율로 해석하지 않습니다. 전체 replay에서는 다른 점/군집도 늘어나므로 box·drum·얇은 barrier 보존 및 벽 혼입을 추가 검수해야 합니다. 운영 fitting 기본값은 변경하지 않았습니다.</p><a href="evidence.json">계수·수량 원문</a> · <a href="../../local2b-20261028-perception-handoff01/index.html">기존 재호 인계 자료</a>""",
        encoding="utf-8",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    build(a.source, a.output)
