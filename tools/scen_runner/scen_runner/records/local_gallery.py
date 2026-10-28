"""Source-exact LOCAL-1 semantic/RGB/path gallery, read-only bag analysis."""

import argparse
from collections import Counter
import html
import json
import math
from pathlib import Path


def select_cases(rows, controls=(), limit=24):
    """Keep failures and their exact controlling plan, then periodic context.

    Selection never pairs a control failure with an unrelated nearest image.
    Preparation failures without a complete semantic stamp remain in raw logs.
    """
    by_sequence = {r["plan_sequence"]: r for r in rows}
    chosen = {}
    seen = set()

    def add(row, why):
        stamp = row.get("scene_stamp_ns")
        if stamp and len(chosen) < limit:
            chosen.setdefault(stamp, (row, why))

    for row in controls:
        reason = row["reason"]
        key = ("control", reason)
        if (
            reason not in ("ok", "plan_replacement_pending", "motion_sample_pending")
            and key not in seen
        ):
            plan = by_sequence.get(row.get("plan_sequence"))
            if plan:
                add(plan, "제어: " + reason)
                seen.add(key)
    for row in rows:
        reason = row["reason"]
        if reason not in seen:
            add(row, "계획: " + reason)
            seen.add(reason)
    for row in sorted(
        (r for r in rows if not r.get("valid") and r.get("path")),
        key=lambda r: r.get("selected_length_m", 100),
    )[:3]:
        add(row, "가장 짧아진 실패 경로")
    # Include the shortest usable paths, even when their reason remains ok.
    for row in sorted(
        (r for r in rows if r.get("valid")),
        key=lambda r: r.get("selected_length_m", 100),
    )[:3]:
        add(row, "짧은 경로 검수")
    for row in rows[:: max(1, len(rows) // 10)]:
        add(row, "주기 표본")
    return chosen


def gallery(root, name="local-gallery", output=None, bag_root=None):
    import cv2
    import numpy as np
    import rosbag
    from .analyze_semantic import raster

    root = Path(root)
    out = Path(output) if output else root / name
    if output and not any((p / "registration.json").is_file() for p in out.parents):
        raise ValueError("external gallery output needs a new registered run")
    out.mkdir(exist_ok=False)
    rows = [json.loads(line) for line in (root / "decisions.jsonl").open()]
    controls = (
        [json.loads(line) for line in (root / "control.jsonl").open()]
        if (root / "control.jsonl").exists()
        else []
    )
    selection = select_cases(rows, controls)
    selected = {stamp: pair[0] for stamp, pair in selection.items()}
    prefix = "/experimental/shadow/semantic_space_trial/"
    topics = {
        prefix + "context": "context",
        prefix + "coverage": "coverage",
        prefix + "drivable_probability": "probability",
    }
    frames = {s: {} for s in selected}
    rgb = {}
    needed = set()
    bags = sorted(Path(bag_root or root).glob("sensors*.bag"))
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            for topic, m, _ in bag.read_messages(topics=list(topics)):
                stamp = m.header.stamp.to_nsec()
                if stamp not in frames:
                    continue
                frames[stamp][topics[topic]] = m
                if topics[topic] == "context":
                    i = list(m.views).index("front")
                    needed.add(m.image_headers[i].stamp.to_nsec())
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            for _, m, _ in bag.read_messages(topics=["/image_jpeg/compressed"]):
                stamp = m.header.stamp.to_nsec()
                if stamp in needed:
                    rgb[stamp] = m.data
    cases = []
    missing = Counter()
    for stamp, row in selected.items():
        f = frames[stamp]
        if set(f) != set(topics.values()):
            missing["incomplete_exact_bundle"] += 1
            continue
        context = f["context"]
        j = list(context.views).index("front")
        source = context.image_headers[j].stamp.to_nsec()
        if source not in rgb:
            missing["missing_exact_rgb"] += 1
            continue
        p, c = [
            np.frombuffer(f[k].data, np.uint8).reshape(f[k].height, f[k].step)[
                :, : f[k].width
            ]
            for k in ("probability", "coverage")
        ]
        image = cv2.resize(
            raster(p, c), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST
        )
        debug = row.get("scene", {})
        if "rotation" in debug:
            R = np.asarray(debug["rotation"]).reshape(3, 3)
            z = debug["ground_z"]
        else:
            # Historical trace lacked native R. This display-only inverse is
            # reconstructed from that exact context, never used by planning.
            a, b, offset = context.plane_coefficients[3 * j : 3 * j + 3]
            roll = math.atan2(-b, 1)
            pitch = math.atan2(a, math.sqrt(1 + b * b))
            cr, sr, cp, sp = (
                math.cos(roll),
                math.sin(roll),
                math.cos(pitch),
                math.sin(pitch),
            )
            R = np.array(
                [[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]]
            )
            z = offset / math.sqrt(1 + a * a + b * b)

        def pt(x, y):
            dx, dy, angle = debug.get("source_in_plan", [0, 0, 0])
            c, s = math.cos(angle), math.sin(angle)
            x, y = x - dx, y - dy
            bx, by, _ = R.T @ np.array([c * x + s * y, -s * x + c * y, z])
            return int(round((8 - by) * 30)), int(round((20 - bx) * 30))

        v = row.get("vehicle", dict(front=3.845, rear=0.79, width=2.160835))
        for rr, cc0, cc1 in debug.get("recent_road_runs", []):
            cv2.line(image, (cc0 * 3, rr * 3), (cc1 * 3, rr * 3), (190, 160, 50), 3)
        for b in debug.get("boundaries", []):
            if b["points"]:
                cv2.polylines(
                    image,
                    [np.asarray([pt(*q) for q in b["points"]], np.int32)],
                    False,
                    (80, 220, 255),
                    1,
                )
        if debug.get("home"):
            cv2.polylines(
                image,
                [np.asarray([pt(*q) for q in debug["home"]], np.int32)],
                False,
                (255, 180, 70),
                2,
            )
        for candidate in row.get("candidates", []):
            if candidate["path"]:
                cv2.polylines(
                    image,
                    [np.asarray([pt(q[0], q[1]) for q in candidate["path"]], np.int32)],
                    False,
                    (140, 110, 140),
                    1,
                )
        for x, y, yaw in row["path"][::5]:
            cs, sn = math.cos(yaw), math.sin(yaw)
            poly = [
                pt(x + cs * a - sn * b, y + sn * a + cs * b)
                for a, b in [
                    (-v["rear"], -v["width"] / 2),
                    (v["front"], -v["width"] / 2),
                    (v["front"], v["width"] / 2),
                    (-v["rear"], v["width"] / 2),
                ]
            ]
            cv2.polylines(image, [np.asarray(poly, np.int32)], True, (220, 190, 60), 1)
        if row["path"]:
            cv2.polylines(
                image,
                [np.asarray([pt(q[0], q[1]) for q in row["path"]], np.int32)],
                False,
                (255, 255, 0),
                2,
            )
        for ob in debug.get("obstacles", []):
            cv2.polylines(
                image,
                [np.asarray([pt(*q) for q in ob["polygon"]], np.int32)],
                True,
                (0, 100, 255),
                2,
            )
        stem = "case-%02d" % len(cases)
        cv2.imwrite(str(out / (stem + ".png")), image)
        (out / (stem + ".jpg")).write_bytes(rgb[source])
        (out / (stem + ".json")).write_text(json.dumps(row, indent=2))
        cases.append(
            dict(
                stem=stem,
                scene_stamp_ns=stamp,
                front_stamp_ns=source,
                reason=row["reason"],
                selection=selection[stamp][1],
                recovery=row.get("recovery", False),
                plan_sequence=row["plan_sequence"],
            )
        )
    summary = dict(
        cases=cases,
        missing=dict(missing),
        interpretation="Exact RGB/semantic stamp; projected path is proposed, not proof of executed path or collision clearance.",
    )
    (out / "index.json").write_text(json.dumps(summary, indent=2))
    cards = "".join(
        "<article><h2>"
        + html.escape(c["reason"])
        + " · plan "
        + str(c["plan_sequence"])
        + "</h2><p>"
        + html.escape(c["selection"])
        + '</p><img width="640" src="'
        + c["stem"]
        + '.jpg"><img width="360" src="'
        + c["stem"]
        + '.png"><p><a href="'
        + c["stem"]
        + '.json">판단 원문</a></p></article>'
        for c in cases
    )
    (out / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><title>LOCAL 경로·실패 검수</title><style>body{background:#141a22;color:#eee;font:16px sans-serif;margin:24px}img{vertical-align:top;max-width:100%}article{border-top:1px solid #555;margin-top:24px}</style><h1>'
        + html.escape(root.name)
        + '</h1><p>청록=선택 경로, 옅은 보라=후보, 파랑=우측 본차로 기준, 노란 외곽=차체 sweep. 청회색 셀=과거 실제 관측 노면(현재 프레임 추론 아님). 초록=노면 확률, 빨강=비노면/가림 그림자, 보라=차체 가림, 갈색=시야 밖. 회복 경로는 충돌 무결성이 검증된 경로가 아닙니다. 실제 진행·접촉은 events/trial-result와 대조합니다.</p><a href="index.json">시각 대응·누락</a>'
        + cards,
        encoding="utf-8",
    )
    print(json.dumps(dict(cases=len(cases), missing=dict(missing))))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root")
    p.add_argument("--name", default="local-gallery")
    p.add_argument("--output")
    p.add_argument("--bag-root")
    a = p.parse_args()
    gallery(a.root, a.name, a.output, a.bag_root)
