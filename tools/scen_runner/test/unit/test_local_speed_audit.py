import json
import pytest
from scen_runner.records.analyze_local_speed import analyze
from scen_runner.records.local_campaign import build


def write_rows(root, name, rows):
    (root / name).write_text("\n".join(map(json.dumps, rows)) + "\n")


def test_fixed_native_window_keeps_braking_failures(tmp_path):
    write_rows(
        tmp_path,
        "events.jsonl",
        [
            dict(event="native_sample", ros_time=t, ego=dict(speed_kmh=v))
            for t, v in [(100, 0), (109, 10), (110, 15), (115, 0), (120, 15)]
        ],
    )
    write_rows(
        tmp_path,
        "control.jsonl",
        [
            dict(
                decision_stamp=t,
                reason=reason,
                command_published=True,
                brake=brake,
                valid=False,
            )
            for t, brake, reason in [
                (99, 1, "preparation"),
                (110, 0, "ok"),
                (115, 1, "local_source_stale"),
                (120, 0, "ok"),
            ]
        ],
    )
    (tmp_path / "trial-result.json").write_text(json.dumps(dict(cap_kph=15)))
    result = analyze(tmp_path)
    assert result["steady_samples"] == 3
    assert result["steady_cap_error_kph"]["max"] == 15
    assert result["hard_brake_ticks"] == 1
    assert "preparation" not in result["control_reasons"]
    assert result["control_reasons"]["local_source_stale"] == 1
    with pytest.raises(FileExistsError):
        analyze(tmp_path)


def test_campaign_keeps_no_drive_setup_failure(tmp_path):
    (tmp_path / "registration.json").write_text("{}")
    trial = tmp_path / "trial"
    trial.mkdir()
    (trial / "trial-result.json").write_text(json.dumps(dict(reason="setup_failed")))
    write_rows(trial, "events.jsonl", [dict(event="setup_failure")])
    output = tmp_path / "campaign"
    build(output, [trial], title="LOCAL-2C test")
    assert json.loads((output / "results.json").read_text()) == []
    aborted = json.loads((output / "aborted.json").read_text())
    assert aborted[0]["reason"] == "no_native_drive"
    assert aborted[0]["result"]["reason"] == "setup_failed"
