#!/usr/bin/env python3
"""Single source of truth for test counts: parses every gtest/rostest JUnit XML with ElementTree and counts
<testcase> elements (tests, failures, errors, skipped, disabled) directly; the same function produces the JSON that
the report quotes, and the <testsuites>/<testsuite> header attributes are cross-checked against the element counts.
usage: collect_tests.py OUT.json LABEL=DIR [LABEL=DIR ...]    (DIR searched recursively for *.xml and *_summary.tsv)
"""
import sys

sys.dont_write_bytecode = True

import glob, json, os  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402


def parse_xml(p):
    root = ET.parse(p).getroot()
    suites = [root] if root.tag == "testsuite" else root.findall("testsuite")
    cases = [tc for s in suites for tc in s.findall("testcase")]
    n = {"tests": len(cases), "failures": 0, "errors": 0, "skipped": 0, "disabled": 0, "failed_cases": []}
    for tc in cases:
        name = "%s.%s" % (tc.get("classname"), tc.get("name"))
        if tc.find("failure") is not None:
            n["failures"] += 1; n["failed_cases"].append(name)
        if tc.find("error") is not None:
            n["errors"] += 1; n["failed_cases"].append(name)
        if tc.find("skipped") is not None or tc.get("result") == "skipped":
            n["skipped"] += 1
        if tc.get("status") == "notrun" or (tc.get("name") or "").startswith("DISABLED_"):
            n["disabled"] += 1
    hdr = {k: root.get(k) for k in ("tests", "failures", "errors", "disabled", "skipped")}
    n["header_attributes"] = hdr
    try:
        ht = int(hdr["tests"]) if hdr["tests"] is not None else None
    except ValueError:
        ht = None
    # gtest counts DISABLED tests in <testsuites tests=...> but not as executed testcases with status=run
    n["header_consistent"] = ht is None or ht == n["tests"]
    return n


def main(out, *specs):
    res = {"groups": {}, "totals": {}}
    tot = {"xml_files": 0, "tests": 0, "failures": 0, "errors": 0, "skipped": 0, "disabled": 0, "header_inconsistent": 0}
    for spec in specs:
        label, d = spec.split("=", 1)
        files = sorted(glob.glob(os.path.join(d, "**", "*.xml"), recursive=True))
        grp = {"dir": d, "xml": {}, "summaries": {}}
        gt = dict.fromkeys(tot, 0)
        for p in files:
            try:
                n = parse_xml(p)
            except ET.ParseError as e:
                grp["xml"][os.path.relpath(p, d)] = {"parse_error": str(e)}
                continue
            grp["xml"][os.path.relpath(p, d)] = n
            gt["xml_files"] += 1
            for k in ("tests", "failures", "errors", "skipped", "disabled"):
                gt[k] += n[k]
            gt["header_inconsistent"] += 0 if n["header_consistent"] else 1
        for p in sorted(glob.glob(os.path.join(d, "**", "*_summary.tsv"), recursive=True)):
            grp["summaries"][os.path.relpath(p, d)] = open(p).read().splitlines()
        grp["totals"] = gt
        res["groups"][label] = grp
        for k in tot:
            tot[k] += gt[k]
    res["totals"] = tot
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps({k: v["totals"] for k, v in res["groups"].items()}, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:])
