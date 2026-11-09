"""EVT stage gate model for the Lite3 navigation real-robot adaptation.

The real person authorized ENTERING the EVT stage on 2026-11-09 16:17.  That
authorization unlocks the stage marker only.  It does NOT unlock the inner
gates, because note/10_真机上机Runbook.md:4 still requires an approved work
window and a single work order for each on-robot action, and :230 requires a
simulation-to-real parameter comparison sheet.  Entering a stage is not the
same as releasing every gate inside it.

Pure stdlib.  No ROS, no network, no hardware, no topic writes, no parameter
changes.  Every gate evaluation refuses an empty/unknown evidence set instead
of printing green.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re

AUTHORIZED_AT = "2026-11-09 16:17"
ENTRY_TOKEN = "evt_entry"

# Each gate lists the evidence keys required to open it and who may open it.
# authority="self"  -> non-L2, may be advanced by the driving tool (§9.211).
# authority="L2"    -> needs the real person's explicit per-item release.
GATES = (
    {
        "id": "G0",
        "name": "离线设计冻结",
        "authority": "self",
        "requires": (
            "vendor_chain_contract",
            "shadow_readiness_gate",
            "directed_tests",
        ),
        "note": "Read-only vendor interface contract + fail-closed shadow gate + tests.",
    },
    {
        "id": "G1",
        "name": "仿真→真机参数对照件就绪",
        "authority": "L2",
        "requires": (
            "param_sheet_template",
            "param_sheet_signed",
        ),
        "note": (
            "note/11 (sim-to-real migration rules draft, 2026-10-28) already "
            "exists; this machine-auditable sign-off skeleton complements it. "
            "Runbook:230 means no EFFECTIVE sheet with measured real values. "
            "Any real_value filled in is a real-robot backfill and needs L2 + "
            "a single work order; the gap is still open."
        ),
    },
    {
        "id": "G2",
        "name": "作业窗与单项作业单批准",
        "authority": "L2",
        "requires": (
            "work_window",
            "single_work_order",
            "estop_reachable",
            "watch_present",
        ),
        "note": "Runbook:4; mode_navigation.launch header prerequisite 1 and 3.",
    },
    {
        "id": "G3",
        "name": "现场预检与外参实测",
        "authority": "L2",
        "requires": (
            "preflight_rc0",
            "extrinsics_measured",
            "param_sheet_signed",
        ),
        "note": "G8-2 rejects an identity extrinsics matrix; values must be measured.",
    },
    {
        "id": "G4",
        "name": "真机执行",
        "authority": "L2",
        "requires": (
            "work_window",
            "single_work_order",
            "preflight_rc0",
            "enable_motion_release",
        ),
        "note": "boundary/protocol.py:3-4 keeps the QNX 43893 link untouched until L2.",
    },
    {
        "id": "G5",
        "name": "判据面签字与正式验收",
        "authority": "L2",
        "requires": (
            "criteria_signoff",
            "formal_acceptance",
        ),
        "note": "Official wording authority stays with the real person (§9.213).",
    },
)

GATE_BY_ID = {g["id"]: g for g in GATES}


def entry_authorized(authorization):
    """Only the explicit evt_entry token counts; nothing is inferred."""
    if authorization is None:
        return False
    if isinstance(authorization, str):
        return authorization == ENTRY_TOKEN
    return ENTRY_TOKEN in set(authorization)


def evaluate_gate(gate_id, evidence, authorization=None):
    """Return the state of one gate.  Empty evidence is refused, never green."""
    if gate_id not in GATE_BY_ID:
        raise ValueError("unknown gate id")
    gate = GATE_BY_ID[gate_id]
    if not evidence:
        return {
            "gate": gate_id,
            "state": "insufficient_data",
            "open": False,
            "authority": gate["authority"],
            "missing": list(gate["requires"]),
            "reasons": ["no evidence supplied; refusing to judge"],
        }

    present, unknown, false_items = [], [], []
    for key in gate["requires"]:
        value = evidence.get(key, None)
        if value is None:
            unknown.append(key)
        elif bool(value) is False:
            false_items.append(key)
        else:
            present.append(key)

    # A gate that needs L2 also needs an explicit per-gate release token.
    release_key = "release:" + gate_id
    if gate["authority"] == "L2":
        released = bool(evidence.get(release_key, False))
    else:
        released = True

    if len(present) == 0:
        return {
            "gate": gate_id,
            "state": "insufficient_data",
            "open": False,
            "authority": gate["authority"],
            "missing": list(unknown) + list(false_items),
            "reasons": ["no usable evidence; refusing to judge"],
        }

    missing = list(unknown) + list(false_items)
    if not released:
        missing.append(release_key)
    open_ = not missing
    return {
        "gate": gate_id,
        "state": "open" if open_ else "blocked",
        "open": open_,
        "authority": gate["authority"],
        "missing": missing,
        "reasons": (
            ["L2 per-gate release absent: " + release_key] if not released else []
        ),
        "entry_authorized": entry_authorized(authorization),
    }


def stage_status(evidence, authorization=None):
    """Highest reachable gate plus the full per-gate table.

    Only self-authority gates can ever be reported open without an explicit
    release token.  stage_entered is independent of gate states: it records
    the real person's authorization, not an execution permission.
    """
    rows = [evaluate_gate(g["id"], evidence, authorization) for g in GATES]
    reachable = []
    for row in rows:
        if not row["open"]:
            break
        reachable.append(row["gate"])
    return {
        "stage": "EVT",
        "stage_entered": entry_authorized(authorization),
        "authorized_at": AUTHORIZED_AT if entry_authorized(authorization) else None,
        "reached": reachable[-1] if reachable else None,
        "blocked_at": next((r["gate"] for r in rows if not r["open"]), None),
        "entry_is_not_gate_release": True,
        "gates": rows,
    }


# ---------------------------------------------------------------------------
# Blank simulation-to-real parameter comparison sheet auditor.
# note/11 already holds the human-readable migration rules draft (real values
# all "to be measured").  Runbook:230 means no EFFECTIVE sheet with measured
# real values -- this machine-auditable skeleton complements it, it does not
# invent the concept or close the gap.  Filling any real_value is an L2
# real-robot backfill.
# ---------------------------------------------------------------------------

SHEET_CATEGORIES = ("extrinsics_tf", "speed_limits", "criteria_gates", "map_version")
REAL_VALUE_RE = re.compile(r"^\s*real_value:\s*(.*?)\s*$")


def audit_param_sheet(text):
    """Zero-dependency literal audit of the blank sheet.

    Returns state = clean | insufficient_data | backfill_detected | drift.
    Empty input is refused.  A non-null real_value is a backfill (red).
    """
    if not text or not text.strip():
        return {"state": "insufficient_data", "reasons": ["empty sheet"]}
    lines = text.splitlines()
    joined = "\n".join(lines)
    if "status: TEMPLATE_ONLY" not in joined:
        return {"state": "drift", "reasons": ["status marker missing"]}
    if "effective: false" not in joined:
        return {"state": "drift", "reasons": ["effective:false guard missing"]}
    for cat in SHEET_CATEGORIES:
        if cat + ":" not in joined:
            return {"state": "drift", "reasons": ["missing category " + cat]}
    values = [m.group(1) for m in (REAL_VALUE_RE.match(l) for l in lines) if m]
    if not values:
        return {"state": "insufficient_data", "reasons": ["no real_value rows"]}
    bad = [v for v in values if v.strip().lower() not in ("null", "~", "")]
    if bad:
        return {
            "state": "backfill_detected",
            "real_value_count": len(values),
            "bad": bad,
            "reasons": ["a real_value is filled; that is an L2 backfill"],
        }
    return {
        "state": "clean",
        "real_value_count": len(values),
        "reasons": [],
    }


def _selftest():
    ok = True
    # 1. main: entry authorization alone does not open L2 gates.
    bare = {"vendor_chain_contract": True, "shadow_readiness_gate": True,
            "directed_tests": True}
    s = stage_status(bare, ENTRY_TOKEN)
    ok &= s["stage_entered"] is True
    ok &= s["reached"] == "G0"
    ok &= s["blocked_at"] == "G1"

    # 1b. no evidence -> refused, not green.
    ok &= stage_status({}, ENTRY_TOKEN)["reached"] is None
    ok &= evaluate_gate("G0", {}, ENTRY_TOKEN)["state"] == "insufficient_data"

    # 2. injection effectiveness: an explicit per-gate release opens G1.
    injected = dict(bare)
    injected.update({"param_sheet_template": True, "param_sheet_signed": True,
                     "release:G1": True})
    s2 = stage_status(injected, ENTRY_TOKEN)
    ok &= s2["reached"] == "G1" and s2["blocked_at"] == "G2"

    # 2b. param sheet audit uses the REAL template file, not an inline sample:
    # an inline sample can silently drift away from the artifact's format.
    sheet_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "configs", "lite3",
                              "SIM_TO_REAL_PARAM_SHEET.yaml.template")
    with open(sheet_path, "r", encoding="utf-8") as fh:
        sheet = fh.read()
    ok &= audit_param_sheet(sheet)["state"] == "clean"
    dirty = sheet.replace("real_value: null", "real_value: 0.16", 1)
    ok &= dirty != sheet  # injection must actually land
    ok &= audit_param_sheet(dirty)["state"] == "backfill_detected"
    ok &= audit_param_sheet("")["state"] == "insufficient_data"

    # 3. reverse control: remove the release -> stage falls back to G0.
    restored = copy.deepcopy(injected)
    restored.pop("release:G1")
    ok &= stage_status(restored, ENTRY_TOKEN)["reached"] == "G0"

    # 3b. remove the entry token -> stage_entered flips, gate states unchanged.
    no_entry = stage_status(injected, None)
    ok &= no_entry["stage_entered"] is False
    ok &= no_entry["reached"] == s2["reached"]

    # 3c. invalid gate id refused.
    try:
        evaluate_gate("G9", bare)
        ok = False
    except ValueError:
        pass

    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--authorization", default=None)
    args = ap.parse_args()
    if args.selftest:
        good = _selftest()
        print("SELFTEST %s" % ("OK" if good else "FAIL"))
        return 0 if good else 1
    print(json.dumps(stage_status({}, args.authorization),
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
