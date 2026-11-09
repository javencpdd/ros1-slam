"""Directed tests for the EVT gate model and the blank parameter sheet.

All offline.  No hardware, no ROS, no topic writes, no parameter changes.
The real person authorized ENTERING EVT; these tests verify that entry does
not silently release the inner L2 gates.
"""

import os
import unittest

import evt_gates as e

SHEET = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "configs", "lite3", "SIM_TO_REAL_PARAM_SHEET.yaml.template",
)

G0_EVIDENCE = {
    "vendor_chain_contract": True,
    "shadow_readiness_gate": True,
    "directed_tests": True,
}


def _sheet_text():
    with open(SHEET, "r", encoding="utf-8") as fh:
        return fh.read()


class EvtGateTests(unittest.TestCase):

    def test_selftest_passes(self):
        self.assertTrue(e._selftest())

    def test_entry_token_is_explicit_and_not_inferred(self):
        self.assertTrue(e.entry_authorized(e.ENTRY_TOKEN))
        self.assertTrue(e.entry_authorized({"evt_entry", "other"}))
        self.assertFalse(e.entry_authorized(None))
        self.assertFalse(e.entry_authorized("EVT"))
        self.assertFalse(e.entry_authorized({"work_window"}))

    def test_entry_does_not_release_inner_l2_gates(self):
        s = e.stage_status(G0_EVIDENCE, e.ENTRY_TOKEN)
        self.assertTrue(s["stage_entered"])
        self.assertEqual(s["reached"], "G0")
        self.assertEqual(s["blocked_at"], "G1")
        self.assertTrue(s["entry_is_not_gate_release"])
        self.assertEqual(s["authorized_at"], e.AUTHORIZED_AT)

    def test_empty_evidence_refused_not_green(self):
        s = e.stage_status({}, e.ENTRY_TOKEN)
        self.assertIsNone(s["reached"])
        self.assertEqual(s["gates"][0]["state"], "insufficient_data")

    def test_l2_gate_needs_explicit_release_token(self):
        ev = dict(G0_EVIDENCE, param_sheet_template=True, param_sheet_signed=True)
        self.assertEqual(e.evaluate_gate("G1", ev)["state"], "blocked")
        ev["release:G1"] = True
        self.assertEqual(e.evaluate_gate("G1", ev)["state"], "open")

    def test_injection_and_reverse_control(self):
        ev = dict(G0_EVIDENCE, param_sheet_template=True,
                  param_sheet_signed=True, **{"release:G1": True})
        self.assertEqual(e.stage_status(ev, e.ENTRY_TOKEN)["reached"], "G1")
        ev.pop("release:G1")
        self.assertEqual(e.stage_status(ev, e.ENTRY_TOKEN)["reached"], "G0")

    def test_withdrawing_entry_flips_stage_flag_but_not_gate_states(self):
        ev = dict(G0_EVIDENCE, param_sheet_template=True,
                  param_sheet_signed=True, **{"release:G1": True})
        a = e.stage_status(ev, e.ENTRY_TOKEN)
        b = e.stage_status(ev, None)
        self.assertTrue(a["stage_entered"])
        self.assertFalse(b["stage_entered"])
        self.assertEqual(a["reached"], b["reached"])

    def test_unknown_gate_refused(self):
        with self.assertRaises(ValueError):
            e.evaluate_gate("G9", G0_EVIDENCE)

    def test_sheet_is_blank_and_guarded(self):
        rep = e.audit_param_sheet(_sheet_text())
        self.assertEqual(rep["state"], "clean")
        self.assertGreaterEqual(rep["real_value_count"], 20)

    def test_sheet_injection_effectiveness_and_reverse_control(self):
        text = _sheet_text()
        dirty = text.replace("    real_value: null\n",
                             "    real_value: 0.16\n", 1)
        self.assertEqual(e.audit_param_sheet(dirty)["state"], "backfill_detected")
        self.assertEqual(e.audit_param_sheet(text)["state"], "clean")

    def test_sheet_refuses_empty_and_detects_missing_guard(self):
        self.assertEqual(e.audit_param_sheet("")["state"], "insufficient_data")
        broken = _sheet_text().replace("effective: false", "effective: true")
        self.assertEqual(e.audit_param_sheet(broken)["state"], "drift")

    def test_sheet_has_no_vendor_numbers_from_official_stack(self):
        text = _sheet_text()
        # The official values are read-only reference facts, never sheet values.
        for forbidden in ("0.47", "1.57", "ndt_resolution: 1.5",
                          "inflation_radius: 0.40", "max_vel_x: 1.0"):
            self.assertNotIn(forbidden, text)

    def test_all_six_gates_declared_with_authority(self):
        self.assertEqual(len(e.GATES), 6)
        self.assertEqual([g["id"] for g in e.GATES],
                         ["G0", "G1", "G2", "G3", "G4", "G5"])
        for g in e.GATES:
            self.assertIn(g["authority"], ("self", "L2"))
            self.assertTrue(g["requires"])


if __name__ == "__main__":
    unittest.main()
