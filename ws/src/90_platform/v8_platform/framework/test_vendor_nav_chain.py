"""Directed tests for the read-only vendor navigation chain adapter.

No ROS, no network, no real robot, no file writes.  Every test uses the
module's pure functions only.
"""

import copy
import unittest

import vendor_nav_chain as v


ALL_TRUE = {k: True for k in v.REQUIRED_SIGNALS}


class VendorNavChainTests(unittest.TestCase):

    # ---- default posture / zero regression ----

    def test_default_mode_is_off_and_never_blocks(self):
        r = v.initial_pose_gate(ALL_TRUE)
        self.assertEqual(r["mode"], "off")
        self.assertEqual(r["decision"], "allow")
        self.assertTrue(r["ready"])

    def test_shadow_only_reports_even_when_not_ready(self):
        s = dict(ALL_TRUE, map_ready=False)
        r = v.initial_pose_gate(s, "shadow")
        self.assertEqual(r["state"], "not_ready")
        self.assertEqual(r["decision"], "observe_only")

    def test_assist_is_the_only_mode_that_vetoes(self):
        s = dict(ALL_TRUE, localization_converged=False)
        self.assertEqual(v.initial_pose_gate(s, "off")["decision"], "allow")
        self.assertEqual(v.initial_pose_gate(s, "shadow")["decision"], "observe_only")
        self.assertEqual(v.initial_pose_gate(s, "assist")["decision"], "veto")

    def test_module_selftest_passes(self):
        self.assertTrue(v._selftest())

    # ---- empty-set refusal ----

    def test_empty_signal_set_is_refused_not_green(self):
        for signals in ({}, None, {k: None for k in v.REQUIRED_SIGNALS}):
            r = v.initial_pose_gate(signals, "assist")
            self.assertEqual(r["state"], "insufficient_data")
            self.assertEqual(r["decision"], "refuse")
            self.assertFalse(r["ready"])

    def test_partial_unknown_is_not_ready_but_is_judged(self):
        s = {"initial_pose_received": True, "map_ready": True}
        r = v.initial_pose_gate(s, "assist")
        self.assertEqual(r["state"], "not_ready")
        self.assertEqual(r["decision"], "veto")
        self.assertIn("localization_converged", r["missing"])

    def test_unknown_mode_is_refused_instead_of_defaulted(self):
        for bad in ("", None, "bogus", "ON"):
            with self.assertRaises(ValueError):
                v.resolve_mode(bad)

    # ---- injection effectiveness / reverse control ----

    def test_injection_flips_ready_and_restore_flips_back(self):
        injured = copy.deepcopy(ALL_TRUE)
        injured["odom_chain_ready"] = False
        self.assertEqual(v.initial_pose_gate(injured, "assist")["decision"], "veto")
        restored = dict(injured, odom_chain_ready=True)
        self.assertEqual(v.initial_pose_gate(restored, "assist")["decision"], "allow")

    def test_no_backfill_detects_leak_and_reverse_control_clears_it(self):
        base = {k: "project-own" for k in v.REQUIRED_SIGNALS}
        base["inflation_radius"] = 0.2
        self.assertEqual(v.no_backfill_report(base)["state"], "clean")
        leaked = dict(base, inflation_radius=v.VENDOR_INTERFACES["inflation_radius"]["value"])
        rep = v.no_backfill_report(leaked)
        self.assertEqual(rep["state"], "backfill_detected")
        self.assertIn("inflation_radius", rep["leaks"])
        self.assertEqual(v.no_backfill_report(dict(base))["state"], "clean")

    def test_no_backfill_refuses_empty_settings(self):
        self.assertEqual(v.no_backfill_report({})["state"], "insufficient_data")

    # ---- vendor facts are read-only and structurally sane ----

    def test_vendor_extrinsics_are_dead_code_and_marked_no_copy(self):
        item = v.VENDOR_INTERFACES["base_link_to_rslidar_tf"]
        self.assertTrue(item["do_not_copy"])
        self.assertIn("dead code", item["reason"])
        self.assertIn("G8-2", item["reason"])

    def test_vendor_script_inventory_matches_observed_host(self):
        s1 = v.VENDOR_CHAIN[0]
        self.assertEqual(len(s1["scripts"]), 3)
        self.assertEqual(len(s1["scripts_present_on_host"]), 2)
        self.assertNotIn("start_rslidar.sh", s1["scripts_present_on_host"])

    def test_vendor_use_sim_time_internal_conflict_is_recorded(self):
        item = v.VENDOR_INTERFACES["use_sim_time"]
        self.assertTrue(item["do_not_copy"])
        self.assertIn("419", item["reason"])

    def test_all_do_not_copy_keys_have_a_reason(self):
        for key in v.NO_BACKFILL_KEYS:
            self.assertTrue(v.VENDOR_INTERFACES[key]["do_not_copy"])
            self.assertTrue(v.VENDOR_INTERFACES[key].get("reason"))

    def test_chain_diff_has_three_rows_with_a_step3_gap(self):
        rows = v.chain_diff()
        self.assertEqual(len(rows), 3)
        self.assertIn("no project-side startup gate", rows[2]["gap"])

    def test_report_is_read_only_and_lists_defaults(self):
        text = v.report_text()
        self.assertIn("READ-ONLY", text)
        self.assertIn("default mode = off", text)
        self.assertIn("DO NOT COPY", text)


if __name__ == "__main__":
    unittest.main()
