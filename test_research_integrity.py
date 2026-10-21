"""Read-only checks for the published *simulation artifact*, not flight certification.
These tests never retune the frozen controllers and never generate a new held-out set.
"""
import csv
import hashlib
import importlib
import json
import math
import sys
import unittest
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "frozen_evidence"
SRC = DATA / "frozen_source"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FrozenEvidenceTests(unittest.TestCase):
    def test_frozen_sources_match_commitment(self):
        recorded = json.loads((DATA / "SEED_COMMITMENT.json").read_text(encoding="utf-8"))
        for relative, expected in recorded["frozen_files_sha256"].items():
            path = SRC / relative
            with self.subTest(file=relative):
                self.assertTrue(path.is_file())
                self.assertEqual(digest(path), expected)

    def test_final_conditions_unchanged(self):
        manifest = json.loads((DATA / "execution_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["N"], 100)
        self.assertEqual(manifest["controllers"], 4)
        self.assertEqual(manifest["final_trial_rows"], 400)
        self.assertEqual(digest(DATA / "final_conditions_FROZEN.csv"), manifest["conditions_sha256"])

    def test_100_paired_cases_and_controller_pass_rates(self):
        expected = {
            "Carefully tuned conventional PID": 95,
            "Performance Auto-PID": 95,
            "Robust Auto-PID": 93,
            "Actuator-aware Integral LQR": 99,
        }
        with (DATA / "final_400_paired_controller_trials.csv").open(newline="", encoding="utf-8") as f:
            data = list(csv.DictReader(f))
        self.assertEqual(len(data), 400)
        cases = defaultdict(set)
        passed = Counter()
        for row in data:
            cases[row["case_id"]].add(row["controller"])
            self.assertIn(row["altitude_pass"], {"True", "False"})
            passed[row["controller"]] += int(row["altitude_pass"] == "True")
            for metric in ("settling_s", "overshoot_pct", "altitude_IAE_m_s"):
                self.assertTrue(math.isfinite(float(row[metric])))
        self.assertEqual(len(cases), 100)
        self.assertTrue(all(names == set(expected) for names in cases.values()))
        self.assertEqual(dict(passed), expected)


class ModelSanityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        cls.np = np
        sys.path.insert(0, str(SRC / "reference_model"))
        cls.model = importlib.import_module("v2_lqr_development")
        cls.alloc = importlib.import_module("measurement_allocator_audit")

    def test_hover_equilibrium_at_nominal_mass(self):
        np, m = self.np, self.model
        omega = math.sqrt(m.M_NOM * m.G / (4 * m.KF))
        self.assertLess(omega, m.OMEGA_MAX)
        x = np.zeros(17)
        x[6] = 1.0
        x[13:17] = omega
        delta = m.dynamics(x, np.full(4, omega), m.M_NOM,
                           0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0)
        self.assertLess(float(np.max(np.abs(delta))), 1e-10)

    def test_collective_priority_allocation(self):
        np, m, alloc = self.np, self.model, self.alloc
        for thrust in (-3.0, 0.0, 8.0, 14.715, 30.0, 48.0, 55.0):
            for torque in (-4.0, -0.5, 0.0, 0.5, 4.0):
                with self.subTest(thrust=thrust, torque=torque):
                    omega, rotor_forces = alloc.allocate_priority(thrust, torque)
                    self.assertTrue(np.all(np.isfinite(omega)))
                    self.assertTrue(np.all(rotor_forces >= -1e-9))
                    self.assertTrue(np.all(rotor_forces <= 12.0 + 1e-9))
                    self.assertTrue(np.all(omega <= m.OMEGA_MAX + 1e-9))
                    self.assertAlmostEqual(float(np.sum(rotor_forces)),
                                           min(48.0, max(0.0, thrust)), places=9)
                    self.assertLessEqual(abs(float(m.ARM * (rotor_forces[1] - rotor_forces[3]))),
                                         abs(torque) + 1e-9)


if __name__ == "__main__":
    unittest.main()
