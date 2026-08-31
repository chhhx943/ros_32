import json
import os
import subprocess
import sys
import tempfile
import unittest


from tools.pid.model import PIDGains
from tools.pid.search import generate_candidates


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class PidSearchAndCliTest(unittest.TestCase):
    def test_seeded_candidates_are_repeatable_and_inside_bounds(self):
        bounds = {"kp": (0.0, 1.0), "ki": (0.0, 2.0), "kd": (0.0, 0.5)}

        first = list(generate_candidates(PIDGains(0.2, 0.6, 0.0), bounds, 4, seed=7))
        second = list(generate_candidates(PIDGains(0.2, 0.6, 0.0), bounds, 4, seed=7))

        self.assertEqual(first, second)
        self.assertEqual(first[0], PIDGains(0.2, 0.6, 0.0))
        for candidate in first:
            self.assertGreaterEqual(candidate.kp, 0.0)
            self.assertLessEqual(candidate.kp, 1.0)
            self.assertGreaterEqual(candidate.ki, 0.0)
            self.assertLessEqual(candidate.ki, 2.0)
            self.assertGreaterEqual(candidate.kd, 0.0)
            self.assertLessEqual(candidate.kd, 0.5)

    def test_cli_simulation_writes_report_and_sample_csv(self):
        with tempfile.TemporaryDirectory() as output_dir:
            result = subprocess.run(
                [sys.executable, os.path.join(ROOT, "tools", "pid", "autotune.py"),
                 "--transport", "sim", "--iterations", "2", "--seed", "7",
                 "--output-dir", output_dir],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertIn("best_pid", result.stdout)
            with open(os.path.join(output_dir, "report.json"), encoding="utf-8") as f:
                report = json.load(f)
            self.assertIn("current_pid", report)
            self.assertIn("candidate_pid", report)
            self.assertIn("best_pid", report)
            self.assertEqual(len(report["experiment_history"]), 2)
            self.assertTrue(os.path.exists(os.path.join(output_dir, "samples.csv")))


if __name__ == "__main__":
    unittest.main()
