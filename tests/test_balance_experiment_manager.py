import importlib.util
import io
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

TOOLS = Path(__file__).parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location(
    "run_balance_experiments", TOOLS / "run_balance_experiments.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ExperimentManagerTests(unittest.TestCase):
    def test_candidate_changes_only_one_bounded_parameter(self):
        best = {"kp": 30.0, "kd": 5.0, "kx": 0.03, "kv": 0.18,
                "ki": 0.0}
        candidate, proposal = MODULE.candidate_for(0, best)
        self.assertEqual(proposal, "kp+")
        self.assertEqual(sum(candidate[key] != best[key] for key in best), 1)
        for key, value in candidate.items():
            low, high = MODULE.BOUNDS[key]
            self.assertGreaterEqual(value, low)
            self.assertLessEqual(value, high)

    def test_failure_is_penalized(self):
        sample = {"sample": 10.0, "theta_rad": 0.0, "theta_dot_rad_s": 0.0,
                  "cart_position_half_travel": 0.0,
                  "cart_velocity_half_travel_s": 0.0,
                  "rated_torque_fraction": 0.0, "sample_interval_us": 3000.0}
        failed = MODULE.score_samples([sample], "pendulum_fall", 15.0)
        self.assertFalse(failed.success)
        self.assertLess(failed.score, -100.0)

    def test_rl_candidates_are_reproducible_bounded_and_joint(self):
        center = {"kp": 30.0, "kd": 5.0, "kx": 0.03,
                  "kv": 0.18, "ki": 0.0}
        state = MODULE.new_rl_state(center, 0.5)
        state["episode"] = 1
        first = MODULE.sample_rl_candidate(state)
        second = MODULE.sample_rl_candidate(state)
        self.assertEqual(first, second)
        self.assertGreaterEqual(sum(first[name] != state["mean"][name]
                                    for name in MODULE.PARAMETERS), 2)
        for name, value in first.items():
            low, high = MODULE.BOUNDS[name]
            self.assertGreaterEqual(value, low)
            self.assertLessEqual(value, high)

    def test_first_rl_episode_uses_exact_configured_baseline(self):
        center = {"kp": 30.0, "kd": 5.0, "kx": 0.03,
                  "kv": 0.18, "ki": 0.0}
        state = MODULE.new_rl_state(center)
        self.assertEqual(MODULE.sample_rl_candidate(state), center)

    def test_rl_replay_updates_after_reward(self):
        center = {"kp": 30.0, "kd": 5.0, "kx": 0.03,
                  "kv": 0.18, "ki": 0.0}
        state = MODULE.new_rl_state(center, None)
        metrics = MODULE.Metrics(0.8, 15.0, 500, 0.01, 0.02, 0.1,
                                 0.1, 0.05, 0.01, "timeout", True)
        MODULE.update_rl_state(state, center, metrics, "test_run")
        self.assertEqual(state["episode"], 1)
        self.assertEqual(len(state["replay"]), 1)
        self.assertEqual(state["best_score"], 0.8)

    def test_console_readiness_can_be_observed_before_writer_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.txt"
            path.write_text("starting\n", encoding="utf-8")

            def publish_prompt():
                time.sleep(0.05)
                with path.open("a", encoding="utf-8") as stream:
                    stream.write("pendulum> ")
                    stream.flush()

            writer = threading.Thread(target=publish_prompt)
            writer.start()
            self.assertTrue(MODULE.wait_for_event(path, "pendulum> ", 1.0))
            writer.join()

    def test_monitor_ignores_delayed_rows_from_previous_run(self):
        message = ("run_id={run_id},sample={sample},theta_rad={theta},theta_dot_rad_s=0,"
                   "cart_position_half_travel=0,"
                   "cart_velocity_half_travel_s=0,"
                   "rated_torque_fraction=0,sample_interval_us=3000")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "phase1.csv"
            path.write_text(
                "timestamp,level,component,message\n"
                f'0,INFO,BalanceTelemetry,"{message.format(run_id=1, sample=2530, theta=0.3)}"\n'
                f'1,INFO,BalanceTelemetry,"{message.format(run_id=2, sample=10, theta=0.3)}"\n',
                encoding="utf-8")

            class Process:
                stdin = io.StringIO()

                @staticmethod
                def poll():
                    return None

            termination, rows = MODULE.monitor_experiment(
                Process(), path, 1.0, run_id=2)
            self.assertEqual(termination, "pendulum_fall")
            self.assertEqual([row["sample"] for row in rows], [10.0])

    def test_start_line_run_id_is_parsed(self):
        line = ('0,INFO,ManualConsole,"Balance loop started: run_id=17,'
                ' frequency_hz=333, polarity=+1"')
        self.assertEqual(MODULE.run_id_from_start_line(line), 17)

    def test_rejection_detail_returns_latest_console_line(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.txt"
            path.write_text(
                "Balance start rejected: error_counts=51\n"
                "Balance start rejected: error_counts=43\n",
                encoding="utf-8")
            self.assertEqual(
                MODULE.last_line_containing(path, "Balance start rejected"),
                "Balance start rejected: error_counts=43")

if __name__ == "__main__":
    unittest.main()
