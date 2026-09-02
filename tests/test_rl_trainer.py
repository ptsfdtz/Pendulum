import importlib.util
import sys
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "tools" / "train_balance_policy.py"
SPEC = importlib.util.spec_from_file_location("train_balance_policy", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class RewardTests(unittest.TestCase):
    def test_upright_centered_is_better(self):
        ideal = MODULE.reward([0.0, 0.0, 0.0, 0.0], 0.0, 0.0)
        displaced = MODULE.reward([0.08, 0.0, 0.5, 0.0], 0.0, 0.0)
        self.assertGreater(ideal, displaced)

    def test_speed_and_aggressive_action_are_penalized(self):
        calm = MODULE.reward([0.0, 0.0, 0.0, 0.0], 0.05, 0.05)
        violent = MODULE.reward([0.0, 1.0, 0.0, 1.0], 0.35, -0.35)
        self.assertGreater(calm, violent)

    def test_policy_action_is_bounded(self):
        action = MODULE.policy_action([100.0, 0.0, 0.0, 0.0, 0.0],
                                      [1.0, 0.0, 0.0, 0.0], 0.35)
        self.assertEqual(action, 0.35)

    def test_partial_csv_message_is_ignored(self):
        self.assertEqual(MODULE.parse_message(None), {})


if __name__ == "__main__":
    unittest.main()
