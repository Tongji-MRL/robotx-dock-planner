import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FullDockingSourceTests(unittest.TestCase):
    def test_explicit_state_machine_and_generated_targets(self):
        source = (ROOT / "scripts/gz_full_docking.cc").read_text(encoding="utf-8")
        for state in (
            "GO_TO_APPROACH",
            "ALIGN",
            "GO_TO_ENTRANCE",
            "GO_TO_STOP",
            "DONE",
            "FAILED",
        ):
            self.assertIn(state, source)
        for target in ("approach_pose", "entrance_pose", "stop_pose"):
            self.assertIn(f'JsonNumber(json, "{target}"', source)
        self.assertNotIn("-494.597168", source)
        self.assertNotIn("-488.113541", source)
        self.assertNotIn("-487.032936", source)

    def test_phase_parameters_and_single_speed_optimization(self):
        source = (ROOT / "scripts/gz_full_docking.cc").read_text(encoding="utf-8")
        for declaration in (
            "kApproachBaseThrust = 45.0",
            "kAlignMaxTurn = 12.0",
            "kEntranceBaseThrust = 12.0",
            "kStopBaseThrust = 6.0",
            "kStopZeroForwardDistance = 0.40",
        ):
            self.assertIn(declaration, source)


if __name__ == "__main__":
    unittest.main()
