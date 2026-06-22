import unittest

from mission_logic import (
    parse_targets,
    prune_stale_targets,
    select_acquisition_candidate,
    select_priority_target,
    TargetSnapshot,
)


class Candidate:
    def __init__(self, camera, distance):
        self.camera = camera
        self.distance = distance


class MissionLogicTest(unittest.TestCase):
    def test_parse_targets_falls_back_to_defaults(self):
        self.assertEqual(
            parse_targets("")[:4],
            ["person", "traffic cone", "grey barrel", "blue barrel"],
        )
        self.assertIn("box", parse_targets(""))
        self.assertIn("Ladder", parse_targets(""))

    def test_select_priority_target_skips_cooling_target(self):
        visible = {
            "person": TargetSnapshot("person", 10.0),
            "blue barrel": TargetSnapshot("blue barrel", 10.0),
        }
        target, cooling = select_priority_target(
            ["person", "blue barrel"],
            visible,
            {"person": 20.0},
            12.0,
        )

        self.assertEqual(target.class_name, "blue barrel")
        self.assertEqual(cooling, [("person", 20.0)])

    def test_prune_stale_targets_removes_old_entries(self):
        visible = {
            "fresh": TargetSnapshot("fresh", 9.0),
            "stale": TargetSnapshot("stale", 1.0),
        }

        pruned = prune_stale_targets(visible, now_sec=10.0, timeout_sec=5.0)

        self.assertEqual(set(pruned), {"fresh"})

    def test_select_acquisition_candidate_prefers_right_on_near_tie(self):
        selected = select_acquisition_candidate(
            [
                Candidate("left", 1.00),
                Candidate("right", 1.04),
                Candidate("back", 2.00),
            ],
            ("right", "back", "left"),
            0.05,
        )

        self.assertEqual(selected.camera, "right")


if __name__ == "__main__":
    unittest.main()
