import unittest

from usv_only_task3.reports import Task3ResultFactory


class Task3ResultFactoryTests(unittest.TestCase):
    def test_docking_and_firefighting_use_separate_oneof_bodies(self):
        factory = Task3ResultFactory("TONG", "usv", initial_seq=10)
        docking = factory.docking(2, now=0.0)
        firefighting = factory.firefighting(1, now=1.0)

        self.assertEqual(docking, {
            "team_id": "TONG",
            "vehicle_id": "usv",
            "seq": 11,
            "sent_at": "1970-01-01T00:00:00Z",
            "docking": {"bay_id": 2},
        })
        self.assertEqual(firefighting, {
            "team_id": "TONG",
            "vehicle_id": "usv",
            "seq": 12,
            "sent_at": "1970-01-01T00:00:01Z",
            "firefighting": {"window_id": 1},
        })
        self.assertNotIn("firefighting", docking)
        self.assertNotIn("docking", firefighting)

    def test_invalid_ids_are_rejected(self):
        factory = Task3ResultFactory()
        with self.assertRaises(ValueError):
            factory.docking(0, now=0.0)
        with self.assertRaises(ValueError):
            factory.firefighting(3, now=0.0)


if __name__ == "__main__":
    unittest.main()
