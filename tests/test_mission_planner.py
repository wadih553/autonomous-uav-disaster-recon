import math
import os
import sys
import unittest

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ground_station", "server"))
sys.path.insert(0, SERVER_DIR)

from mission_planner import MissionPlanner


class MissionPlannerTests(unittest.TestCase):
    def setUp(self):
        self.planner = MissionPlanner()

    def test_scan_mission_has_expected_waypoint_count_and_validates(self):
        mission = self.planner.build_scan_mission(
            center_lat=33.8938,
            center_lon=35.5018,
            scan_radius_m=50,
            cruise_altitude_m=30,
            scan_altitude_m=15,
            num_points=12,
        )
        self.assertEqual(len(mission["waypoints"]), 15)
        self.assertTrue(self.planner.validate(mission)[0])

    def test_generated_points_are_near_requested_radius(self):
        lat, lon = 33.8938, 35.5018
        mission = self.planner.build_scan_mission(lat, lon, 100, 30, 15, 8)
        for waypoint in mission["waypoints"][1:9]:
            self.assertTrue(-90 <= waypoint["lat"] <= 90)
            self.assertTrue(-180 <= waypoint["lon"] <= 180)
            self.assertTrue(math.isfinite(waypoint["alt"]))

    def test_rejects_non_object(self):
        self.assertFalse(self.planner.validate(None)[0])
        self.assertFalse(self.planner.validate([])[0])

    def test_rejects_nan_and_infinity(self):
        mission = {"mission_id": "test", "waypoints": [{"seq": 0, "lat": float("nan"), "lon": 0, "alt": 10}]}
        self.assertFalse(self.planner.validate(mission)[0])
        mission["waypoints"][0]["lat"] = 0
        mission["waypoints"][0]["alt"] = float("inf")
        self.assertFalse(self.planner.validate(mission)[0])

    def test_rejects_bad_waypoint_and_coordinate_ranges(self):
        self.assertFalse(self.planner.validate({"mission_id": "x", "waypoints": ["bad"]})[0])
        mission = {"mission_id": "x", "waypoints": [{"seq": 0, "lat": 91, "lon": 0, "alt": 5}]}
        self.assertFalse(self.planner.validate(mission)[0])

    def test_rejects_too_many_waypoints(self):
        waypoints = [{"seq": i, "lat": 0, "lon": 0, "alt": 1} for i in range(501)]
        self.assertFalse(self.planner.validate({"mission_id": "x", "waypoints": waypoints})[0])

    def test_scan_generation_rejects_invalid_parameters(self):
        invalid = [
            (91, 0, 10, 20, 10, 12),
            (0, 181, 10, 20, 10, 12),
            (0, 0, 0, 20, 10, 12),
            (0, 0, 10, -1, 10, 12),
            (0, 0, 10, 20, 10, 2),
            (0, 0, 10, 20, 10, 101),
            (float("nan"), 0, 10, 20, 10, 12),
        ]
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.planner.build_scan_mission(*args)


if __name__ == "__main__":
    unittest.main()
