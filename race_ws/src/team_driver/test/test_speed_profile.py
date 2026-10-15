import math
import unittest

from team_driver.speed_profile import build_speed_profile


class TestSpeedProfile(unittest.TestCase):

    def setUp(self):
        self.path = [
            (math.cos(2 * math.pi * i / 40),
             math.sin(2 * math.pi * i / 40))
            for i in range(40)
        ]

    def test_profile_lengths(self):
        curvature, speeds = build_speed_profile(self.path)
        self.assertEqual(len(curvature), len(self.path))
        self.assertEqual(len(speeds), len(self.path))

    def test_speeds_are_positive_and_capped(self):
        _, speeds = build_speed_profile(self.path)
        self.assertTrue(all(math.isfinite(v) and v > 0 for v in speeds))
        self.assertTrue(all(v <= 5.0 + 1e-9 for v in speeds))

    def test_acceleration_constraint(self):
        _, speeds = build_speed_profile(self.path)
        n = len(speeds)

        for i in range(n):
            j = (i + 1) % n
            ds = math.dist(self.path[i], self.path[j])
            self.assertLessEqual(
                speeds[j] ** 2,
                speeds[i] ** 2 + 2 * 1.2 * ds + 1e-8,
            )

    def test_braking_constraint(self):
        _, speeds = build_speed_profile(self.path)
        n = len(speeds)

        for i in range(n):
            j = (i + 1) % n
            ds = math.dist(self.path[i], self.path[j])
            self.assertLessEqual(
                speeds[i] ** 2,
                speeds[j] ** 2 + 2 * 1.0 * ds + 1e-8,
            )

    def test_invalid_short_path(self):
        with self.assertRaises(ValueError):
            build_speed_profile([(0, 0), (1, 0), (2, 0), (3, 0)])


if __name__ == "__main__":
    unittest.main()
