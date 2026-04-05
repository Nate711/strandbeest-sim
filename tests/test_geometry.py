from __future__ import annotations

import unittest

from strandbeest_gui.geometry import LinkageParams, distance, foot_path, solve_pose


class GeometryTests(unittest.TestCase):
    def test_default_geometry_preserves_lengths(self) -> None:
        params = LinkageParams()
        previous = None
        for angle in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 315.0):
            pose = solve_pose(params, angle, previous)
            previous = pose

            self.assertAlmostEqual(pose.crank_center[0] - pose.ground[0], params.a, places=6)
            self.assertAlmostEqual(pose.crank_center[1] - pose.ground[1], params.l, places=6)

            self.assertAlmostEqual(distance(pose.ground, pose.upper_joint), params.b, places=6)
            self.assertAlmostEqual(distance(pose.ground, pose.lower_joint), params.c, places=6)
            self.assertAlmostEqual(distance(pose.ground, pose.left_joint), params.d, places=6)
            self.assertAlmostEqual(distance(pose.left_joint, pose.upper_joint), params.e, places=6)
            self.assertAlmostEqual(distance(pose.left_joint, pose.knee_joint), params.f, places=6)
            self.assertAlmostEqual(distance(pose.knee_joint, pose.lower_joint), params.g, places=6)
            self.assertAlmostEqual(distance(pose.knee_joint, pose.foot), params.h, places=6)
            self.assertAlmostEqual(distance(pose.lower_joint, pose.foot), params.i, places=6)
            self.assertAlmostEqual(distance(pose.crank_end, pose.upper_joint), params.j, places=6)
            self.assertAlmostEqual(distance(pose.crank_end, pose.lower_joint), params.k, places=6)
            self.assertAlmostEqual(distance(pose.crank_center, pose.crank_end), params.m, places=6)

    def test_foot_path_returns_expected_sample_count(self) -> None:
        params = LinkageParams()
        path = foot_path(params, samples=120)
        self.assertEqual(len(path), 120)
        for x, y in path:
            self.assertIsInstance(x, float)
            self.assertIsInstance(y, float)


if __name__ == "__main__":
    unittest.main()
