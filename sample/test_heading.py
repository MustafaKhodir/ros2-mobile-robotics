import math
import unittest

from heading import TurnCounter, quaternion_to_heading, wrap


def quaternion_for(heading_deg):
    """Quaternion for a robot on flat ground facing heading_deg (a turn about z only)."""
    half = math.radians(heading_deg) / 2
    return math.cos(half), 0.0, 0.0, math.sin(half)


class QuaternionToHeading(unittest.TestCase):
    def test_known_headings(self):
        for deg in (0, 30, 90, 135, -45, -90, -170, 179):
            self.assertAlmostEqual(quaternion_to_heading(*quaternion_for(deg)), deg, places=6)

    def test_facing_backwards_is_180(self):
        self.assertAlmostEqual(abs(quaternion_to_heading(*quaternion_for(180))), 180, places=6)

    def test_negative_quaternion_is_the_same_heading(self):
        w, x, y, z = quaternion_for(60)
        self.assertAlmostEqual(quaternion_to_heading(-w, -x, -y, -z), 60, places=6)


class Wrap(unittest.TestCase):
    def test_wrap(self):
        self.assertEqual(wrap(190), -170)
        self.assertEqual(wrap(-190), 170)
        self.assertEqual(wrap(360), 0)
        self.assertEqual(wrap(45), 45)


class TurnCounting(unittest.TestCase):
    def test_quarter_turn_from_zero(self):
        t = TurnCounter(0)
        for h in range(0, 91, 10):
            t.update(h)
        self.assertTrue(t.done())

    def test_turn_across_180(self):
        # Left turn from 150 degrees: the heading runs 150, 170, -170, -150, -120
        t = TurnCounter(150)
        for h in (170, -170, -150, -120):
            t.update(h)
        self.assertAlmostEqual(t.turned, 90)
        self.assertTrue(t.done())

    def test_not_done_before_90(self):
        t = TurnCounter(10)
        t.update(60)
        self.assertFalse(t.done())


if __name__ == "__main__":
    unittest.main()
