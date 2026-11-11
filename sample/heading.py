"""Heading maths from my TurtleBot3 square path node (trajectory_tracking.py).

The robot's odometry gives its orientation as a quaternion. These functions turn
that into a heading in degrees and measure how far the robot has turned, even when
the heading crosses from 180 to -180 degrees. They are the same maths the node
uses, taken out on their own so they can be tested without ROS2.
"""
import math


def quaternion_to_heading(w, x, y, z):
    """Heading (yaw) in degrees, between -180 and 180, from an orientation quaternion."""
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap(angle):
    """Wraps an angle in degrees into the range -180 to 180."""
    return (angle + 180.0) % 360.0 - 180.0


class TurnCounter:
    """Adds up how far the robot has turned since a corner started.

    Each new heading is compared with the last one and the step is wrapped,
    so a turn that crosses 180 to -180 counts as a small step, not a jump of 360.
    """

    def __init__(self, start_heading):
        self.last = start_heading
        self.turned = 0.0

    def update(self, heading):
        self.turned += wrap(heading - self.last)
        self.last = heading
        return self.turned

    def done(self, target=90.0):
        return self.turned >= target
