#!/usr/bin/env python3
# Laser scanner obstacle avoidance (TurtleBot3 Burger, ROS2).
#
# The robot drives forward and watches the 60 degree cone in front of it on
# the laser distance sensor. If anything in that cone is closer than 0.5 m it
# stops, turns away from whichever side the obstacle is nearer to, and carries
# on once the cone is clear. I ran this with Cartographer and RViz open so the
# robot built a SLAM map of the arena while it wandered.
#
# Run it after the TurtleBot bringup (or the Gazebo world) is up:
#   python3 laser_navigation.py
# Ctrl+C stops the robot.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class MyRobot(Node):
    def __init__(self):
        super().__init__('laser_navigation_node')

        # The LDS publishes with best effort reliability, so the subscription
        # has to ask for the same or it never receives anything
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_policy)
        self.sub2 = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        timer_period = 0.3  # seconds between control decisions
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.my_x = 0
        self.my_y = 0
        self.my_th = 0
        self.ranges = []              # latest laser readings, one per degree
        self.obstacle_range = 0.5     # metres, anything closer is an obstacle
        self.forward_speed = 0.2      # m/s
        self.turn_speed = 0.5         # rad/s

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)

    def odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi

    # Smallest real reading in a slice of the scan. The real LDS reports 0.0
    # for beams that get no return, so those get skipped or the robot
    # would think it is always touching something.
    def nearest(self, readings):
        good = [r for r in readings if r > 0.01]
        return min(good) if good else float('inf')

    # Runs every 0.3 s and decides what the robot does next
    def timer_callback(self):
        move = Twist()

        # nothing to decide until the first scan has arrived
        if len(self.ranges) < 360:
            return

        # the cone in front: 0 to 30 degrees on one side of centre, 330 to 360 on the other
        side_a = self.nearest(self.ranges[0:30])
        side_b = self.nearest(self.ranges[330:360])

        if min(side_a, side_b) < self.obstacle_range:
            # obstacle in front, stop and turn away from the closer side
            if side_a > side_b:
                move.angular.z = -self.turn_speed
            else:
                move.angular.z = self.turn_speed
            move.linear.x = 0.0
        else:
            # clear ahead, keep going
            move.linear.x = self.forward_speed
            move.angular.z = 0.0

        self.pub.publish(move)

        print('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th))
        print('nearest ahead: {0:5.2f} m and {1:5.2f} m'.format(side_a, side_b))

    def stop_robot(self):
        self.pub.publish(Twist())   # all zeros
        print('Shutdown')


def main(args=None):
    rclpy.init(args=args)
    myrobot = MyRobot()
    try:
        rclpy.spin(myrobot)
    except KeyboardInterrupt:
        pass
    finally:
        myrobot.stop_robot()
        myrobot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
