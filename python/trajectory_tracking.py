#!/usr/bin/env python3
# Square path with odometry only (TurtleBot3 Burger, ROS2).
#
# The robot drives a 1 m square using nothing but its wheel odometry. It goes
# forward until x passes 1 m, turns on the spot until the heading passes 90
# degrees, goes forward until y passes 1 m, and so on round the four sides,
# then stops. q is the step number so I could see on the terminal which leg
# it was on. Every control tick the x, y and heading get written to
# myodom.txt so I could plot the route in Excel afterwards.
#
# Run it after the TurtleBot bringup (or the Gazebo world) is up:
#   python3 trajectory_tracking.py
# Ctrl+C stops the robot and closes the log file.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MyRobot(Node):
    def __init__(self):
        super().__init__('square_path_node')
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        timer_period = 0.3  # seconds between control decisions
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.forward_speed = 0.20  # m/s on the straight sides
        self.turn_speed = 0.2      # rad/s on the corners, positive is anticlockwise
        self.side_length = 1.00    # metres

        self.q = 0       # which leg of the square we are on, 7 means finished
        self.my_x = 0    # x position from odometry
        self.my_y = 0    # y position from odometry
        self.my_th = 0   # heading in degrees, from -180 to 180

        # odometry log, one line per control tick
        self.myfile = open('myodom.txt', 'w')
        self.myfile.write('x, y, heading\n')

    # Keeps the newest position and converts the quaternion to a heading in degrees
    def odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2 + q3 * q3))) * 180 / math.pi

    # Runs every 0.3 s and decides what the robot does on this leg
    def timer_callback(self):
        move = Twist()

        if self.q == 0:                 # side 1, forward along x
            move.linear.x = self.forward_speed
            if self.my_x > self.side_length:
                self.q = 1
        elif self.q == 1:               # corner 1, turn until facing 90 degrees
            move.angular.z = self.turn_speed
            if self.my_th > 90.00:
                self.q = 2
        elif self.q == 2:               # side 2, forward along y
            move.linear.x = self.forward_speed
            if self.my_y > self.side_length:
                self.q = 3
        elif self.q == 3:               # corner 2, turn until facing 180 degrees
            move.angular.z = self.turn_speed
            # the heading wraps from 180 to -180 here so I check both sides of it
            if self.my_th > 177 or self.my_th < -170:
                self.q = 4
        elif self.q == 4:               # side 3, back along x
            move.linear.x = self.forward_speed
            if self.my_x < 0.00:
                self.q = 5
        elif self.q == 5:               # corner 3, turn until facing -90 degrees
            move.angular.z = self.turn_speed
            if self.my_th > -90.00:
                self.q = 6
        elif self.q == 6:               # side 4, back along y to the start
            move.linear.x = self.forward_speed
            if self.my_y < 0.05:
                self.q = 7
                print('Square route has been completed!')
        else:                           # q is 7, stay stopped
            move.linear.x = 0.0
            move.angular.z = 0.0

        self.pub.publish(move)

        # print and log the pose so I can see where it is and plot it later
        print('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}, q = {3:d}'.format(self.my_x, self.my_y, self.my_th, self.q))
        self.myfile.write('{0:5.2f}, {1:5.2f}, {2:5.2f}\n'.format(self.my_x, self.my_y, self.my_th))

    def stop_robot(self):
        self.pub.publish(Twist())   # all zeros
        self.myfile.close()
        print('Shutdown, odometry saved to myodom.txt')


def main(args=None):
    rclpy.init(args=args)
    myrobot_node = MyRobot()
    try:
        rclpy.spin(myrobot_node)
    except KeyboardInterrupt:
        pass
    finally:
        myrobot_node.stop_robot()
        myrobot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
