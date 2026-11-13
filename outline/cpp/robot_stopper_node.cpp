// Outline only. The full code is private.
// Function names, structure and comments are shown. The working logic is removed.
// Copyright 2026 Mustafa Khodir. All rights reserved.

// Stopper: landmark based navigation from the home corner to the charger (ROS2, C++).
//
// The TurtleBot starts in the corner of the simulated room and has to reach
// the charging station on the other side without hitting the pillars. I did
// it as a six stage state machine driven by laser readings at fixed angles:
//   stage 1  drive fast until the front beam sees the first pillar at 1.3 m
//   stage 2  curve right until the front left beam opens up past 1.43 m
//   stage 3  drive until the front beam reads 0.60 m
//   stage 4  curve right until the front right beam drops under 0.32 m
//   stage 5  creep forward until the front beam reads 0.43 m
//   stage 6  stop, we are at the charger
// The landmark distances came from trial runs and reading laserData.csv.
//
// While it runs it logs four CSV files in the working directory so I could
// plot them afterwards: the raw laser beams, the laser points converted into
// room coordinates (which draws the map), the robot's trajectory, and its
// speed over time.
//
// The package.xml and CMakeLists.txt at the top of the repo build this node
// with colcon. See the README for the steps.

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using namespace std;

class Stopper : public rclcpp::Node
{
public:
    // Speeds I switch between, m/s forward and rad/s for turning
    constexpr const static double FORWARD_SPEED_LOW = 0.1;
    constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
    constexpr const static double FORWARD_SPEED_HIGH = 0.5;
    constexpr const static double FORWARD_SPEED_STOP = 0;
    constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
    constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.6;
    constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
    constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
    constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
    constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;

    Stopper() : Node("stopper")
    { /* removed */ }

    void startMoving();

private:
    // Where the robot starts in the room, metres
    double positionX = 0.3, positionY = 0.3;
    double robotHeading = 0.0;     // heading in radians, updated from odometry
    double robotVelocity = 0.0;    // current forward speed, logged to VelData.csv

    // Laser distances that mark the change from one stage to the next, metres
    static constexpr double LANDMARK_1 = 1.3;
    static constexpr double LANDMARK_2 = 1.43;
    static constexpr double LANDMARK_3 = 0.60;
    static constexpr double LANDMARK_4 = 0.32;
    static constexpr double LANDMARK_5 = 0.43;

    int stage = 1;          // where we are in the state machine

    // Safe stop: if no scan arrives for this long the robot stops
    static constexpr double SCAN_TIMEOUT_S = 1.0;
    bool haveScan = false;
    std::chrono::steady_clock::time_point lastScanTime;
    int scanCount = 0;      // counts scans so the log lines can be matched up

    static constexpr int LOOP_RATE_HZ = 20;
    static constexpr int WARN_THROTTLE_MS = 2000;

    // One beam at each of eight directions round the robot
    double frontRange, frontLeftRange, leftRange, backLeftRange;
    double backRange, backRightRange, rightRange, frontRightRange;

    // Beam directions used when placing the readings on the map, radians
    static constexpr double FRONT_ANGLE = 0;
    static constexpr double FRONT_LEFT_ANGLE = M_PI / 4;
    static constexpr double LEFT_ANGLE = M_PI / 2;
    static constexpr double BACK_LEFT_ANGLE = M_PI - M_PI / 4;
    static constexpr double BACK_ANGLE = M_PI;
    static constexpr double BACK_RIGHT_ANGLE = M_PI + M_PI / 4;
    static constexpr double RIGHT_ANGLE = -M_PI / 2;
    static constexpr double FRONT_RIGHT_ANGLE = -M_PI / 4;

    std::ofstream laserFile, robotTrajFile, robotVelFile, laserMapFile;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    void moveStop();
    void moveForward(double forwardSpeed);
    void moveForwardRight(double forwardSpeed, double turnRightSpeed);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    double beamAt(const sensor_msgs::msg::LaserScan &scan, double angleRad);
    bool stageBeamUsable(double range);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void transformMapPoint(std::ofstream &file, double laserRange, double laserTh, double robotTh,
                           double robotX, double robotY);
};

// Publishes all zeros so the robot stops
void Stopper::moveStop()
{ /* removed */ }

// Straight ahead at the given speed
void Stopper::moveForward(double forwardSpeed)
{ /* removed */ }

// Drive and turn at the same time, which gives a curve
void Stopper::moveForwardRight(double forwardSpeed, double turnRightSpeed)
{ /* removed */ }

// Keeps the robot's position and heading up to date and logs them
void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{ /* removed */ }

// Reading at a given angle, 0 is straight ahead and positive is anticlockwise.
// The index comes from the scan's own angle_min and angle_increment, so it
// works whatever resolution the scanner runs at. NaN, infinite and out of
// range readings come back as NaN so nothing downstream acts on them.
double Stopper::beamAt(const sensor_msgs::msg::LaserScan &scan, double angleRad)
{ /* removed */ }

// The beam a stage depends on has to be usable before the robot acts on it.
// If it is not, the robot stops and waits in the same stage.
bool Stopper::stageBeamUsable(double range)
{ /* removed */ }

// Reads the eight beams, logs them, builds the map, and runs the state machine
void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{ /* removed */ }

// Converts one laser beam into an x y point in the room and writes it out
void Stopper::transformMapPoint(ofstream &file, double laserRange, double laserTh, double robotTh,
                                double robotX, double robotY)
{ /* removed */ }

// Opens the log files and spins the node at 20 Hz until Ctrl+C
void Stopper::startMoving()
{ /* removed */ }

int main(int argc, char *argv[])
{ /* removed */ }
