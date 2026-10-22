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
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using namespace std;

class Stopper : public rclcpp::Node
{
public:
    // speeds I switch between, m/s forward and rad/s for turning
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
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // The scan topic is published best effort, so subscribe with the
        // sensor data QoS profile or no scans arrive
        laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
        odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
    }

    void startMoving();

private:
    // where the robot starts in the room, metres
    double PositionX = 0.3, PositionY = 0.3;
    double robotHeadAngle = 0.0;   // heading in radians, updated from odometry
    double robVelocity = 0.0;      // current forward speed, logged to VelData.csv

    // laser distances that mark the change from one stage to the next, metres
    double laser_landmark1 = 1.3;
    double laser_landmark2 = 1.43;
    double laser_landmark3 = 0.60;
    double laser_landmark4 = 0.32;
    double laser_landmark5 = 0.43;

    int stage = 1;          // where we are in the state machine

    // The highest beam index read is 315, so a scan needs at least 316 readings
    static constexpr size_t MIN_SCAN_READINGS = 316;

    // Safe stop: if no scan arrives for this long the robot stops
    static constexpr double SCAN_TIMEOUT_S = 1.0;
    bool haveScan = false;
    std::chrono::steady_clock::time_point lastScanTime;
    int laser_index = 0;    // counts scans so the log lines can be matched up

    // one beam at each of eight directions round the robot
    double frontRange, backRange, mleftRange, bleftRange, leftRange, rightRange, brightRange, mrightRange;
    double leftAngle = M_PI / 2, mleftAngle = M_PI / 4, frontAngle = 0;
    double mrightAngle = -M_PI / 4, rightAngle = -M_PI / 2;

    std::ofstream laserFile, robotTrajFile, robotVelFile, laserMapFile;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;

    void moveStop();
    void moveForward(double forwardSpeed);
    void moveRight(double turn_right_speed);
    void moveForwardRight(double forwardSpeed, double turn_right_speed);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void transformMapPoint(std::ofstream &fp, double laserRange, double laserTh, double robotTh,
                           double robotX, double robotY);
};

// Publishes all zeros so the robot stops
void Stopper::moveStop()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = FORWARD_SPEED_STOP;
    msg.angular.z = 0.0;
    robVelocity = 0.0;
    publisher_->publish(msg);
}

// Straight ahead at the given speed
void Stopper::moveForward(double forwardSpeed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    robVelocity = forwardSpeed;
    publisher_->publish(msg);
}

// Turn on the spot, negative is to the right
void Stopper::moveRight(double turn_right_speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = turn_right_speed;
    robVelocity = 0.0;
    publisher_->publish(msg);
}

// Drive and turn at the same time, which gives a curve
void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    msg.angular.z = turn_right_speed;
    robVelocity = forwardSpeed;
    publisher_->publish(msg);
}

// Keeps the robot's position and heading up to date and logs them
void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    PositionX = odom->pose.pose.position.x;
    PositionY = odom->pose.pose.position.y;
    double q0 = odom->pose.pose.orientation.w;
    double q1 = odom->pose.pose.orientation.x;
    double q2 = odom->pose.pose.orientation.y;
    double q3 = odom->pose.pose.orientation.z;
    robotHeadAngle = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    robotTrajFile << PositionX << ", " << PositionY << std::endl;
    robotVelFile << robVelocity << std::endl;
}

// Reads the eight beams, logs them, builds the map, and runs the state machine
void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // A short scan would make the fixed beam indexes below read past the end
    if (scan->ranges.size() < MIN_SCAN_READINGS)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Scan has %zu readings, need %zu. Stopping.", scan->ranges.size(), MIN_SCAN_READINGS);
        moveStop();
        return;
    }

    haveScan = true;
    lastScanTime = std::chrono::steady_clock::now();

    frontRange = scan->ranges[0];
    mleftRange = scan->ranges[45];
    leftRange = scan->ranges[90];
    bleftRange = scan->ranges[135];
    backRange = scan->ranges[180];
    brightRange = scan->ranges[225];
    rightRange = scan->ranges[270];
    mrightRange = scan->ranges[315];

    laserFile << leftRange << "," << bleftRange << "," << mleftRange << "," << backRange << ","
              << frontRange << "," << mrightRange << "," << brightRange << "," << rightRange
              << "," << laser_index++ << std::endl;

    // turn each beam into a point in room coordinates, that is what draws the map
    transformMapPoint(laserMapFile, frontRange, frontAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, backRange, M_PI, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, bleftRange, M_PI - M_PI / 4, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, brightRange, M_PI + M_PI / 4, robotHeadAngle, PositionX, PositionY);

    switch (stage)
    {
    case 1:
        if (frontRange > laser_landmark1)
            moveForward(FORWARD_SPEED_HIGH);
        else
        {
            stage = 2;
            RCLCPP_INFO(this->get_logger(), "Robot stage: %d", stage);
        }
        break;
    case 2:
        if (mleftRange < laser_landmark2)
            moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        else
        {
            stage = 3;
            RCLCPP_INFO(this->get_logger(), "Robot stage: %d", stage);
        }
        break;
    case 3:
        if (frontRange > laser_landmark3)
            moveForward(FORWARD_SPEED_MIDDLE);
        else
        {
            stage = 4;
            RCLCPP_INFO(this->get_logger(), "Robot stage: %d", stage);
        }
        break;
    case 4:
        if (mrightRange > laser_landmark4)
            moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        else
        {
            stage = 5;
            RCLCPP_INFO(this->get_logger(), "Robot stage: %d", stage);
        }
        break;
    case 5:
        if (frontRange > laser_landmark5)
            moveForward(FORWARD_SPEED_LOW);
        else
        {
            stage = 6;
            RCLCPP_INFO(this->get_logger(), "Robot stage: %d, at the charger", stage);
        }
        break;
    case 6:
        moveStop();
        break;
    }
}

// Converts one laser beam into an x y point in the room and writes it out
void Stopper::transformMapPoint(ofstream &fp, double laserRange, double laserTh, double robotTh,
                                double robotX, double robotY)
{
    // beams that hit nothing come back as infinity, no point plotting those
    if (!std::isfinite(laserRange))
        return;

    double transX = laserRange * cos(robotTh + laserTh) + robotX;
    double transY = laserRange * sin(robotTh + laserTh) + robotY;

    // the room starts at 0, 0 so clip anything that lands outside
    if (transX < 0)
        transX = 0;
    if (transY < 0)
        transY = 0;

    fp << transX << ", " << transY << std::endl;
}

// Opens the log files and spins the node at 20 Hz until Ctrl+C
void Stopper::startMoving()
{
    RCLCPP_INFO(this->get_logger(), "Start Moving");
    robotTrajFile.open("TrajData.csv", std::ios::trunc);
    robotVelFile.open("VelData.csv", std::ios::trunc);
    laserFile.open("laserData.csv", std::ios::trunc);
    laserMapFile.open("laserMapData.csv", std::ios::trunc);

    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());

        // Safe stop: no fresh scan for a second means the robot is driving
        // blind, so stop until scans come back
        double sinceScan = std::chrono::duration<double>(std::chrono::steady_clock::now() - lastScanTime).count();
        if (!haveScan || sinceScan > SCAN_TIMEOUT_S)
            moveStop();

        loop_rate.sleep();
    }

    moveStop();
    robotTrajFile.close();
    robotVelFile.close();
    laserFile.close();
    laserMapFile.close();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto stopper = std::make_shared<Stopper>();
    stopper->startMoving();
    rclcpp::shutdown();
    return 0;
}
