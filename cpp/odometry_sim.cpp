// Differential drive odometry simulation for two robots.
//
// Both robots start at (30, 30) cm facing 45 degrees and run for 100 steps of
// 1 second. Each step the position is updated from the two wheel speeds using
// the differential drive kinematics:
//   x     += (vl + vr) / 2 * cos(theta) * dt
//   y     += (vl + vr) / 2 * sin(theta) * dt
//   theta += (vr - vl) / wheelbase * dt
// Robot 1 has the left wheel faster (9 and 7 cm/s) so it curves right. Robot 2
// has the right wheel faster (6 and 8 cm/s) so it curves left. The two paths
// are written to trajectory1.csv for plotting.
//
// Build and run:
//   g++ odometry_sim.cpp -o odometry_sim
//   ./odometry_sim

#include <cmath>
#include <iostream>
#include <fstream>
using namespace std;

#define PI 3.14159265

const int SIZE = 100;          // number of time steps
const double wheelbase = 30;   // distance between the wheels in cm
const double dt = 1;           // seconds per step

// Fills the x, y and theta arrays for one robot from its two wheel speeds
void robot_kinematics(double left_vel, double right_vel, double x[], double y[], double theta[])
{
    for (int i = 0; i < SIZE - 1; i++)
    {
        x[i + 1] = x[i] + ((left_vel + right_vel) / 2) * cos(theta[i]) * dt;
        y[i + 1] = y[i] + ((left_vel + right_vel) / 2) * sin(theta[i]) * dt;
        theta[i + 1] = theta[i] + ((right_vel - left_vel) / wheelbase) * dt;
    }
}

int main(int argc, char **argv)
{
    // wheel speeds in cm/s, left then right
    double vl1 = 9, vr1 = 7;   // first robot
    double vl2 = 6, vr2 = 8;   // second robot

    // both start at (30, 30) facing 45 degrees
    double rob_x1[SIZE] = {30}, rob_y1[SIZE] = {30}, rob_theta1[SIZE] = {PI / 4};
    double rob_x2[SIZE] = {30}, rob_y2[SIZE] = {30}, rob_theta2[SIZE] = {PI / 4};

    robot_kinematics(vl1, vr1, rob_x1, rob_y1, rob_theta1);
    robot_kinematics(vl2, vr2, rob_x2, rob_y2, rob_theta2);

    ofstream fp("trajectory1.csv");
    fp << "x1, y1, x2, y2" << endl;
    for (int i = 0; i < SIZE; i++)
    {
        fp << rob_x1[i] << ", " << rob_y1[i] << ", " << rob_x2[i] << ", " << rob_y2[i] << endl;
    }
    fp.close();

    cout << "Trajectories written to trajectory1.csv" << endl;
    return 0;
}
