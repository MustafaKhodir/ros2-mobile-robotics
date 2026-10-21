// Environment map builder.
//
// Writes a 2D map of a 2.5 m by 2.5 m room to EnvironmentMap.txt as a list of
// x y points, one per line: four walls, four round pillars, a round charging
// station and a square robot sitting in the corner at (0.3, 0.3). Open the file
// in a spreadsheet and plot the two columns as a scatter chart to see it.
//
// Build and run:
//   g++ environment_map.cpp -o environment_map
//   ./environment_map

#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

#define PI 3.14159265

int build_environment_map()
{
    // centres of the four pillars and the charger, in metres
    double pillar_x[4] = {0.8, 0.8, 1.7, 2.2};
    double pillar_y[4] = {2.25, 1.5, 1.0, 1.3};
    double charger_x = 2.25, charger_y = 0.25;
    double r = 0.125;              // radius of the pillars and the charger
    double wallLength = 2.5;       // the room is square
    double RD = PI / 180;          // degrees to radians

    // the robot is a 0.25 m square with no wheels, drawn around its centre
    double half_edge = 0.25 / 2;
    double center_x = 0.3;
    double center_y = 0.3;

    ofstream map;
    map.open("EnvironmentMap.txt");

    // circles for the pillars and the charger, a point every 20 degrees
    for (int i = 0; i < 360; i = i + 20)
    {
        for (int p = 0; p < 4; p++)
        {
            map << pillar_x[p] + r * cos(i * RD) << ' ' << pillar_y[p] + r * sin(i * RD) << endl;
        }
        map << charger_x + r * cos(i * RD) << ' ' << charger_y + r * sin(i * RD) << endl;
    }

    // the four walls, a point every centimetre
    for (double j = 0; j < wallLength; j = j + 0.01)
    {
        map << j << ' ' << 0 << endl;            // bottom wall
        map << 0 << ' ' << j << endl;            // left wall
        map << j << ' ' << wallLength << endl;   // top wall
        map << wallLength << ' ' << j << endl;   // right wall
    }

    // the robot, top and bottom edges then left and right edges
    for (double j = center_x - half_edge; j <= center_x + half_edge; j += 0.01)
    {
        map << j << ' ' << center_y + half_edge << endl;
        map << j << ' ' << center_y - half_edge << endl;
    }
    for (double j = center_y - half_edge; j <= center_y + half_edge; j += 0.01)
    {
        map << center_x + half_edge << ' ' << j << endl;
        map << center_x - half_edge << ' ' << j << endl;
    }

    map.close();
    return 0;
}

int main(int argc, char **argv)
{
    build_environment_map();
    cout << "Map written to EnvironmentMap.txt" << endl;
    return 0;
}
