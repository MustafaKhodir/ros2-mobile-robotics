# ROS2 Mobile Robotics: TurtleBot Navigation and Odometry Simulation

Four programs I wrote for the TurtleBot3 Burger, two in Python on ROS2 and two in C++. The Python nodes drive the real robot and its Gazebo twin: one traces a 1 m square from odometry alone, the other wanders an arena avoiding pillars with the laser scanner while Cartographer builds a SLAM map. The C++ side is a differential drive odometry simulation that plots two robots' paths and draws a room map, and a ROS2 node that steers the robot from its home corner to a charging station using laser landmarks.

![The TurtleBot3 in the arena with the cardboard pillars it has to avoid](media/photos/turtlebot-in-the-arena.jpg)

## What it does

- **Square path from odometry.** The robot drives a 1 m square with no external sensors. It reads its own x, y and heading from the wheel encoders, drives each side at 0.2 m/s, turns each corner on the spot at 0.2 rad/s, and logs every reading to a text file so the route can be plotted.
- **Laser scanner navigation.** The robot drives forward and checks the 60 degree cone in front of it. Anything closer than 0.5 m makes it stop and turn away from the nearer side. Running it with Cartographer and RViz open builds a map of the arena as it goes.
- **Odometry simulation in C++.** Two differential drive robots start at the same point and heading with different wheel speeds. The program steps their kinematics 100 times and writes both paths to a CSV, which shows one curving right and the other curving left.
- **Environment map in C++.** Writes a point cloud of a 2.5 m square room with four pillars, a charging station and a square robot, ready to plot.
- **Laser guided drive to the charger.** A ROS2 C++ node that takes the simulated robot from the corner to the charger through the pillars using a six stage state machine, while logging the laser beams, the map points, the trajectory and the speed to CSV files.

## My role

All of this is my own work. I wrote every line of the Python and C++, ran the robot in the arena and in Gazebo, collected the odometry and laser logs, and plotted the results in Excel and LibreOffice.

## How it works

### Square path (python/trajectory_tracking.py)

The node subscribes to the odom topic and publishes velocity commands every 0.3 s. It converts the orientation quaternion to a heading in degrees and runs a seven step sequence: forward until x passes 1 m, turn until the heading passes 90 degrees, forward until y passes 1 m, turn until the heading wraps past 180, back along x, turn until minus 90, back along y, stop. The heading wrap at 180 degrees is handled by checking both sides of it.

Every tick the x, y and heading are appended to myodom.txt. I ran two trials at different speeds, loaded the files into Excel and both plotted as a near perfect square. On the real robot the start heading has to be lined up exactly so the first corner lands where it should, and the run takes longer than in simulation because of the floor. The odometry error grew on every turn, which is the usual encoder drift, and the TurtleBot's Kalman filter fusing the encoders with its IMU is what keeps it as accurate as it is.

![Odometry log from a real run plotted in Excel](media/figures/square-path-odometry-plot.png)

### Laser navigation (python/laser_navigation.py)

The laser scan is 360 readings, one per degree. The node looks at the 30 readings either side of straight ahead and takes the nearest real reading on each side (the real sensor reports zero for a beam that gets no return, so zeros are skipped). If either side is under 0.5 m the robot stops moving forward and turns at 0.5 rad/s away from whichever side is closer, then carries on at 0.2 m/s once the cone is clear. The laser topic has to be subscribed with best effort reliability or nothing arrives.

I used this with Cartographer and RViz to build SLAM maps of the arena. Driving the robot by hand with the keyboard took about 5 minutes in simulation and over 8 minutes on the real robot, because the real arena is much bigger. Letting the node drive took about 3 minutes in simulation and 6 minutes for real, with no collisions in either. The things I would add next are a particle filter for the pose estimate and a random walk so the robot stops repeating the same loop when there are no obstacles.

![SLAM map of the real arena built in Cartographer while the robot drove itself](media/figures/real-arena-slam-map.png)

### Odometry simulation (cpp/odometry_sim.cpp)

Both robots start at (30, 30) cm facing 45 degrees with a 30 cm wheelbase, and run 100 steps of one second. Each step applies the differential drive equations: the centre moves at the average of the two wheel speeds along the current heading, and the heading changes by the wheel speed difference divided by the wheelbase. Robot 1 has wheel speeds of 9 and 7 cm/s, so the faster left wheel pushes it into a right hand circle. Robot 2 has 6 and 8 cm/s and circles the other way. The output goes to trajectory1.csv.

![Both simulated trajectories plotted from the CSV](media/figures/two-robot-trajectories.png)

### Environment map (cpp/environment_map.cpp)

The map is just a text file of x y points. Each pillar and the charger is a circle of 18 points, the walls are a point every centimetre, and the robot is a 25 cm square in the corner at (0.3, 0.3). Plotting the two columns as a scatter chart gives the room layout, which is what the charger node is driving through.

![The environment map plotted as a scatter chart](media/figures/environment-map-plot.png)

### Drive to the charger (cpp/robot_stopper_node.cpp)

The node reads eight laser beams at 45 degree spacing and runs a state machine. Stage 1 drives at 0.5 m/s until the front beam sees the first pillar at 1.3 m. Stage 2 curves right at 0.3 m/s until the front left beam opens past 1.43 m. Stage 3 drives until the front beam reads 0.6 m. Stage 4 curves right again until the front right beam drops under 0.32 m. Stage 5 creeps at 0.1 m/s until the front beam reads 0.43 m, and stage 6 stops at the charger. The landmark distances came from trial runs and from reading the logged laser data. Adding the three rear beams at 135, 180 and 225 degrees made the map log noticeably more complete.

Every scan is written to laserData.csv, and each beam is converted into room coordinates using the robot's pose and written to laserMapData.csv, which plots as a map. The odometry callback logs the trajectory and the current speed so the whole run can be graphed.

![The simulated room in Gazebo with the robot heading for the charger](media/figures/gazebo-charger-arena.png)

![Robot trajectory from home to the charger, plotted from TrajData.csv](media/figures/robot-trajectory-to-charger.png)

## How to run it

### Python nodes

1. Bring up the TurtleBot3 as normal, or launch the Gazebo world, so the scan, odom and cmd_vel topics are live.
2. In a terminal with ROS2 sourced run one of:

```bash
python3 python/trajectory_tracking.py
```

```bash
python3 python/laser_navigation.py
```

3. Ctrl+C stops the robot. The square path node also leaves myodom.txt in the working directory.
4. For a map, launch Cartographer and RViz in two more terminals before starting the laser node.

### C++ simulation programs

These need no ROS, just a C++ compiler.

```bash
g++ cpp/odometry_sim.cpp -o odometry_sim && ./odometry_sim
```

```bash
g++ cpp/environment_map.cpp -o environment_map && ./environment_map
```

Open trajectory1.csv or EnvironmentMap.txt in a spreadsheet and plot the columns as a scatter chart.

### Charger node

Drop robot_stopper_node.cpp into a ROS2 C++ package whose package.xml and CMakeLists.txt depend on rclcpp, geometry_msgs, sensor_msgs and nav_msgs, build it with colcon, launch the Gazebo room, then run the node. The four CSV logs appear in the directory you ran it from.

## What's in this repo

- `python/` the two ROS2 Python nodes.
- `cpp/` the two standalone C++ programs and the ROS2 C++ node.
- `media/photos/` the TurtleBot in the arena.
- `media/figures/` the plots and screenshots of the runs.

## Demo video

Google Drive link: (add link here)

The full project report is available on request.

Copyright 2026 Mustafa Khodir. All rights reserved.
