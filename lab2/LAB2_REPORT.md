## Lab 2 – ROS2 Integration with Gazebo and LiDAR Processing

This report describes the work done in the `lab2` directory, the structure of the ROS2 package, the implemented nodes, and how to build and run the project.

### 1. Package Structure of `lab2`

The `lab2` package is a ROS2 package that integrates the Gazebo simulator with ROS2 and implements basic mobile robot control and LiDAR data processing.

- **Main files and directories**:
  - `setup.py` – Python setup script that declares the ROS2 package `lab2`, its metadata (version, maintainer, license), data files, and console entry points.
  - `worlds/robot.sdf` – Gazebo world that contains the robot model and its environment.
  - `config/robot.rviz` – RViz2 configuration used to visualize the robot and sensor data.
  - `launch/gazebo_ros2.launch.py` – launch file that starts the Gazebo simulation, the ROS–Gazebo bridge, and RViz2.
  - `lab2/robot_controller.py` – ROS2 node that publishes velocity commands to control the robot.
  - `lab2/lidar_subscriber.py` – ROS2 node that subscribes to LiDAR data, processes it, and publishes aggregated information.

- **`setup.py` details**:
  - Registers the package name `lab2` and installs various resources (launch files, worlds, RViz config) under the `share/lab2` directory.
  - Defines console scripts so the nodes can be run via `ros2 run`:
    - `robot_controller = lab2.robot_controller:main`
    - `lidar_subscriber = lab2.lidar_subscriber:main`

### 2. Robot Control Node `robot_controller`

The file `lab2/robot_controller.py` implements a simple motion controller for the differential drive robot.

- A ROS2 node named `robot_controller` is created by subclassing `Node` (`class RobotController(Node)`).
- A publisher is created for the topic `/cmd_vel` with message type `geometry_msgs/msg/Twist`. This topic is bridged to Gazebo and drives the robot in the simulation.
- A timer is created with a period of 0.1 seconds (10 Hz). On each timer tick, the callback `timer_callback` is executed.
- In `timer_callback`:
  - A `Twist` message is created.
  - The linear velocity is set to move the robot forward: `linear.x = 0.5` m/s.
  - The angular velocity is set as a sinusoidal function of an internal counter: `angular.z = 0.3 * sin(counter * 0.1)`, which causes the robot to follow a wavy path.
  - The `Twist` message is published to `/cmd_vel`.
  - Every 50 timer ticks (about every 5 seconds), the node logs the current linear and angular velocities for monitoring.

### 3. LiDAR Processing Node `lidar_subscriber`

The file `lab2/lidar_subscriber.py` implements a node that subscribes to raw LiDAR data, computes statistics, detects obstacles, and publishes processed data.

- A ROS2 node named `lidar_subscriber` is created (`class LidarSubscriber(Node)`).
- Two interfaces are initialized:
  - A subscriber to `/lidar` with message type `sensor_msgs/msg/LaserScan`.
  - A publisher to `/lidar_processed` with message type `std_msgs/msg/Float32MultiArray`.

- In the callback `lidar_callback`:
  1. **Filtering valid ranges**  
     - All range readings from `msg.ranges` are filtered using the sensor limits: only values where `range_min < r < range_max` are kept.
     - If there are no valid readings, the node logs a message and returns.

  2. **Global statistics**  
     - For all valid ranges, the node computes:
       - Minimum distance,
       - Maximum distance,
       - Average distance.
     - These values, along with the count of valid points and the total number of rays, are logged to the console. This gives an overall summary of how close obstacles are around the robot.

  3. **Splitting LiDAR data into sectors (left, front, right)**  
     - The full `ranges` array is split into three equal sectors:
       - Left sector: first third of the array.
       - Front sector: middle third of the array.
       - Right sector: last third of the array.
     - For each sector, invalid readings are removed (same validity check as above).
     - For each sector, the node computes:
       - Minimum distance in that sector.
       - Average distance in that sector.
     - If a sector has no valid data, a fallback value is used (typically the maximum range), meaning “no obstacle detected within the sensor range in that direction”.

  4. **Front obstacle detection**  
     - The node specifically checks the **front sector** for obstacles.
     - If there are valid readings in the front sector and the minimum front distance is less than 1 meter, the node logs a warning that an obstacle is detected in front of the robot.
     - Additionally, the node logs the minimum distances for left, front, and right sectors as an estimation of free space in each direction.

  5. **Publishing processed data**  
     - The node creates a `Float32MultiArray` message and fills it with six values:
       - `[min_left, min_front, min_right, avg_left, avg_front, avg_right]`
     - This message is published to the topic `/lidar_processed`.
     - Other nodes can subscribe to `/lidar_processed` to make navigation or decision‑making easier, without having to work with raw `LaserScan` data directly.

### 4. Launch File `gazebo_ros2.launch.py` and ROS–Gazebo Integration

The file `launch/gazebo_ros2.launch.py` orchestrates the simulator, the ROS–Gazebo bridge, and visualization.

- The function `generate_launch_description()`:
  - Uses `FindPackageShare` to locate the `lab2` and `ros_gz_sim` packages.
  - Constructs paths to:
    - `worlds/robot.sdf` – the simulation world containing the robot.
    - `config/robot.rviz` – the RViz2 configuration.
    - `gz_sim.launch.py` – the standard Gazebo Sim launch file.

- The returned `LaunchDescription` includes three main actions:
  1. **Launch Gazebo with the robot world**  
     - Includes the `gz_sim.launch.py` launch file from `ros_gz_sim`.
     - Passes the `robot.sdf` world file as an argument so Gazebo starts with the correct environment and robot model.
  2. **Start the ROS–Gazebo bridge**  
     - Launches the `ros_gz_bridge` executable `parameter_bridge`.
     - Bridges the following topics:
       - `/lidar` as `sensor_msgs/msg/LaserScan` ↔ `gz.msgs.LaserScan`.
       - `/cmd_vel` as `geometry_msgs/msg/Twist` ↔ `gz.msgs.Twist`.
     - This allows ROS2 nodes to send velocity commands to the simulated robot and receive LiDAR data from the simulator.
  3. **Start RViz2 for visualization**  
     - Launches RViz2 with the configuration file `robot.rviz`.
     - This setup visualizes the robot, LiDAR scan data, and other elements of the scene.

### 5. Build and Run Instructions

This section describes how to build and run the project inside the provided Docker workspace, following the same workflow as in Lab 1.

#### 5.1. Start and enter the Docker workspace

From the repository root

1. Start the lab container
./scripts/cmd run

2. In a new terminal, attach to the running container
./scripts/cmd bash

Inside the container, the working directory is typically `/opt/ws`.#### 5.2. Build the workspace with the `lab2` packageInside the container, in `/opt/ws`:

3. Build the colcon workspace (including the lab2 package)
colcon build

4. Source the workspace so ROS2 can find the lab2 package
source install/setup.bash

#### 5.3. Launch the simulation with ROS2 integration# 5. Launch Gazebo, the ROS–Gazebo bridge, and RViz2ros2 launch lab2 gazebo_ros2.launch.py

This command:
Starts Gazebo with the robot.sdf world,

Starts the bridge for /cmd_vel and /lidar,

Opens RViz2 with the predefined configuration.

5.4. Run the control and LiDAR processing nodes

In additional terminals inside the same container (with source install/setup.bash already executed):

# 6. Run the robot controller noderos2 run lab2 robot_controller# 7. Run the LiDAR processing noderos2 run lab2 lidar_subscriber

The robot_controller node continuously publishes velocity commands to /cmd_vel, causing the simulated robot to move forward with a sinusoidal steering pattern.

The lidar_subscriber node subscribes to /lidar, computes overall and sector-based statistics, detects obstacles in front of the robot, and publishes processed data to /lidar_processed.