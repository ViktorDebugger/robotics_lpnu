# Lab 2: Introduction to ROS2 and Simulation Environment

## Learning Goals

1. Understand fundamental ROS2 concepts (nodes, topics, services, packages)
2. Create a basic ROS2 Python package
3. Launch Gazebo from ROS2 using launch files
4. Use `ros_gz_bridge` to connect Gazebo and ROS2 topics
5. Write a ROS2 publisher node to control robot motion
6. Write a ROS2 subscriber node to process LiDAR data
7. Visualize sensor data in RViz2

---

## Part 1: Understanding ROS2 Fundamentals

Before writing code, it's important to understand how ROS2 works. Based on the [Articulated Robotics ROS Overview](https://articulatedrobotics.xyz/tutorials/ready-for-ros/ros-overview).

### What is ROS2?

ROS (Robot Operating System) is not actually an operating system, but a set of common libraries and tools for building robotic systems. **ROS2** is the modern version with significant improvements over ROS1.

### Nodes

A ROS system is made up of many small programs called **nodes** that run simultaneously and communicate with each other. Each node is designed to perform one specific task:

- Reading sensor data
- Controlling motors
- Processing images
- Planning trajectories
- etc.

**Key commands:**
```bash
ros2 run <package> <node>    # Run a node
ros2 node list                # See all running nodes
ros2 node info <node_name>    # Get info about a node
```

### Topics and Messages

Nodes communicate using **topics** and **messages**:
- A **topic** is a named channel (like `/cmd_vel` or `/lidar`)
- A **message** is the data sent over that topic (like velocity commands or sensor readings)
- A **publisher** sends messages to a topic
- A **subscriber** receives messages from a topic

This system is powerful because:
- Publishers don't need to know who's listening
- Subscribers don't need to know where data comes from
- Nodes can run on different computers (ROS handles networking)

**Key commands:**
```bash
ros2 topic list                    # See all topics
ros2 topic echo <topic>            # View messages
ros2 topic info <topic>            # See publishers/subscribers
ros2 topic pub <topic> <msg> ...   # Manually publish
```

### Services

Unlike topics (many messages, anyone can listen), **services** provide request/reply communication:
- One node sends a request
- Another node processes it and sends back a reply
- Used for operations like "start motor" or "calculate path"

**Key commands:**
```bash
ros2 service list                  # See all services
ros2 service call <service> <msg>  # Call a service
```

### Packages

ROS code is organized into **packages**. A package contains:
- One or more nodes
- Launch files
- Configuration files
- Dependencies list

Example: A `lidar_driver` package might contain a node for reading the sensor and a launch file to start it.

### Workspaces

When developing, you create a **workspace** to organize your packages:

```
workspace/
├── src/           # Your source code (packages)
├── build/         # Compiled code
└── install/       # Installed files
```

**Important commands:**
```bash
colcon build              # Build all packages
colcon build --symlink-install  # Build with symlinks (faster for Python)
source install/setup.bash # Make packages visible to ROS
```

**Every time you open a new terminal:**
1. Source ROS: `source /opt/ros/jazzy/setup.bash`
2. Source your workspace: `source /opt/ws/install/setup.bash`

---

## Part 2: Trying ROS2 Launches and Demos

Before building your own packages, let's explore what ROS2 can do with pre-installed examples.

### What is `ros2 launch`?

A **launch file** starts multiple nodes and configures them all at once. Instead of running each node separately with `ros2 run`, you use `ros2 launch` to start an entire system.

### Demo 1: Turtlesim with Multiple Nodes

```bash
# Launch turtlesim demo (creates window with turtle)
ros2 run turtlesim turtlesim_node
```

**What's running?** Open a new terminal and check:
```bash
./scripts/cmd bash
ros2 node list         # See all nodes
ros2 topic list        # See all topics
ros2 topic echo /turtle1/pose  # Watch turtle position
```

**Try controlling it:**
```bash
ros2 run turtlesim turtle_teleop_key
```

**Learn more:** [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

### Demo 3: Check ROS2 Package Structure

```bash
# See where ROS2 packages are installed
ros2 pkg prefix turtlesim

# Look at the package structure
ls /opt/ros/jazzy/share/turtlesim/
```

Notice: `launch/`, `package.xml`, etc. - your package will have similar structure!

### Key Takeaway

ROS2 nodes communicate via topics:
- `talker` publishes messages
- `listener` subscribes to messages
- `ros2 topic` commands help you inspect the system


## Part 3: ROS2-Gazebo Integration Overview

Based on the [Gazebo ROS2 tutorials](https://gazebosim.org/docs/harmonic/ros2_overview/), there are three main integration points:

1. **Launch Gazebo from ROS2** - Use ROS2 launch files to start Gazebo
2. **Bridge topics** - Use `ros_gz_bridge` to translate between Gazebo Transport and ROS2
3. **Create ROS2 nodes** - Write publishers and subscribers to interact with the simulation

---

## Part 4: Package Structure Overview

Your final package will look like this:

```
lab2/
├── package.xml              # ROS2 package manifest (dependencies)
├── setup.py                 # Python package setup (build config)
├── setup.cfg                # Setuptools config (script paths)
├── resource/
│   └── lab2                 # Empty marker file (required by ament)
├── lab2/                    # Python package directory
│   ├── __init__.py          # Makes this a Python package
│   ├── robot_controller.py  # Publisher: controls robot
│   └── lidar_subscriber.py  # Subscriber: processes LiDAR
├── launch/
│   └── gazebo_ros2.launch.py  # Launches Gazebo + bridge + RViz2
├── worlds/
│   └── robot.sdf            # Robot world from Lab 1
└── config/
    └── robot.rviz           # RViz2 configuration
```

**Key files:**
- `package.xml` - Lists all package dependencies (like `requirements.txt` in Python)
- `setup.py` - Tells `colcon build` what to install and where
- `setup.cfg` - Ensures executables install to correct location for `ros2 run`
- `resource/lab2` - Empty file required by ament_python packages

---

## Task 1: Create ROS2 Python Package

### Step 1.1: Create Package with ROS2 Command

```bash
cd /opt/ws/src/code

# Create ROS2 package with dependencies
ros2 pkg create --build-type ament_python --license Apache-2.0 \
  lab2 \
  --dependencies rclpy geometry_msgs sensor_msgs

cd lab2
```

This command creates the package structure with `package.xml` and `setup.py` files. However, we need to add more dependencies and configure data files.

### Step 1.2: Update `package.xml`

Edit `/opt/ws/src/code/lab2/package.xml` to add Gazebo and RViz2 dependencies:

The `ros2 pkg create` command already created a basic `package.xml`. You need to add these lines inside the `<package>` tag (after the existing `<exec_depend>` lines):

```xml
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>rviz2</exec_depend>
```

**Your final `package.xml` should look similar to [this example](https://classes.cs.uchicago.edu/archive/2025/fall/20600-1/ros2_intro.html).**

**Dependencies explained:**
- `rclpy` - ROS2 Python client library (already added)
- `geometry_msgs` - Twist messages for robot velocity (already added)
- `sensor_msgs` - LaserScan messages for LiDAR (already added)
- `ros_gz_bridge` - Connects Gazebo and ROS2 topics (you add)
- `ros_gz_sim` - Gazebo Sim integration (you add)
- `rviz2` - 3D visualization tool (you add)

### Step 1.3: Update `setup.py`

Edit `/opt/ws/src/code/lab2/setup.py` to add data files for launch, worlds, and config.

The `ros2 pkg create` command already created a basic `setup.py`. You need to modify it:

**Complete `setup.py` should look like:**

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lab2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy]*'))),
        (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@lpnu.ua',
    description='Lab 2: ROS2 Integration with Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = lab2.robot_controller:main',
            'lidar_subscriber = lab2.lidar_subscriber:main',
        ],
    },
)
```

**Key changes from the generated file:**
- Added `import os` and `from glob import glob` at the top
- Added three new entries in `data_files` list for launch, worlds, and config directories
- Added `entry_points` with two console scripts (executables you'll create)

### Step 1.4: Create `setup.cfg`

Create `/opt/ws/src/code/lab2/setup.cfg` with this content:

```ini
[develop]
script_dir=$base/lib/lab2
[install]
install_scripts=$base/lib/lab2
```

**Why is this needed?**
- By default, setuptools installs scripts to `bin/`
- ROS2's `ros2 run` looks for executables in `lib/<package_name>/`
- This file tells setuptools to install scripts to the correct location

**Without `setup.cfg`:** You'll get `No executable found` errors when running `ros2 run lab2 robot_controller`

### Step 1.5: Create Additional Directories

```bash
cd /opt/ws/src/code/lab2
mkdir -p launch worlds config
```

### Step 1.6: Copy Robot World

```bash
cp /opt/ws/src/code/lab1/worlds/robot.sdf /opt/ws/src/code/lab2/worlds/
```

Now your package structure is ready! Let's create the launch file next.

---

## Task 2: Create Launch File

Following [Launch Gazebo from ROS2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/).

### What is a Launch File?

A **launch file** starts multiple nodes at once with their parameters. Instead of running:
`ros2 run package1 node1 &
ros2 run package2 node2 &
ros2 run package3 node3 &`

You run: `ros2 launch package launch_file.py`

### Create `/opt/ws/src/code/lab2/launch/gazebo_ros2.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    lab2_pkg = FindPackageShare('lab2')
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')
    
    # File paths
    world_file = PathJoinSubstitution([lab2_pkg, 'worlds', 'robot.sdf'])
    rviz_config = PathJoinSubstitution([lab2_pkg, 'config', 'robot.rviz'])
    gz_sim_launch = PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        # 1. Launch Gazebo with robot world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': [world_file]}.items(),
        ),

        # 2. Bridge between Gazebo and ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        ),

        # 3. Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
```

**This launch file does three things:**
1. Starts Gazebo with your robot world
2. Creates `ros_gz_bridge` to connect `/lidar` and `/cmd_vel` topics
3. Starts RViz2 for visualization

**Bridge syntax:**
- `[` = one-way: Gazebo → ROS2 (sensor data)
- `@` = bidirectional: ROS2 ↔ Gazebo (commands and feedback)

---

## Task 3: Write ROS2 Publisher (Robot Controller)

Following [Writing a Simple Publisher](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

### What is a Publisher?

A **publisher** sends messages to a topic. In robotics:
- Control commands are published (velocity, position, etc.)
- The robot receives and executes them

### Create `/opt/ws/src/code/lab2/lab2/robot_controller.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # TODO: Create publisher for /cmd_vel topic
        # Hint: self.create_publisher(MessageType, 'topic_name', queue_size)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TODO: Create timer to publish at 10Hz (every 0.1 seconds)
        # Hint: self.create_timer(period_seconds, callback_function)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.counter = 0
        self.get_logger().info('Robot Controller started - publishing to /cmd_vel')

    def timer_callback(self):
        """Called every 0.1 seconds to publish velocity commands"""
        msg = Twist()
        
        # TODO: Set linear and angular velocities
        # Linear: msg.linear.x (forward/backward in m/s)
        # Angular: msg.angular.z (rotation in rad/s)
        
        # Example: Move forward with sinusoidal turning
        msg.linear.x = 0.5  # 0.5 m/s forward
        msg.angular.z = 0.3 * math.sin(self.counter * 0.1)  # Wavy motion
        
        # TODO: Publish the message
        self.publisher.publish(msg)
        
        self.counter += 1
        
        # Log every 5 seconds
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'Publishing: linear.x={msg.linear.x:.2f}, '
                f'angular.z={msg.angular.z:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key ROS2 concepts:**
- `Node` - Base class for all ROS2 nodes
- `create_publisher()` - Creates a topic publisher
- `create_timer()` - Calls a function at regular intervals
- `Twist` - Message type for velocity (linear + angular)
- `rclpy.spin()` - Keeps the node running until interrupted

**Try different movement patterns:**
- Circle: `msg.linear.x = 0.5; msg.angular.z = 0.3`
- Straight: `msg.linear.x = 0.5; msg.angular.z = 0.0`
- Spin in place: `msg.linear.x = 0.0; msg.angular.z = 1.0`

---

## Task 4: Write ROS2 Subscriber (LiDAR Processor)

Following [Writing a Simple Subscriber](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

### What is a Subscriber?

A **subscriber** receives messages from a topic. In robotics:
- Sensor data is subscribed to (camera, LiDAR, IMU, etc.)
- The node processes it for decision-making

### Create `/opt/ws/src/code/lab2/lab2/lidar_subscriber.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # TODO: Create subscriber for /lidar topic
        # Hint: self.create_subscription(MessageType, 'topic', callback, queue_size)
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )
        
        self.get_logger().info('LiDAR Subscriber started - listening to /lidar')

    def lidar_callback(self, msg):
        """Called automatically whenever a new LiDAR message arrives"""
        
        # TODO: Process the LiDAR data
        # msg.ranges is a list of distances (in meters)
        # msg.range_min and msg.range_max define valid range
        
        # Filter out invalid readings (too close or too far)
        valid_ranges = [
            r for r in msg.ranges 
            if msg.range_min < r < msg.range_max
        ]
        
        if valid_ranges:
            # TODO: Calculate statistics
            min_distance = min(valid_ranges)
            max_distance = max(valid_ranges)
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            
            self.get_logger().info(
                f'LiDAR: min={min_distance:.2f}m, '
                f'max={max_distance:.2f}m, '
                f'avg={avg_distance:.2f}m, '
                f'points={len(valid_ranges)}/{len(msg.ranges)}'
            )
            
            # TODO: Add obstacle detection
            # Warn if obstacle is very close
            if min_distance < 1.0:
                self.get_logger().warn(
                    f'Obstacle detected at {min_distance:.2f}m!'
                )
        else:
            self.get_logger().info('No valid LiDAR data')


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key ROS2 concepts:**
- `create_subscription()` - Creates a topic subscriber
- **Callback function** - Automatically called when message arrives
- `LaserScan` message contains:
  - `ranges[]` - Array of distance measurements
  - `range_min/max` - Valid range bounds
  - `angle_min/max` - Scan angle bounds

**Ideas to extend:**
- Find obstacles only in front (center indices)
- Calculate free space in different directions
- Publish processed data to a new topic

---

## Task 5: Create RViz2 Configuration

RViz2 uses configuration files (`.rviz`) to save display settings.

### Option A: Use the Provided Template (Recommended)

A basic RViz config is already provided in `/opt/ws/src/code/lab2/config/robot.rviz` with:
- **Grid display** - Shows coordinate frame
- **LaserScan display** - Visualizes LiDAR as colored points
- **Fixed Frame** - Set to `vehicle_blue/lidar_link/gpu_lidar`

### Option B: Create Your Own (Learning Exercise)

1. **Launch RViz2:**
   ```bash
   rviz2
   ```

2. **Set Fixed Frame:**
   - Global Options → Fixed Frame → `vehicle_blue/lidar_link/gpu_lidar`

3. **Add Grid:**
   - Add → Grid

4. **Add LaserScan:**
   - Add → LaserScan
   - Topic: `/lidar`
   - Size: `0.05`
   - Use rainbow: checked

5. **Save:**
   - File → Save Config As → `robot.rviz`

### RViz2 Resources

- [RViz2 User Guide](https://github.com/ros2/rviz/blob/ros2/docs/user_guide.md)
- [RViz2 Display Types](https://github.com/ros2/rviz/tree/ros2/rviz_default_plugins)

---

## Task 6: Build and Test

### Build the Package

```bash
# You should already be at /opt/ws when you enter the container

# Build your package
colcon build --packages-select lab2

# Source the workspace (makes your package visible)
source install/setup.bash
```

**Note:** Every time you modify `package.xml`, `setup.py`, or create new launch files, you must rebuild:
```bash
colcon build --packages-select lab2
source install/setup.bash
```

### If You Need to Clean and Rebuild

```bash
# Clean workspace
rm -rf ./build ./install ./log

# Rebuild everything
colcon build

# Or rebuild only lab2
colcon build --packages-select lab2

# Don't forget to source!
source install/setup.bash
```

### Launch Everything

```bash
ros2 launch lab2 gazebo_ros2.launch.py
```

**You should see:**
- Gazebo window with robot and obstacles
- RViz2 window with LiDAR visualization
- Both running together

### Test the Controller (New Terminal)

```bash
./scripts/cmd bash
source /opt/ws/install/setup.bash

# Run the robot controller
ros2 run lab2 robot_controller
```

**Watch the robot move in a sinusoidal pattern!**

### Test the Subscriber (Another New Terminal)

```bash
./scripts/cmd bash
source /opt/ws/install/setup.bash

# Run the LiDAR subscriber
ros2 run lab2 lidar_subscriber
```

**Watch the console for LiDAR statistics and obstacle warnings!**

---

## Task 7: Explore ROS2 Tools

### List and Inspect Nodes

```bash
# See all running nodes
ros2 node list

# Get detailed info about a node
ros2 node info /robot_controller
ros2 node info /lidar_subscriber
```

### List and Inspect Topics

```bash
# ROS2 topics
ros2 topic list

# Gazebo topics (for comparison)
gz topic -l

# See who's publishing/subscribing
ros2 topic info /cmd_vel
ros2 topic info /lidar

# See message structure
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan

# View messages in real-time
ros2 topic echo /cmd_vel
ros2 topic echo /lidar --once
```

### Publish Manually

```bash
# Control robot from command line
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10

# Press Ctrl+C to stop
```

## Understanding the Bridge

The `ros_gz_bridge` translates between Gazebo Transport and ROS2.

### Bridge Configuration

```
'/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```

**Format:** `topic@ros_type[gz_type`

- `topic` - Topic name (same on both sides)
- `ros_type` - ROS2 message type
- `gz_type` - Gazebo message type
- `[` - One-way: Gazebo → ROS2
- `]` - One-way: ROS2 → Gazebo
- `@` - Bidirectional

**For `/cmd_vel`:**
```
'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
```

Uses `@` for bidirectional (can send commands and receive feedback).

Reference: [ROS2 Integration](https://gazebosim.org/docs/harmonic/ros2_integration/)

---

## Troubleshooting

### "Package not found"

```bash
# Make sure you built the package
cd /opt/ws
colcon build --packages-select lab2

# Source the workspace
source install/setup.bash
```

### Bridge not connecting

```bash
# Check topics on both sides
ros2 topic list | grep lidar
gz topic -l | grep lidar

# Check bridge node
ros2 node info /parameter_bridge
```

### RViz2 shows no data

- Fixed Frame must match sensor frame
- Topic must be `/lidar` (check spelling)
- LaserScan display must be enabled (checkbox)
- Check QoS settings if using modified nodes

### Permission denied on Python files

```bash
chmod +x /opt/ws/src/code/lab2/lab2/*.py
```

---

## Resources

### ROS2 Official Tutorials
- [ROS2 Overview](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Writing Publishers and Subscribers](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

### Gazebo Integration
- [ROS2 Integration Overview](https://gazebosim.org/docs/harmonic/ros2_overview/)
- [Launch Gazebo from ROS2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [Use ROS2 to interact with Gazebo](https://gazebosim.org/docs/harmonic/ros2_integration/)

### Articulated Robotics
- [ROS Overview](https://articulatedrobotics.xyz/tutorials/ready-for-ros/ros-overview) - Excellent conceptual overview
- [YouTube Channel](https://www.youtube.com/@ArticulatedRobotics) - Great video tutorials
