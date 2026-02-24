## Lab 1 – Mobile Robot with LiDAR in Gazebo

This document summarizes the work done for **Lab 1** in the file `worlds/robot.sdf` and describes how to build, launch, and interact with the simulation.

### 1. World and Environment Setup

- **Physics and plugins**: The world `car_world` uses a standard physics configuration with system plugins for physics, user commands, scene broadcasting, and sensors (with the `ogre2` render engine to enable GPU sensors such as `gpu_lidar`).
- **Lighting**: A directional light named `sun` is added to illuminate the scene and provide realistic shading and visibility.
- **Ground plane**: A static `ground_plane` model with a large planar surface is included so the robot and obstacles rest on a flat floor and collisions with the ground are handled correctly.

### 2. Robot Model Configuration

- **Base model**: The robot `vehicle_blue` has a main chassis link with proper inertial parameters, a blue box visual, and a matching collision shape.
- **Four wheels**: There are four wheel links (two front, two rear) with cylindrical geometry, inertial properties, and revolute joints connecting each wheel to the chassis.
- **Differential drive plugin**: The `gz::sim::systems::DiffDrive` plugin is configured with:
  - Two left wheel joints (`left_front_wheel_joint`, `left_rear_wheel_joint`),
  - Two right wheel joints (`right_front_wheel_joint`, `right_rear_wheel_joint`),
  - Wheel separation `1.2` m and wheel radius `0.4` m,
  - Odometry publish frequency of `1` Hz,
  - Command topic `cmd_vel` of type `gz.msgs.Twist`.
  This plugin converts velocity commands on `/cmd_vel` into wheel rotations and robot motion.

### 3. Keyboard Control via Triggered Publishers

To control the robot without writing a separate node, several instances of the `gz::sim::systems::TriggeredPublisher` plugin are attached to `vehicle_blue`:

- Each instance subscribes to `/keyboard/keypress` (type `gz.msgs.Int32`) and watches for a specific key code in the `data` field.
- When the matching key is pressed, the plugin publishes a `gz.msgs.Twist` command to `/cmd_vel`:
  - One plugin sends a forward linear velocity (`x = 1`),
  - One sends a backward linear velocity (`x = -1`),
  - One turns left (angular `z = 1`),
  - One turns right (angular `z = -1`).
Together with the DiffDrive plugin, this provides basic teleoperation from the keyboard.

### 4. LiDAR Sensor Configuration

The robot is equipped with a GPU LiDAR sensor:

- **Sensor mounting**: A dedicated link `lidar_link` is created and positioned in front of and slightly above the chassis using a fixed joint `lidar_joint`. This ensures the LiDAR moves rigidly with the robot.
- **Sensor type and topic**: A sensor named `gpu_lidar` of type `gpu_lidar` is added to `lidar_link`. It publishes laser scan data to the topic `/lidar` at **10 Hz**.
- **Scan geometry**:
  - Horizontal scan with **640 samples**,
  - Horizontal field of view of about **±80°** around the forward direction,
  - Single vertical beam (2D LiDAR).
- **Range settings**: Minimum range `0.08` m, maximum range `10.0` m, with range resolution `0.01` m.
- **Visualization flag**: The sensor has `<visualize>true</visualize>`, allowing Gazebo’s GUI to display LiDAR data when the appropriate visualization panel is used.

### 5. Obstacles and Scene Layout

- Three static obstacle models are added to the world:
  - A **red cylinder** (pillar),
  - A **green box** (rectangular block),
  - A **blue sphere** (ball).
- Each obstacle has:
  - A visual geometry with a distinctive color,
  - A collision geometry approximating its shape,
  - A fixed pose in front of the robot so that the LiDAR can detect them while the robot moves.

### 6. Build and Launch Instructions

These steps assume you are in the repository root (`/home/viktor/robotics_lpnu`) on the host machine.

#### 6.1. Start and enter the Docker workspace

```bash
# 1. Start the lab container (from the repo root)
./scripts/cmd run

# 2. In a new terminal, attach to the running container
./scripts/cmd bash
```

Inside the container the working directory is typically `/opt/ws`.

#### 6.2. Build the workspace

Run these commands inside the container:

```bash
# 3. Build the colcon workspace
colcon build

# 4. Source the workspace so that Gazebo can find the lab packages
source install/setup.bash
```

#### 6.3. Launch the Gazebo scene

Still inside the container, start the simulation with the lab world:

```bash
# 5. Launch the Lab 1 Gazebo world
gz sim install/lab1/share/lab1/worlds/robot.sdf
```

This opens Gazebo Sim with the `car_world` scene containing `vehicle_blue`, the ground plane, light, and obstacles.

### 7. Interacting with the Robot and Sensors

#### 7.1. Keyboard teleoperation

In another terminal **inside the same container**, you can monitor keyboard events:

```bash
gz topic -e -t /keyboard/keypress
```

When the key publisher is active in Gazebo, pressing the arrow keys generates integer key codes on `/keyboard/keypress`. The `TriggeredPublisher` plugins convert these codes into appropriate `/cmd_vel` commands, making the robot drive forward, backward, and turn left or right.

#### 7.2. LiDAR topic

To inspect LiDAR data on `/lidar` while suppressing the unused intensity field:

```bash
gz topic -e -t /lidar | grep -v "intensities"
```

Here the `ranges` array shows measured distances to obstacles; values close to `range_max` correspond to no obstacle detected along that beam.


Also ensure X access is granted to Docker so the GUI can open:

This report can be used as a summary of the Lab 1 implementation and as a step-by-step guide to rebuild, launch, and interact with the mobile robot and its LiDAR sensor in Gazebo.

