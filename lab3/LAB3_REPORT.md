# Lab 3 – Moving Mobile Robots in Simulation

This report describes the work done in the `lab3` directory, the structure of the ROS2 package, the implemented path-following nodes, the completed tasks, and how to build and run the project.

---

## 1. Project Description

The `lab3` package is a ROS2 package for **path following and trajectory control** of differential-drive mobile robots in Gazebo simulation. It implements:

- **Differential drive kinematics** — conversion between (v, ω) and wheel speeds
- **Odometry-based path following** — square path with feedback from odometry
- **Time-based trajectories** — circle and figure-8 paths
- **RViz2 visualization** — trajectory display on `/path`

The package supports two robot setups:

| Robot | World | Launch file |
|-------|-------|-------------|
| Custom 4-wheel (`vehicle_blue`) | `worlds/robot.sdf` | `bringup.launch.py` |
| TurtleBot3 | `turtlebot3/worlds/room.sdf` | `turtlebot3_room_bringup.launch.py` |

---

## 2. Package Structure

```
lab3/
├── lab3/
│   ├── diff_drive_math.py      # Kinematics: twist ↔ wheel speeds, curve radius
│   ├── velocity_publisher.py   # Publish constant (v, w) for testing
│   ├── odom_path_publisher.py  # Odometry → Path for RViz2
│   ├── square_path.py          # Odometry-based square
│   ├── circle_path.py          # Timed circle
│   └── figure_8_path.py        # Timed figure-8 (two circles)
├── launch/
│   ├── gazebo.launch.py             # Gazebo + bridges only
│   ├── bringup.launch.py            # Gazebo, bridges, odom_path, RViz2
│   ├── turtlebot3_room.launch.py
│   └── turtlebot3_room_bringup.launch.py
├── turtlebot3/
│   ├── worlds/room.sdf              # 8×8 m room
│   ├── urdf/README.md
│   ├── xacro/simple_diff_drive.urdf.xacro
│   └── README.md
├── rviz/
│   └── trajectory.rviz              # Path + Odometry display
├── worlds/
│   └── robot.sdf                    # 4-wheel robot world
├── setup.py
├── package.xml
└── README.md
```

### 2.1 Main Files and Roles

| File | Role |
|------|------|
| `diff_drive_math.py` | `twist_to_wheel_speeds(v, w, R, L)` and `curve_radius(v, w)` |
| `velocity_publisher` | Publishes constant Twist to `/cmd_vel`, logs wheel speeds |
| `odom_path_publisher` | Subscribes to odometry, publishes Path to `/path` |
| `square_path` | Move forward + turn 90° (×4), uses odometry feedback |
| `circle_path` | Timed motion, one full circle (waits for odometry before start) |
| `figure_8_path` | Two circles: first left (w>0), then right (w<0) |

---

## 3. Task Completion

### Task 1: Run square and circle

**Status:** Completed

Both scripts run on the custom 4-wheel robot (`bringup.launch.py`) and TurtleBot3 (`turtlebot3_room_bringup.launch.py`).

| Command | Description |
|---------|-------------|
| `ros2 run lab3 square_path` | Draws a 2×2 m square using odometry feedback |
| `ros2 run lab3 circle_path` | Draws one full circle (timed motion) |

For TurtleBot3, use `odom_topic:=/odom` for `square_path`:

```bash
ros2 run lab3 square_path --ros-args -p odom_topic:=/odom
ros2 run lab3 circle_path --ros-args -p odom_topic:=/odom
```

### Task 2: Implement figure-8

**Status:** Completed

- **File:** `lab3/figure_8_path.py`
- **Logic:** Two circles, first left (w>0), then right (w<0), same timed motion as `circle_path`
- **Parameters:** `wheel_radius=0.4`, `wheel_separation=1.2` (matched to `robot.sdf`)
- **Run on both robots:**
  - 4-wheel: `bringup.launch.py` → `ros2 run lab3 figure_8_path`
  - TurtleBot3: `turtlebot3_room_bringup.launch.py` → `ros2 run lab3 figure_8_path`

### Task 3: RViz2 visualization

**Status:** Completed

- **Launch:** `ros2 launch lab3 bringup.launch.py` starts Gazebo, bridges, `odom_path_publisher`, and RViz2
- **Trajectory:** `odom_path_publisher` collects odometry and publishes to `/path`
- **RViz2:** `trajectory.rviz` displays Path (`/path`), Odometry, and Grid (Fixed Frame: `odom`)

To see the trajectory: run bringup, then run any path script in another terminal.

---

## 4. Script Modifications (What, How, Why)

During the lab, the following files were changed to meet the tasks and fix issues.

### 4.1 `square_path.py`

| Change | How | Why |
|--------|-----|-----|
| `angular_speed` | Reduced from 0.8 to **0.25** rad/s | Robot's odometry publishes at 1 Hz. With 0.8 rad/s the turn overshoots 90° (≈120°), producing a triangle. At 0.25 rad/s there are enough odom updates to stop near 90°. |
| Turn logic | Kept odometry-based turn (not time-based) | User preference: parameters adapted to robot, turn controlled by odometry. |

### 4.2 `circle_path.py`

| Change | How | Why |
|--------|-----|-----|
| Odometry wait | Added subscriber to `/model/vehicle_blue/odometry` and wait loop before starting | Without waiting, the node could start before the bridge was ready, causing an arc instead of a full circle. |
| `wheel_radius` | Changed from 0.15 to **0.4** | Match `robot.sdf` DiffDrive. |
| `wheel_separation` | Changed from 0.7 to **1.2** | Match `robot.sdf` DiffDrive. |

### 4.3 `figure_8_path.py`

| Change | How | Why |
|--------|-----|-----|
| Implementation | Implemented from stub: two circles, timed motion | Task 2: figure-8 = circle left (w>0) + circle right (w<0). |
| Order of circles | First left (w>0), then right (w<0) | Per requirements. |
| No odometry | Timed motion only, no odom subscription | Per requirements: "no odometry needed". |
| Parameters | `wheel_radius=0.4`, `wheel_separation=1.2` | Match `robot.sdf`. |

### 4.4 `worlds/robot.sdf`

| Change | How | Why |
|--------|-----|-----|
| Obstacles | Moved `red_pillar` (4,2)→(15,10), `green_block` (3,-5)→(15,-10), `blue_ball` (10,0)→(-12,5) | Path scripts (square 2×2 m, circle ~2 m, figure-8 ~4 m, velocity_publisher straight) stay in the center. Obstacles were moved away so they do not block these trajectories. |

---

## 5. Parameters (for robot tuning)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `side_length` | 2.0 | Square side length (m) |
| `linear_speed` | 0.4 | Forward speed (m/s) |
| `angular_speed` | 0.25 | Turn rate (rad/s) — tuned for 1 Hz odometry |
| `odom_topic` | `/model/vehicle_blue/odometry` | Odometry topic (use `/odom` for TurtleBot3) |
| `wheel_radius` | 0.4 | Wheel radius (m) — matches robot.sdf |
| `wheel_separation` | 1.2 | Distance between wheels (m) |

Example:

```bash
ros2 run lab3 square_path --ros-args -p side_length:=2.5 -p odom_topic:=/odom
```

---

## 6. Build and Run Instructions

### 6.1 Prerequisites

- Docker (for lab workspace)
- Optionally: TurtleBot3 packages (`ros-jazzy-turtlebot3`, `ros-jazzy-turtlebot3-simulations`)

### 6.2 Load and build the project

1. Start the Docker container and enter the workspace:

   ```bash
   ./scripts/cmd run
   # In a new terminal:
   ./scripts/cmd bash
   ```

2. Build the workspace (inside the container, `/opt/ws`):

   ```bash
   colcon build --packages-select lab3
   source install/setup.bash
   ```

3. (Optional) Install TurtleBot3 if needed:

   ```bash
   sudo apt update && sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations
   ```

### 6.3 Launch the simulation

#### Option A: Custom 4-wheel robot

```bash
ros2 launch lab3 bringup.launch.py
```

This starts:

- Gazebo with `worlds/robot.sdf`
- Bridges: `/cmd_vel`, `/model/vehicle_blue/odometry`
- `odom_path_publisher`
- RViz2 with `trajectory.rviz`

#### Option B: TurtleBot3

```bash
ros2 launch lab3 turtlebot3_room_bringup.launch.py
```

### 6.4 Run path scripts

In **another terminal** (with `source install/setup.bash`):

| Script | Command |
|--------|---------|
| velocity_publisher | `ros2 run lab3 velocity_publisher` |
| circle_path | `ros2 run lab3 circle_path` |
| square_path | `ros2 run lab3 square_path` |
| figure_8_path | `ros2 run lab3 figure_8_path` |

For TurtleBot3, add `-p odom_topic:=/odom` to `square_path` and `circle_path`.

---

## 7. Summary

- **Lab 3** implements path following for differential-drive robots: square (odometry-based), circle, and figure-8 (timed).
- **Tasks 1–3** are complete: square and circle run on both robots; figure-8 is implemented; RViz2 shows the trajectory on `/path`.
- Parameters are adapted to the custom robot (`robot.sdf`) and can be overridden for TurtleBot3.

---

## 8. Deliverables

### 8.1 Best parameters for square path

For the custom 4-wheel robot (`robot.sdf`) with odometry at 1 Hz:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `side_length` | 2.0 | Stable square size. |
| `linear_speed` | 0.4 | Good balance of speed and control. |
| `angular_speed` | **0.25** | Avoids turn overshoot; odometry at 1 Hz cannot track 0.8 rad/s accurately. |
| `odom_topic` | `/model/vehicle_blue/odometry` | For 4-wheel robot; use `/odom` for TurtleBot3. |

Example:

```bash
ros2 run lab3 square_path --ros-args -p angular_speed:=0.25
```

### 8.2 Brief answers

**What is differential drive?**  
A differential drive robot has two independently driven wheels on the same axis. Linear velocity (v) and angular velocity (ω) are produced by different left/right wheel speeds. The robot turns by rotating one wheel faster than the other; it moves straight when both wheels spin at the same speed.

**Why might the square drift?**  
The square can drift because:

- **Odometry errors** — wheel slip, uneven surface, or imperfect odometry model accumulate over time.
- **Turn overshoot/undershoot** — low odometry frequency (e.g. 1 Hz) or high angular speed make it hard to stop exactly at 90°.
- **Physical differences** — unequal wheel diameters or encoder resolution cause systematic drift.
