# Lab 4 – Dead Reckoning

This report describes the work done in the `lab4` directory, the structure of the ROS2 package, the implemented dead reckoning node, build and run instructions, and deliverables.

---

## 1. Project Description

The `lab4` package implements **dead reckoning** — estimating the robot's pose by integrating velocity commands from `/cmd_vel`. The result is compared with Gazebo ground-truth odometry to observe drift.

### 1.1 Learning Goals

- Integrate velocity commands (v, ω) to estimate pose
- Compare dead reckoning with Gazebo ground truth
- Understand causes of drift

### 1.2 Reference

- [Motion Model for Differential Drive](https://www.roboticsbook.org/S52_diffdrive_actions.html)

---

## 2. Package Structure

```
lab4/
├── lab4/
│   ├── __init__.py
│   └── dead_reckoning.py      # Integrates /cmd_vel, publishes Path to /path_dr
├── launch/
│   └── dead_reckoning_bringup.launch.py   # Gazebo, TurtleBot3, odom_path, dead_reckoning, RViz2
├── rviz/
│   └── dead_reckoning.rviz    # Path (odom) + Path (dead reckoning) displays
├── resource/
│   └── lab4
├── setup.py
├── package.xml
├── setup.cfg
└── README.md
```

### 2.1 Main Files and Roles

| File | Role |
|------|------|
| `dead_reckoning.py` | Subscribes to `/cmd_vel`, integrates (v, ω), publishes Path to `/path_dr`; subscribes to `/odom` for ground truth |
| `dead_reckoning_bringup.launch.py` | Starts Gazebo, TurtleBot3, `odom_path_publisher`, `dead_reckoning` node, RViz2 |
| `dead_reckoning.rviz` | Shows `/path` (odom) and `/path_dr` (dead reckoning) in different colors |

### 2.2 Topic Diagram

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | TwistStamped | circle_path (lab3) | dead_reckoning, Gazebo bridge |
| `/odom` | Odometry | Gazebo (via bridge) | dead_reckoning, odom_path_publisher |
| `/path` | Path | odom_path_publisher | RViz2 |
| `/path_dr` | Path | dead_reckoning | RViz2 |

---

## 3. Implementation Details

### 3.1 Dead Reckoning Algorithm

The pose is updated using the differential-drive motion model:

```
x'     = x  + v·cos(θ)·Δt
y'     = y  + v·sin(θ)·Δt
θ'     = θ  + ω·Δt
```

- `(x, y, θ)` — current pose in `odom` frame
- `(v, ω)` — linear and angular velocity from `/cmd_vel`
- `Δt` — time step between messages

### 3.2 Supported Message Types

The node supports both `TwistStamped` and `Twist` for `/cmd_vel`:

- **TwistStamped** (default): uses `header.stamp` for integration
- **Twist**: uses `get_clock().now()` as timestamp

Set `cmd_vel_stamped:=false` if `/cmd_vel` publishes `Twist`.

### 3.3 Features

- Periodic path republishing (5 Hz) so `/path_dr` is always visible
- Initial empty path published at startup
- `max_poses` limits path length to avoid unbounded memory use
- Subscribes to `/odom` for optional ground-truth comparison

---

## 4. Changes to Files (What, How, Why)

During the lab, the following changes were made.

### 4.1 `lab4/__init__.py`

| Change | How | Why |
|--------|-----|-----|
| Created file | Added `__init__.py` in `lab4/lab4/` | Without it, Python does not treat `lab4` as a package; `ros2 run lab4 dead_reckoning` failed with `ModuleNotFoundError: No module named 'lab4'`. |

### 4.2 `lab4/dead_reckoning.py`

| Change | How | Why |
|--------|-----|-----|
| Support for Twist and TwistStamped | Parameter `cmd_vel_stamped`; `_cmd_stamped_cb` for TwistStamped, `_cmd_twist_cb` for Twist | TurtleBot3 and other setups may publish either type on `/cmd_vel`; both need to work. |
| Periodic path republishing | Timer 0.2 s calls `_timer_publish()` | Path was published only when `/cmd_vel` arrived; when `circle_path` finished, `/path_dr` stopped, and RViz showed "topic does not appear to be published". Timer keeps publishing so the path stays visible. |
| Initial empty path at startup | Publish empty Path immediately after creating the publisher | Ensures `/path_dr` appears as soon as the node starts, so RViz and `ros2 topic hz /path_dr` work. |
| Relaxed dt limit | Use `dt < 2.0` instead of `dt < 1.0` | Allows larger gaps between `/cmd_vel` messages without skipping integration. |

---

## 5. Build and Run Instructions

### 5.1 Prerequisites

- Docker (workspace runs in container)
- Built `lab3` package (for `circle_path` and `odom_path_publisher`)

### 5.2 Step-by-Step Installation

1. **Start the Docker container:**
   ```bash
   ./scripts/cmd run
   ```

2. **Enter the workspace in a new terminal:**
   ```bash
   ./scripts/cmd bash
   ```

3. **Build packages** (inside container, at `/opt/ws`):
   ```bash
   colcon build --packages-select lab3 lab4
   source install/setup.bash
   ```

### 5.3 Launch the Simulation

**Terminal 1:**

```bash
ros2 launch lab4 dead_reckoning_bringup.launch.py
```

This launches:

- Gazebo with TurtleBot3 in `room.sdf`
- Parameter bridge (clock, odom, cmd_vel, joint_states, tf, imu, scan)
- `odom_path_publisher` (odom → `/path`)
- `dead_reckoning` (cmd_vel → `/path_dr`)
- RViz2 with `dead_reckoning.rviz`

### 5.4 Run Circle Trajectory

**Terminal 2** (with `source install/setup.bash`):

```bash
ros2 run lab3 circle_path --ros-args -p odom_topic:=/odom -p use_sim_time:=true
```

### 5.5 Observe

- **Green path** (`/path`) — odometry (ground truth)
- **Red path** (`/path_dr`) — dead reckoning

---

## 6. Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_vel_topic` | `/cmd_vel` | Topic for velocity commands |
| `cmd_vel_stamped` | true | true = TwistStamped, false = Twist |
| `ground_truth_topic` | `/odom` | Odometry topic for comparison |
| `path_dr_topic` | `/path_dr` | Output path topic |
| `frame_id` | `odom` | Frame for path messages |
| `max_poses` | 2000 | Maximum number of poses in path |

Example:

```bash
ros2 run lab4 dead_reckoning --ros-args \
  -p cmd_vel_stamped:=false \
  -p use_sim_time:=true
```

---

## 7. Summary

- **Lab 4** implements dead reckoning by integrating `/cmd_vel` and publishing the estimated path to `/path_dr`.
- RViz shows both odometry path (green) and dead reckoning path (red).
- The implementation supports TwistStamped and Twist, and uses periodic republishing for stable visualization.

---

## 8. Deliverables

### 8.1 Implemented `dead_reckoning.py`

- Integrates (v, ω) from `/cmd_vel`
- Publishes `nav_msgs/Path` to `/path_dr`
- Subscribes to `/odom` for ground-truth pose
- Supports both TwistStamped and Twist

### 8.2 Screenshot

Screenshot of RViz showing both paths (odom and dead reckoning) during or after `circle_path` execution.

### 8.3 Why Does Dead Reckoning Drift?

Dead reckoning drifts because:

1. **No feedback** — only velocity commands are integrated; actual motion (wheel slip, surface friction, uneven terrain) is not measured.
2. **Wheel slip** — real/simulated wheels slip; commanded velocities do not match actual motion.
3. **Discretization** — integration uses finite Δt; timing jitter and sampling add errors.
4. **No correction** — unlike odometry, there is no correction from encoders or sensors; errors accumulate over time.
5. **Model mismatch** — the differential-drive model assumes ideal motion; real robots have asymmetric wheels, loading, and mechanical imperfections.
