# Lab 5 – Obstacle Avoidance

This report summarizes the work for the `lab5` package: extending the TurtleBot3 room world with obstacles, implementing reactive navigation with lidar and odometry, launching the full stack in Gazebo, and tuning the behaviour toward a goal pose.

---

## 1. Project Description

Lab 5 implements **obstacle avoidance** for **TurtleBot3 Burger** in simulation. The robot must move from the spawn pose toward a goal using **2D lidar** (`sensor_msgs/LaserScan` on `/scan`) and **odometry** (`nav_msgs/Odometry` on `/odom`), while publishing **velocity commands** (`geometry_msgs/TwistStamped` on `/cmd_vel`) compatible with the Gazebo `ros_gz` bridge used in this course.

### 1.1 Learning Goals (from assignment)

- Implement obstacle avoidance for TurtleBot3.
- Use lidar (`/scan`) and odometry (`/odom`) to navigate to a goal.
- Publish velocity commands (`/cmd_vel`).

### 1.2 References

- Course materials linked in `lab5/README.md` (potential fields and obstacle-avoidance lectures).
- [SDF world format](https://gazebosim.org/docs/latest/sdf.html) for custom obstacles.

---

## 2. Package Structure

```
lab5/
├── lab5/
│   ├── __init__.py
│   └── obstacle_avoidance.py      # APF-style controller: scan + odom → cmd_vel
├── launch/
│   └── obstacle_avoidance_bringup.launch.py   # Gazebo, spawn, node, optional RViz2
├── rviz/
│   └── obstacle_avoidance.rviz
├── resource/
│   └── lab5
├── setup.py
├── package.xml
├── setup.cfg
├── README.md
└── LAB5_REPORT.md
```

### 2.1 Main Files and Roles

| File | Role |
|------|------|
| `obstacle_avoidance.py` | Subscribes to `/scan` and `/odom`; publishes `TwistStamped` on `/cmd_vel`; timer-driven control loop. |
| `obstacle_avoidance_bringup.launch.py` | Loads `lab3` room world, Gazebo server + GUI, `robot_state_publisher`, delayed TurtleBot3 spawn, obstacle-avoidance node with `use_sim_time` and goal parameters; optional RViz2. |
| `obstacle_avoidance.rviz` | Visualization preset for the lab (laser, TF, etc., as configured). |

### 2.2 Topics

| Topic | Type | Direction (controller) |
|-------|------|-------------------------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe |
| `/odom` | `nav_msgs/Odometry` | Subscribe |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Publish |

Using **`TwistStamped`** on `/cmd_vel` avoids mixing message types with the TurtleBot3 Gazebo bridge (which expects stamped commands in this setup). A topic that advertises both `Twist` and `TwistStamped` breaks tools such as `ros2 topic echo /cmd_vel` without an explicit type.

---

## 3. World Changes (`lab3/turtlebot3/worlds/room.sdf`)

Per the assignment, the **room world** was edited so the robot has real obstacles and a visible start/goal:

| Element | Description |
|---------|-------------|
| `obstacle_cylinder1` | Large grey cylinder (pillar), northwest area — stays off the main diagonal corridor from the origin. |
| `obstacle_box2` | Blue box on the floor — blocks part of open space, placement adjusted during iteration. |
| `obstacle_cylinder3` | Smaller brown cylinder — near the front-right region, forces detours. |
| `corridor_wall_n1` | Grey slab — narrows passage so the path is not a straight line everywhere (“corridor” effect). |
| `start_point` / `goal_point` | Green and red spheres marking start (**(0, 0)**) and goal (marker in world; align mental model with `goal_x` / `goal_y`). |

Earlier drafts added multiple corridor segments and extra clutter; these were **reduced** so the scene stays readable and tuning is easier, while still requiring real avoidance behaviour.

---

## 4. Algorithm (Artificial Potential Field–style)

The node in `obstacle_avoidance.py` uses a **combined force** in the **robot frame**:

1. **Attractive component** — proportional to the unit direction toward the goal, expressed in base coordinates (`angle_to_goal_local` from odometry yaw).
2. **Repulsive component** — for each valid lidar return closer than `safe_dist`, a repulsion term pushes away along the ray direction (magnitude grows with proximity, using an inverse-distance weighting similar in spirit to classical potential fields).

The resultant `(F_x, F_y)` is turned into:

- **Linear velocity** `v` from the forward component (clamped, e.g. to ~0.2 m/s).
- **Angular velocity** `w` from the heading implied by the force (`atan2` scaling), clamped (e.g. ±1 rad/s).

**Goal stopping:** when distance to goal is below a threshold (about **0.2 m** in the implementation), the node publishes zero velocity and logs **“Goal reached!”** (throttled so the console is not flooded).

**Escape heuristic:** if the robot is far from the goal but barely moves (`v` and `w` very small), a small fixed turn rate is applied to reduce the chance of getting stuck in a degenerate configuration.

**Caveats:** potential-field methods can suffer from **local minima** (robot oscillates or stops away from the goal). If that happens in simulation, tune `k_att`, `k_rep`, `safe_dist`, or obstacle layout.

---

## 5. Build, Source, and Launch

### 5.1 Prerequisites

- Workspace available (including Docker workflow from `commands.txt` if used).
- Packages **`lab3`** (world file share path) and **`lab5`** built together.

### 5.2 Build and environment

From the workspace root (inside the container, typically `/opt/ws`):

```bash
colcon build --packages-select lab3 lab5
source install/setup.bash
```

### 5.3 Run the full demo

```bash
ros2 launch lab5 obstacle_avoidance_bringup.launch.py
```

This starts:

- Gazebo with `room.sdf` (obstacles and markers).
- TurtleBot3 Burger spawn at **(0, 0)** after a short delay.
- `robot_state_publisher` with `use_sim_time:=true`.
- `obstacle_avoidance` with `use_sim_time:=true` and launch-file goal parameters.
- RViz2 when `rviz:=true` (default).

Optional: disable RViz for a lighter session:

```bash
ros2 launch lab5 obstacle_avoidance_bringup.launch.py rviz:=false
```

### 5.4 Tuning the goal without editing code

The launch file passes **`goal_x:=3.0`** and **`goal_y:=2.0`**. Override at runtime if needed:

```bash
ros2 run lab5 obstacle_avoidance --ros-args -p use_sim_time:=true -p goal_x:=3.0 -p goal_y:=3.0
```

It is good practice to match **`goal_x` / `goal_y`** to the **red marker** pose in `room.sdf` so Gazebo and the controller agree on the target.

---

## 6. Parameters (controller)

| Symbol / parameter | Meaning |
|--------------------|---------|
| `goal_x`, `goal_y` | Goal position in the odometry frame (ROS params; launch defaults **3.0**, **2.0**). |
| `scan_topic`, `odom_topic`, `cmd_vel_topic` | Topic names (defaults `/scan`, `/odom`, `/cmd_vel`). |
| `k_att` | Attractive gain toward the goal (implementation constant). |
| `k_rep` | Repulsive gain from lidar (implementation constant). |
| `safe_dist` | Influence distance for repulsion (m). |
| Control timer | Fixed step in code (e.g. **0.1 s**); sets reactivity of the loop. |

---

## 7. Results and Observation

After a successful run you should observe:

1. **Gazebo** — TurtleBot3 drives from the start region toward the goal, slows near obstacles, and passes through narrowed regions without prolonged wall contact (depending on tuning).
2. **Terminal** — when the goal tolerance is met, a **“Goal reached!”** log appears (throttled).
3. **RViz** — laser scan and TF consistent with motion in the room (if RViz is launched).

**Screenshots (deliverable):** capture **Gazebo** (and optionally **RViz**) showing the robot **near the goal** after navigating past cylinders, the box, and the corridor slab.

**If behaviour is poor:** increase repulsion near walls, reduce linear cap, or widen gaps in `room.sdf`; verify `/scan` and `/odom` publish steadily (`ros2 topic hz /scan /odom`).

---

## 8. Deliverables Checklist

| Deliverable | Status |
|-------------|--------|
| Implemented `obstacle_avoidance.py` (reactive navigation, APF-style) | Done |
| Modified `room.sdf` with obstacles + start/goal markers | Done |
| Launch file + `use_sim_time` + goal parameters | Done |
| Brief description of algorithm and parameters | This report, §4 and §6 |
| Screenshots: robot reaching goal while avoiding obstacles | To attach from your runs |

---

## 9. Summary

Lab 5 connects **perception** (2D lidar), **state** (odom pose), and **actuation** (`TwistStamped` on `/cmd_vel`) in a single ROS 2 node. The **room world** was enriched with static models so avoidance is non-trivial; the **controller** follows an artificial-potential style law with goal stopping and logging. Building **`lab3`** and **`lab5`**, sourcing the workspace, and running **`obstacle_avoidance_bringup.launch.py`** reproduces the full assignment pipeline described in **`lab5/README.md`**.