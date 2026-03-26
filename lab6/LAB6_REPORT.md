# Lab 6 — Motion Planning with Nav2 (TurtleBot3, `room_nav2`)

This report summarizes **loading the stack**, **tasks completed** (through optional planner/controller comparisons), and **expected results**, in the same style as `LAB3_REPORT.md` / `LAB5_REPORT.md` and the assignment text in [`lab6/README.md`](README.md).

---

## 1. Project description

Lab 6 uses the **Nav2** stack in Gazebo: **map_server**, **AMCL**, **global planner**, **local controller** (DWB by default), **costmaps**, and **bt_navigator**. The world is **`room_nav2`** (8×8 m, walls and static box obstacles). The robot is **TurtleBot3 Burger** at the origin; navigation goals are sent from **RViz** after localization.

### 1.1 Learning goals (from assignment)

- Start Nav2 with a saved map and AMCL in simulation.
- Set **2D Pose Estimate** and **Nav2 Goal** in RViz; interpret global path vs local plan and costmaps.
- Tune `nav2_params` for smoother motion, clearer **local costmap**, and sensible goal stops.
- *(Optional)* Compare **global** planners (NavFn variants, Smac 2D) and **local** controllers (DWB vs Regulated Pure Pursuit).

### 1.2 References

- [Nav2 documentation](https://docs.nav2.org/)
- [Nav2 Planner Server](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
- [Nav2 Controller Server](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [Smac 2D Planner](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)
- [Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)

---

## 2. Package layout and config files

```
lab6/
├── config/
│   ├── nav2_params.yaml              # Tuned baseline: NavFn (A*) + DWB
│   ├── nav2_params_navfn_dijkstra.yaml  # NavFn, use_astar: false
│   ├── nav2_params_smac2d.yaml       # Smac Planner 2D (global)
│   └── nav2_params_rpp.yaml          # Regulated Pure Pursuit (local)
├── launch/
│   └── nav2_room_bringup.launch.py   # Gazebo room_nav2 + Nav2 + optional RViz; params_profile, nav2 delay
├── maps/
│   ├── room_nav2.yaml / room_nav2.pgm
└── worlds/
    └── room_nav2.sdf
```

| File | Role |
|------|------|
| `nav2_params.yaml` | Default profile: tuned costmaps, **DWB**, NavFn with A* (`use_astar: true`), goal checker, velocity smoother. |
| `nav2_params_navfn_dijkstra.yaml` | Same stack; **NavFn** with `use_astar: false` (Dijkstra-style expansion on the grid). |
| `nav2_params_smac2d.yaml` | Same local/controller setup; **SmacPlanner2D** as global planner ([docs](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)). |
| `nav2_params_rpp.yaml` | Same global planner as default (NavFn A*); **RegulatedPurePursuit** as `FollowPath` ([docs](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)). |
| `nav2_room_bringup.launch.py` | Sets `map`, `params_file`, `use_sim_time`, **`nav2_startup_delay`** (timer) so Nav2 starts after the sim; **`params_profile`** selects YAML. |

**Global vs local (short):**

- **Global** (`planner_server`): path on the **saved occupancy map**; shown in RViz as the long **global plan**.
- **Local** (`controller_server` + costmaps + sensors): tracks the path, avoids nearby obstacles using the **rolling local costmap**, outputs **`/cmd_vel`** (via smoother / collision monitor chain in this bringup).

---

## 3. Setup, build, and launch

Per `README.md`, first-time installs use the course Docker image with Nav2. Inside the workspace (e.g. `/opt/ws`):

```bash
colcon build --packages-select lab6
source install/setup.bash
ros2 launch lab6 nav2_room_bringup.launch.py
```

**Optional:** increase delay if lifecycle startup races with Gazebo (default is 7 s):

```bash
ros2 launch lab6 nav2_room_bringup.launch.py nav2_startup_delay:=12.0
```

Wait until **`lifecycle_manager_navigation`** logs that **managed nodes are active**, then in RViz set **Fixed Frame** to `map`, use **2D Pose Estimate**, then **Nav2 Goal**.

---

## 4. Task completion

### Task 1 — First navigation run

**Status:** Completed (procedure as in `README.md`).

1. Launch bringup; confirm Gazebo `room_nav2`, TurtleBot3, RViz.
2. **2D Pose Estimate** to align AMCL with the true pose.
3. **Nav2 Goal**; observe global path, local plan, and costmaps.

**Note:** If RViz reports `navigate_to_pose action server is not available`, Nav2 is not fully up yet (or lifecycle timed out during parallel boot). Wait for navigation lifecycle completion, or increase `nav2_startup_delay`.

### Task 2 — Parameter tuning (`nav2_params.yaml`)

**Status:** Completed.

The initial repo settings were intentionally harsh (slow costmap updates, coarse resolution, high speeds, very loose goal tolerances). The tuned baseline improves tracking, stability in sim, and goal behavior.

#### 4.2.1 Parameter experiments table (deliverable)

| Parameter(s) | Before (illustrative / original intent) | After (tuned) | What we expected / observed |
|--------------|----------------------------------------|---------------|-----------------------------|
| `local_costmap` **`update_frequency` / `publish_frequency`** | 2 Hz / 1 Hz | **12 Hz / 10 Hz** | Faster map refresh → **less blur / drift** of obstacles in RViz when the robot turns. |
| `local_costmap` **`resolution`** / **`width`×`height`** | 0.2 m, 3×3 m | **0.05 m**, **4×4 m** | Finer cells aligned with `room_nav2` map resolution; slightly larger window for context. |
| `global_costmap` **`resolution`**, **`update_frequency` / `publish_frequency`** | 0.2 m, 0.5 Hz | **0.05 m**, **2 Hz** | Global grid matches map scale; planner sees geometry consistent with the static map. |
| `controller_server` **`controller_frequency`** | 4 Hz | **20 Hz** | Snappier control loop, pairs better with faster local costmap updates. |
| `FollowPath` (DWB) **`max_vel_x` / `max_speed_xy`**, **`max_vel_theta`** | 1.2 m/s, 2.5 rad/s | **0.45 m/s**, **1.2 rad/s** | Slower, calmer motion in sim; less aggressive rotation → fewer laser/costmap artifacts. |
| `goal_checker` **`xy_goal_tolerance` / `yaw_goal_tolerance`** | 0.75 m / 1.25 rad | **0.2 m / 0.35 rad** | Goal completes with a **meaningful** final pose instead of “anywhere in a huge disk”. |
| `inflation_layer` **`inflation_radius` / `cost_scaling_factor`** (global & local) | 1.0 / 10 | **0.65 & 0.55 / 4** | Moderate clearance; smoother cost falloff for visualization and planning. |
| `velocity_smoother` **`max_velocity`** etc. | e.g. 0.5 linear, 2.0 yaw | **[0.45, 0, 1.2]** (aligned with DWB) | Limits match the controller so outputs are consistent end-to-end. |

#### 4.2.2 Drift / blur and goal stops (short text)

**Local map drift / blur** was addressed mainly by **raising local costmap update and publish rates**, using **resolution 0.05 m** (same as the map), and by **reducing angular velocity** so the lidar frame does not swing as aggressively through the voxel layer. **Better goal stops** came from **tightening `goal_checker` tolerances** (and matching DWB `xy_goal_tolerance`) instead of the original multi‑meter / multi‑radian settings that allowed Nav2 to “succeed” while poorly aligned.

**Global vs local:** fixing **global resolution** helps the **planner** respect narrow passages on the static map; fixing **local rates and speeds** helps **DWB** and the **local costmap** stay consistent with what the laser sees **while driving**.

### Task 3 — Global planner comparison (optional)

**Status:** Implemented via separate YAML files and launch profile.

| Planner setup | Config file | Launch command |
|---------------|-------------|----------------|
| NavFn + A* (tuned default) | `nav2_params.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py` |
| NavFn **Dijkstra-style** (`use_astar: false`) | `nav2_params_navfn_dijkstra.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py params_profile:=navfn_dijkstra` |
| **Smac 2D** | `nav2_params_smac2d.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py params_profile:=smac2d` |

**What to compare in RViz:** shape of the **global path** (corners, distance to inflated obstacles, choice of corridor). **Driving** feel still depends strongly on the **local controller** (DWB here).

### Task 4 — Local controller comparison (optional)

**Status:** Implemented via `nav2_params_rpp.yaml`; **`enable_stamped_cmd_vel: true`** retained for TurtleBot3 / Gazebo.

| Local controller | Config file | Launch command |
|------------------|-------------|----------------|
| **DWB** | `nav2_params.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py` |
| **Regulated Pure Pursuit** | `nav2_params_rpp.yaml` | `ros2 launch lab6 nav2_room_bringup.launch.py params_profile:=rpp` |

(`params_profile:=regulated_pp` is an alias of `rpp`.)

**What to compare:** path tracking smoothness, behavior near obstacles and in turns, and final approach / heading alignment at the goal.

---

## 5. Results and observations

After a successful run you should see:

1. **Gazebo** — TurtleBot3 executes the planned motion toward the goal in `room_nav2`, with recoveries only when blocked (within sim noise).
2. **RViz** — **global path** stable; **local costmap** follows obstacles without extreme smearing; **Footprint** and **LaserScan** consistent with walls and boxes.
3. **Terminal** — `bt_navigator` reports navigation start/finish; no persistent `change_state` timeouts from `controller_server` if `nav2_startup_delay` is sufficient.

**Screenshots (deliverable):** capture **before/after** tuning or side‑by‑side profiles (e.g. NavFn vs Smac path; DWB vs RPP at the same goal).

**If navigation never starts:** confirm `ros2 action list | grep navigate` shows `navigate_to_pose`, fix **2D Pose Estimate**, and ensure **`map` → `base_link`** TF is valid.

---

## 6. Deliverables checklist

| Deliverable | Where in this report |
|-------------|----------------------|
| Short description of loading / launch | §3 |
| Task 1 procedure | §4.1 |
| Table: ≥3 parameter edits + observations | §4.2.1 |
| Text: drift/blur, goal stops, global vs local | §4.2.2 |
| Optional: two global planner setups + commands | §4.3 |
| Optional: two local controller setups + commands | §4.4 |
| Screenshots | Attach from your runs (§5) |

---

## 7. Summary

Lab 6 connects the **full Nav2 pipeline** in simulation: **static map + AMCL + planner + DWB (default) + costmaps**. Work included **tuning `nav2_params.yaml`**, adding **optional profiles** for **NavFn/Dijkstra**, **Smac 2D**, and **Regulated Pure Pursuit**, and **deferring Nav2 bringup** with `nav2_startup_delay` to avoid lifecycle race with Gazebo. Rebuild **`lab6`**, **source** the workspace, run **`nav2_room_bringup.launch.py`**, localize, then send goals as described in **`README.md`**.
