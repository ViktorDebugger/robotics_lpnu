# Laboratory 8 Report

## SO-101 Serial Bridge, Joint Goals, and End-Effector Control

Course: Robotics (ROS 2)  
Package: `lab8`

---

## 1. Objective

The objective of this laboratory is to operate the physical SO-101 manipulator through a ROS 2 serial bridge, visualize the arm with URDF and TF, and command motion in joint space and in task space. The work includes:

- reading live joint positions from servos and publishing `sensor_msgs/JointState` on `/joint_states`;
- mapping raw servo ticks to joint limits using a calibration YAML;
- sending validated joint goals on `/arm_goal_ticks` and executing multi-step motions from YAML;
- publishing forward-kinematics end-effector pose (`/ee_pose`) and a visualization marker (`/ee_marker`);
- sending Cartesian-style goals on `/ee_goal` and resolving them with a simplified inverse-kinematics model;
- safe rejection of out-of-limit commands before hardware writes.

---

## 2. Theory

The SO-101 arm in this package is modeled with six actuated joints in fixed order: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, and `gripper`. Joint angles for the first five are handled in radians; the gripper is normalized to \([0,1]\).

**Joint space** control specifies the vector \(\mathbf{q} = (q_0,\ldots,q_5)^T\) directly. **Task space** control specifies a desired end-effector position \((x,y,z)\) in a base-centric frame; the provided IK (`ik_position`) solves for \(\mathbf{q}\) under a simplified geometric model (fixed wrist pitch for teaching, roll set to zero in the analytic IK path used here).

**Calibration** maps servo encoder ticks in \([0,4095]\) (clamped) to joint limits by linear interpolation between `range_min` and `range_max` per joint in `motor_calibration.yaml`.

The `JointStateReader` node is the central bridge: it polls servos over serial, publishes state, subscribes to joint goals and EE goals, converts commands to ticks, and writes goal packets to the bus.

---

## 3. Package Structure and Roles

| Path | Role |
|------|------|
| `lab8/joint_state_reader.py` | Serial I/O, tick↔joint mapping, `/joint_states`, goal subscribers (`/arm_goal_ticks`, `/ee_goal`), FK publishers (`/ee_pose`, `/ee_marker`) |
| `lab8/kinematics.py` | Teaching FK (`fk_pose`) and simplified IK (`ik_position`) for SO-101 |
| `lab8/limits.py` | Joint order, limits, safe presets, `validate_joint_goal` |
| `lab8/send_goal.py` | CLI: send one validated joint goal (presets or explicit six values) |
| `lab8/run_motion.py` | CLI: play waypoint list from YAML |
| `lab8/send_ee_goal.py` | CLI: publish `PoseStamped` goal on `/ee_goal` |
| `launch/so101_real.launch.py` | Bringup: `joint_state_reader`, `robot_state_publisher` |
| `urdf/so101_new_calib.urdf` | Robot description (meshes under `urdf/assets/`, `package://lab8/...`) |
| `config/motor_calibration.yaml` | Per-joint raw tick range for mapping to joint limits |
| `config/pick_place_basic.yaml` | Example waypoint motion (joint targets and hold times) |
| `config/lab8.rviz` | RViz layout for grid, robot model, EE marker |

---

## 4. Procedure and Results

### 4.1 Build and environment

Commands used (adjust workspace path to your machine or Docker layout):

```bash
cd /opt/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lab8 --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Dependencies: ROS 2 Jazzy, Python 3, `python3-serial` and `python3-yaml` available in the environment (system packages or image).

### 4.2 Real robot bringup

Commands used:

```bash
ros2 launch lab8 so101_real.launch.py \
  port:=/dev/ttyACM0 \
  baud_rate:=1000000
```

Expected outcome:

- `joint_state_reader` connects to the serial port (or logs an error if the port is missing or busy).
- `robot_state_publisher` loads the bundled URDF string and initializes the robot model.
- TF and `/joint_states` update when servos are readable.

**Note:** The serial device must be visible inside the runtime environment (e.g. Docker `--device` for `/dev/ttyACM0`). On the host, verify with `ls /dev/ttyACM*` before launching.

### 4.3 Joint observation (required first step)

Commands used:

```bash
ros2 topic echo /joint_states
```

Procedure:

- Move one joint at a time (manually or with small safe commands) and confirm name order and sign of `position` entries.
- Record approximate zero or rest behavior before sending larger goals.

**Result (to fill in the report):** tabulate one configuration: joint names and measured `position` values after a known pose.

### 4.4 Safe joint goals and presets

Commands used:

```bash
ros2 run lab8 send_goal -- --preset ready
ros2 run lab8 send_goal -- --joints 0.2 -1.2 1.2 0.4 0.0 0.7
ros2 run lab8 send_goal -- --preset ready --shoulder_pan 0.3 --gripper 0.2
```

Expected outcome:

- Goals outside `JOINT_LIMITS` are rejected by `joint_state_reader` with a warning in the log.
- In-range goals produce servo motion consistent with calibration.

**Result (to fill in):** describe one rejected goal (which joint, limit message) and one accepted goal (before/after joint values from `/joint_states`).

### 4.5 YAML motion program

Commands used:

```bash
ros2 run lab8 run_motion -- --file "$(ros2 pkg prefix lab8)/share/lab8/config/pick_place_basic.yaml"
```

Procedure:

- Edit `config/pick_place_basic.yaml` with additional waypoints (`joints`, `hold_sec`, optional `name`).
- Re-run `run_motion` and observe sequencing.

**Result (to fill in):** number of waypoints, total duration, qualitative description of gripper open vs close steps.

### 4.6 End-effector goals and forward kinematics

Commands used:

```bash
ros2 run lab8 send_ee_goal -- --x 0.26 --y 0.00 --z 0.08
ros2 topic echo /ee_pose --once
```

Recommended beginner workspace from assignment notes:

- \(x \in [0.15, 0.28]\), \(y \in [-0.10, 0.10]\), \(z \in [0.05, 0.22]\).

**Result (to fill in):** one reachable EE goal: commanded \((x,y,z)\), resulting `/ee_pose`, and whether `joint_state_reader` logged IK success or rejection.

---

## 5. Discussion

**Strengths of the pipeline**

- Clear separation between **state** (`/joint_states`) and **commands** (`/arm_goal_ticks`, `/ee_goal`).
- Validation layer (`validate_joint_goal`) reduces risk of sending impossible targets to hardware.
- Calibration file decouples raw servo counts from URDF joint limits.

**Limitations**

- The analytic model in `kinematics.py` is a **teaching** simplification; it may not match the physical arm or full URDF dynamics exactly.
- IK from `/ee_goal` primarily uses position \((x,y,z)\); orientation fields in `PoseStamped` are not fully exploited by the simplified IK path.
- Serial communication and servo protocol are sensitive to wiring, baud rate, power, and concurrent access to the port.
- RViz visualization depends on correct `robot_description`, mesh paths, and a consistent fixed frame (`base_link` / `base`).

**Docker and permissions**

- If `/dev/ttyACM0` exists on the host but not in the container, the bridge cannot open the port until the device is passed through and the user has rights (e.g. `dialout`).

---

## 6. Figures (to attach)

- **Figure 1** — SO-101 hardware setup and USB connection.
- **Figure 2** — Terminal: successful `ros2 launch lab8 so101_real.launch.py` (no node crashes).
- **Figure 3** — `ros2 topic echo /joint_states` while moving one joint.
- **Figure 4** — RViz view: robot model and TF tree for SO-101.
- **Figure 5** — `send_goal` preset `ready` (or home): joint values before/after.
- **Figure 6** — Log excerpt: rejected goal with limit reason.
- **Figure 7** — Edited `pick_place_basic.yaml` and terminal running `run_motion`.
- **Figure 8** — `send_ee_goal` command and corresponding motion snapshot.
- **Figure 9** — `ros2 topic echo /ee_pose` after an EE command.
- **Figure 10** — RViz `Marker` display for `/ee_marker` (if enabled in config).

---

## 7. Conclusion

In this laboratory, the SO-101 arm was integrated with ROS 2 through a serial bridge that publishes joint state, accepts joint and EE goals, and drives servos using calibrated tick mapping. Safe limits and presets support incremental testing from observation to scripted motions. Forward kinematics and a simplified IK path connect task-space intents to executable joint commands. Remaining discrepancies between the analytic model and the real device should be interpreted in light of calibration accuracy and the teaching-level kinematics assumptions.

---

## 8. References

1. ROS 2 Jazzy documentation: <https://docs.ros.org/en/jazzy/>  
2. LeRobot SO-101 documentation: <https://huggingface.co/docs/lerobot/so101>  
3. SO-ARM100 SO101 URDF source: <https://github.com/TheRobotStudio/SO-ARM100>  
4. Package assignment source: `lab8/README.md`
