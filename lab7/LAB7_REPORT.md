# Laboratory 7 Report

## Coordinate Transforms (TF2), URDF/Xacro, and RTR Manipulator

Course: Robotics (ROS 2)  
Package: `lab7`

---

## 1. Objective

The objective of this laboratory is to implement and validate TF2-based pose handling for an RTR manipulator, describe the robot with URDF/Xacro, and connect joint states to TF through ROS 2 Control. The work includes:

- dynamic TF broadcasting and listening with analytical verification;
- URDF/Xacro modeling and visualization in RViz2;
- ROS 2 Control integration via mock hardware, `joint_state_broadcaster`, and `forward_position_controller`;
- test-based validation of kinematics utilities.

---

## 2. Theory

The RTR manipulator has three degrees of freedom:

- \( \theta_1 \): revolute base yaw;
- \( \theta_2 \): prismatic vertical translation;
- \( \theta_3 \): revolute elbow joint.

Forward kinematics for end-effector position \( \mathbf{p} = (x,y,z)^T \):

\[
x=\cos\theta_1\,(l_3\cos\theta_3+l_2),\quad
y=\sin\theta_1\,(l_3\cos\theta_3+l_2),\quad
z=l_3\sin\theta_3+\theta_2
\]

TF2 is used to publish and query transforms between coordinate frames, while URDF/Xacro defines robot links, joints, and frame hierarchy.

---

## 3. Package Structure and Roles

| Path | Role |
|------|------|
| `lab7/rtr_kinematics.py` | Analytical forward kinematics, orientation quaternion, pose composition, TF-vs-analytic matcher |
| `lab7/tf2_demo_cli.py` | Shared CLI parser for `theta_1 theta_2 theta_3 [l2] [l3]` |
| `lab7/tf2_broadcaster_demo.py` | Publishes dynamic transform `world -> rtr_ee_demo` |
| `lab7/tf2_listener_demo.py` | Reads TF transform and checks agreement with analytical model |
| `urdf/rtr_manipulator.xacro` | RTR URDF/Xacro model with ROS 2 Control mock hardware |
| `launch/rtr_visualize.launch.py` | Visualization stack (`joint_state_publisher_gui`, `robot_state_publisher`, RViz2) |
| `launch/rtr_ros2_control.launch.py` | ROS 2 Control stack (`ros2_control_node`, spawners, RViz2) |
| `config/rtr_controllers.yaml` | Controller manager and forward-command controller configuration |
| `tests/test_rtr_kinematics.py` | Unit tests for closed-form forward kinematics |
| `tests/test_tf2_analytic_agreement.py` | Unit tests for pose and TF/analytic agreement logic |

---

## 4. Procedure and Results

### 4.1 Build and environment

Commands used:

```bash
cd /opt/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lab7 --symlink-install
source install/setup.bash
```

### 4.2 Part A: TF2 broadcaster/listener validation

Commands used:

```bash
ros2 run lab7 tf2_broadcaster_demo -- 0.2 0.5 0.35
ros2 run lab7 tf2_listener_demo -- 0.2 0.5 0.35
ros2 run tf2_ros tf2_echo world rtr_ee_demo
```

Analytical computation for \( \theta_1=0.2,\ \theta_2=0.5,\ \theta_3=0.35,\ l_2=0.9,\ l_3=1.0 \):

\[
\mathbf{p}_{analytic}\approx (1.802701,\ 0.365445,\ 0.842898)
\]

TF output (rounded): \( (1.803,\ 0.365,\ 0.843) \).

| Quantity | Analytical | TF (`tf2_echo`) | Absolute difference |
|---|---:|---:|---:|
| \(x\) | 1.802701 | 1.803 | 0.000299 |
| \(y\) | 0.365445 | 0.365 | 0.000445 |
| \(z\) | 0.842898 | 0.843 | 0.000102 |

Result: TF and analytical values agree within numerical rounding tolerance.

### 4.3 Part B: URDF/Xacro visualization and `tool0` comparison

Command used:

```bash
ros2 launch lab7 rtr_visualize.launch.py
```

In `joint_state_publisher_gui`, joint values were changed interactively and TF frames were observed in RViz.

#### Xacro refinement performed

File modified: `urdf/rtr_manipulator.xacro`

- updated parameters: `l2=1.0`, `l3=1.1`, `width=0.07`;
- added basic `<collision>` geometry blocks for:
  - `base_link`
  - `link_yaw`
  - `link_carriage`
  - `link_upper`
  - `link_forearm`

Verification command:

```bash
grep -n "<collision>" /opt/ws/src/code/lab7/urdf/rtr_manipulator.xacro
```

Expected output: five `<collision>` entries.

### 4.4 Part C: ROS 2 Control and joint-command pipeline

Commands used:

```bash
ros2 launch lab7 rtr_ros2_control.launch.py
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2, 0.6, 0.4]}"
ros2 run tf2_ros tf2_echo base_link tool0
```

`joint_state_broadcaster` reads position state interfaces from mock hardware and publishes `/joint_states`.  
`robot_state_publisher` consumes `/joint_states` and URDF to publish the TF tree.  
This confirms the chain:

mock hardware -> `joint_state_broadcaster` -> `/joint_states` -> `robot_state_publisher` -> TF

### 4.5 Automated checks

Command used:

```bash
colcon test --packages-select lab7
```

Result: package tests finished successfully.

---

## 5. Discussion

Agreement between TF and analytical forward kinematics was checked for at least two configurations:

1. **Configuration A** (`world -> rtr_ee_demo`):  
   \( \theta_1=0.2,\ \theta_2=0.5,\ \theta_3=0.35,\ l_2=0.9,\ l_3=1.0 \).  
   TF and analytical values matched with sub-millimeter-scale numerical differences due to output rounding.

2. **Configuration B** (`base_link -> tool0`, control scenario):  
   TF output was collected after command publication and remained consistent with the expected kinematic behavior of the updated RTR chain in RViz and TF.

Model limitations:

- simplified rigid-body model with ideal joints;
- mock hardware instead of physical sensors/actuators;
- no friction/backlash/noise effects;
- basic collision geometry intended for structural validation, not high-fidelity contact simulation.

---

## 6. Figures (to attach)

- **Figure 1** - Kinematic diagram of the RTR manipulator with \( \theta_1,\theta_2,\theta_3 \).
- **Figure 2** - TF2 broadcaster command/output (`world -> rtr_ee_demo`).
- **Figure 3** - TF2 listener output with analytical agreement check.
- **Figure 4** - `tf2_echo world rtr_ee_demo` transform output.
- **Figure 5** - `joint_state_publisher_gui` with `joint_theta1`, `joint_theta2`, `joint_theta3`.
- **Figure 6** - RViz2 TF view of RTR links (`base_link` to `tool0`).
- **Figure 7** - Updated Xacro parameters (`l2`, `l3`, `width`) in `rtr_manipulator.xacro`.
- **Figure 8** - Terminal/editor evidence of added `<collision>` blocks.
- **Figure 9** - RViz2 view after Xacro refinement.
- **Figure 10** - `tf2_echo base_link tool0` output for updated model.
- **Figure 11** - `rtr_ros2_control.launch.py` runtime (RViz/control stack).
- **Figure 12** - terminal confirmation of published joint command to `/forward_position_controller/commands`.
- **Figure 13** - TF output confirming post-command end-effector transform.

---

## 7. Conclusion

In this laboratory, TF2 broadcasting/listening, URDF/Xacro modeling, and ROS 2 Control integration were implemented and validated for an RTR manipulator. Analytical forward kinematics matched TF output for tested configurations, confirming correctness of the kinematic pipeline. The model was refined by updating geometric parameters and adding basic collision geometry, and all package tests completed successfully. Overall, the lab provided practical experience with coordinated use of kinematics, TF, robot description files, launch systems, and controller-based joint-state propagation in ROS 2.

---

## 8. References

1. ROS 2 Jazzy TF2 docs: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html>  
2. ROS 2 Jazzy URDF docs: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html>  
3. Package assignment source: `lab7/README.md`
