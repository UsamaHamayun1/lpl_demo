# LPL Demo

A TurtleBot3 Waffle Pi drives down a corridor while the **LPL guard** scales its speed by a
*confidence* value that reacts to the obstacles around it, and hands authority to a human when it
can't resolve a situation on its own.

| Obstacle | LPL behaviour |
|---|---|
| 🟨 `obstacle_safe` (static) | Cautious: confidence drops to 70% within 2 m |
| 🟥 `obstacle_danger` (moving) | Pauses 3 s, then passes at 70% on a wide path |
| 🟦 `obstacle_blue` (movable) | Pauses 3 s, then hands control to the human (WASD); SPACE hands it back |
| Collision (< 0.45 m) | Hard stop and lockout |

---

## 🛠️ Prerequisites

* **Ubuntu 24.04 LTS**
* **ROS 2 Jazzy**
* **Gazebo Harmonic** (via `ros_gz`)
* **TurtleBot3** + **Nav2** Jazzy packages
* **Python**: PyQt5, OpenCV, NumPy

Everything above is installed by:

```bash
sudo bash scripts/install_deps.sh
rosdep update
```

> The project originally targeted Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic. Gazebo Classic is
> end-of-life and not available on 24.04, so it was ported; see [Migration notes](#-migration-notes).

---

## 🏗️ Build

```bash
mkdir -p ~/lpl_ws/src && cd ~/lpl_ws/src
git clone https://github.com/UsamaHamayun1/lpl_demo.git
cd ~/lpl_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lpl_demo
source install/setup.bash
```

Run `source ~/lpl_ws/install/setup.bash` in every new terminal (or add it to `~/.bashrc`).

---

## 🏃‍♂️ Run the demo

One terminal per command, each sourced:

```bash
# 1. Simulation: Gazebo Harmonic, robot, ROS bridges, world-state publisher
ros2 launch lpl_demo gauntlet.launch.py          # add gui:=false for headless

# 2. LPL guard (the confidence / authority logic)
ros2 run lpl_demo lpl_manager_guard

# 3. Red-box behaviour (STABLE / JITTER, switched from the dashboard)
ros2 run lpl_demo lpl_actordirector

# 4. Mission control dashboard
ros2 run lpl_demo lpl_dashboard

# 5. Autonomy: scripted corridor planner
ros2 run lpl_demo lpl_nav2_demo
```

**Dashboard keys:** `W A S D` drive (`X` reverse) while in manual override · `SPACE` hand authority
back · `U N H J` move the blue box, `K` stop it.

Once the blue box takes over, drive past it (or move it out of the way), press **SPACE**, and the
robot resumes autonomous driving.

### Other worlds

```bash
ros2 launch lpl_demo sim.launch.py world:=<name> [x_pose:=.. y_pose:=..] [gui:=false]
```

`<name>` is any file in `worlds/` without `.world`: `gauntlet2` (default), `gauntlet1`,
`actor_puppet`, `cafe_complex`, `dynamic_tests`, `three_boxes`, `l_maze`. The old launch files
(`actor_puppet.launch.py`, `world_only.launch.py`, …) still work and wrap `sim.launch.py`.

### Using real Nav2 instead of the scripted planner

```bash
ros2 launch lpl_demo lpl_nav2.launch.py map:=/path/to/map.yaml
```

Nav2's final output (collision monitor) is redirected to `/cmd_vel_auto` so it passes through the
guard. A map of the corridor is not included yet.

---

## 🔌 Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel_auto` | `geometry_msgs/TwistStamped` | planner / Nav2 → guard |
| `/cmd_vel_input` | `geometry_msgs/Twist` | dashboard teleop → guard |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | guard → robot |
| `/lpl/status` | `std_msgs/String` (JSON) | guard → dashboard |
| `/lpl/authorize` | `std_msgs/Empty` | dashboard → guard |
| `/lpl/activation` | `std_msgs/Bool` | dashboard → guard, planner |
| `/lpl/actor_mode` | `std_msgs/String` | dashboard → actor director |
| `/actor_cmd_vel`, `/blue_cmd_vel` | `geometry_msgs/Twist` | → red / blue box |
| `/lpl/model_states` | `tf2_msgs/TFMessage` | `gz_world_state` → nodes |
| `/camera/image_raw`, `/scan`, `/odom`, `/imu` | sensors | robot → ROS |

---

## 🔁 Migration notes

Humble + Gazebo Classic → Jazzy + Gazebo Harmonic:

* **`/gazebo/model_states` is gone.** `gz_world_state` reads `/world/<world>/pose/info` from
  gz-transport and publishes `/lpl/model_states` (`TFMessage`, `child_frame_id` = entity name).
  `lpl_demo.model_states.from_tf()` turns it back into the old `.name` / `.pose` lists.
* **`/cmd_vel` is `TwistStamped`** (TurtleBot3 and Nav2 on Jazzy). The guard publishes stamped
  commands; autonomy input moved from `/cmd_vel_nav` (used internally by Nav2 on Jazzy) to
  `/cmd_vel_auto`.
* **Moving boxes:** `libgazebo_ros_planar_move` → `gz-sim-velocity-control-system`, bridged from
  `/actor_cmd_vel` and `/blue_cmd_vel` (see `config/bridge.yaml`).
* **Worlds:** Gazebo Classic material scripts → explicit colours; `model://sun` and
  `model://ground_plane` inlined; Harmonic system plugins added. `cafe_complex` downloads the
  walking-actor mesh from Gazebo Fuel on first run.
* `lpl_manager`, `lpl_manager_formula`, `lpl_manager_gauntlet` and `manager_with_multi` are earlier
  iterations that still import `gazebo_msgs` and were not ported.
