# TurtleBot 4 Autonomous Navigation in Simulation (SLAM, AMCL, Nav2)

A simulated TurtleBot 4 maps an unknown Gazebo warehouse with SLAM, saves the map, localizes on it with AMCL, and drives itself to user-selected goals with the Nav2 stack. The project also measures how the costmap inflation radius trades clearance against travel time, and documents the system-level failures found while bringing up the stack in Docker.

![Robot localized on the saved map in RViz](figures/02_rviz_amcl_goal1.jpg)

## Results at a glance

- Mapped the Gazebo warehouse with `slam_toolbox` and saved it as an occupancy grid (0.05 m/cell); the map is included in [`maps/`](maps/).
- Localized on the saved map with AMCL and reached every goal with **no collisions and zero recovery behaviours**.
- Final position error across measured goals: **0.07 m to 0.35 m**, on routes of 6 to 8 m around shelving.
- Tested three costmap inflation radii (0.20, 0.45, 0.90 m) and quantified the clearance vs. speed trade-off.
- Diagnosed and fixed 8 bring-up failures (GPU fallback, DDS shared memory, lifecycle ordering, message type mismatch and more).

## Tech stack

| Component | Version / detail |
|---|---|
| Middleware | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (warehouse world) |
| Robot | TurtleBot 4 (standard) |
| Mapping | `slam_toolbox` |
| Localization | AMCL (particle filter) via `nav2_amcl` |
| Navigation | Nav2 (planner, controller, behaviour tree, costmaps) |
| Visualization | RViz 2 |
| Container | Docker image from [`Dockerfile`](Dockerfile) (`ros:jazzy` + desktop-full, TurtleBot 4 simulator, Nav2, slam_toolbox), launched with `rocker` 0.3.0 |
| Host | Ubuntu 24.04 (bare metal), NVIDIA RTX 4060 Laptop GPU, 15 GiB RAM |

## Pipeline

```
 Teleop + LiDAR ──> slam_toolbox ──> occupancy grid ──> map_saver (PGM + YAML)
                                                              │
                                                              v
 LiDAR + odometry ──> AMCL (localize on saved map) ──> map -> odom transform
                                                              │
                                                              v
 RViz Nav2 Goal ──> BT navigator ──> global planner (global costmap)
                                       └─> controller (local costmap) ──> smoother ──> collision monitor ──> /cmd_vel
```

1. **Map**: launch with SLAM, drive the robot by keyboard teleop, save the occupancy grid.
2. **Localize**: relaunch in localization mode with the saved map, set the initial pose with *2D Pose Estimate*; the AMCL particle cloud converges.
3. **Navigate**: send goals with *Nav2 Goal*; the global planner routes across the global costmap and the controller tracks it against the live local costmap.

## Saved map

The occupancy grid produced by SLAM and saved with `map_saver_cli` (`maps/warehouse_map.pgm` + `maps/warehouse_map.yaml`). White is free space, black is occupied, grey is unknown.

| Property | Value |
|---|---|
| Resolution | 0.05 m/cell |
| Grid size | 601 x 1009 cells |
| Origin | (-15.000, -25.323, 0) |
| Mode | trinary (occupied ≥ 0.65, free ≤ 0.196) |

<img src="figures/00_saved_map.png" alt="Saved warehouse occupancy grid" width="320">

## Navigation results

Goals sent through the Nav2 `NavigateToPose` action (the same action the RViz button uses):

| Goal | Target (m) | Final pose (m) | Position error | Result |
|---|---|---|---|---|
| 1 | (5.00, 0.00) | (4.65, −0.02) | 0.35 m | Succeeded |
| 2 | (0.00, 4.00) | (−0.18, 4.05) | 0.18 m | Succeeded |
| 3 | (−4.00, −3.00) | (−4.05, −3.05) | 0.07 m | Succeeded |

| RViz: second goal | Gazebo: third goal |
|---|---|
| ![Second goal in RViz](figures/04_rviz_goal2.jpg) | ![Third goal in Gazebo](figures/05_gazebo_goal3.jpg) |

## Inflation radius experiment

The inflation radius was changed live on both costmaps and the same route was driven for each value:

```bash
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.90
ros2 param set /local_costmap/local_costmap   inflation_layer.inflation_radius 0.90
```

Clearance = smallest LiDAR return recorded over the run.

| Inflation radius | Result | Closest approach | Traverse time | Costmap appearance |
|---|---|---|---|---|
| 0.90 m | Succeeded | 1.01 m | ~65 s | Thick halos that merge across narrow gaps |
| 0.45 m (default) | Succeeded | 0.63 m | ~40 s | Moderate bands along walls |
| 0.20 m | Succeeded | 0.67 m | ~30 s | Thin outlines, most floor open |

- A wide buffer bought about 1 m of clearance but took ~60% longer than the default and can close off aisles the robot physically fits through.
- A narrow buffer was fastest but leaves little margin for map error, localization drift or a person stepping into the aisle.
- Caveat: clearance at 0.20 m and 0.45 m is nearly identical because the warehouse corridors are wide enough for both to run down the middle. The difference would show in tighter spaces; the costmap images are the clearer evidence.

![Costmap inflation layer in RViz](figures/07_rviz_costmap_inflation.jpg)

## Problems found and fixed

None of these are covered in the course instructions; each was diagnosed from node logs, lifecycle states and topic types.

| Symptom | Root cause | Fix |
|---|---|---|
| Simulation ran at 0.17x real time | Container user not in `video`/`render` groups, so rendering silently fell back to CPU | Added groups, then moved to the NVIDIA path below |
| GPU not usable at all | NVIDIA Container Toolkit not installed; Docker had no GPU runtime | Installed toolkit, configured Docker runtime, relaunched `rocker --nvidia`; speed rose to ~1.0x real time |
| Robot ignored all drive commands | E-stop engaged from the HMI panel, so the motion controller forwarded zeros | Released via the `/e_stop` service |
| Keyboard teleop did nothing | Jazzy robot subscribes to `TwistStamped`; teleop publishes `Twist` by default | Ran teleop with `stamped:=true` |
| Nav2 bring-up aborted | Docker's default 64 MB `/dev/shm` exhausted; DDS transport could not open ports, lifecycle requests never arrived | Relaunched container with 2 GB shared memory |
| Planner timed out while activating | Global costmap needs the `map -> odom` transform, which AMCL only publishes after an initial pose; autostart activated Nav2 first | Set 2D Pose Estimate before activating navigation nodes |
| Gazebo crash took down the sim | Fault in the GUI render thread on window close | Minimize the Gazebo window instead of closing it |
| Goals accepted but robot never moved | Gazebo world paused, so the sim clock stopped | Unpaused and checked the simulation statistics topic |

## How to run

Prerequisites: Ubuntu 24.04, Docker, NVIDIA Container Toolkit (for GPU), [`rocker`](https://github.com/osrf/rocker).

```bash
# 1. Clone and build the image
git clone https://github.com/adhikariganesh9813/turtlebot4-slam-nav2-gazebo.git
cd turtlebot4-slam-nav2-gazebo
docker build -t tb4_jazzy .

# 2. Start the container with GPU, X11, your user/home directory and 2 GB shared memory
#    (Docker's 64 MB default is too small for the ROS 2 DDS transport; Nav2 bring-up fails without it)
rocker --nvidia --x11 --user --home --shm-size 2g tb4_jazzy
```

Inside the container, `cd` into the cloned repo (your home directory is mounted) and run `source /opt/ros/jazzy/setup.bash` in each new terminal.

**Option A: use the included map (skip mapping)**

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  slam:=false localization:=true nav2:=true rviz:=true \
  map:=$PWD/maps/warehouse_map.yaml
```

**Option B: build your own map**

```bash
# Gazebo + SLAM + Nav2 + RViz (press Play in Gazebo)
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true

# Release the robot from the dock (it ignores velocity commands while docked)
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

# Drive around to build the map (Jazzy expects stamped velocity messages)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Save the map
ros2 run nav2_map_server map_saver_cli -f maps/warehouse_map
```

Then relaunch with Option A.

**Navigate**: in RViz, set **2D Pose Estimate** first (AMCL needs an initial pose before the global costmap can activate), then click **Nav2 Goal**.

If the robot ignores commands, check the E-stop:

```bash
ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: false}"
```

## How the closed loop works

- **Sense**: LiDAR publishes scans at ~10 Hz; wheel encoders publish odometry, which drifts without bound on its own.
- **Localize**: AMCL scores each particle by how well the current scan matches the stored map at that pose, resamples, and publishes the `map -> odom` correction. Odometry gives smooth short-term motion; the laser gives long-term truth.
- **Plan**: the behaviour tree asks the global planner for a route over the global costmap (strategic, map-only).
- **Control**: the controller samples short candidate trajectories against the live local costmap, scores obstacle cost and progress along the route, and sends the best velocity command through the smoother and collision monitor.
- **Repeat**: motion produces new odometry and scans, AMCL refines the pose, the local costmap is rebuilt, and the route is replanned if the world no longer matches the plan.

## Repository structure

```
.
├── Dockerfile                 # ROS 2 Jazzy + TurtleBot 4 simulator + Nav2 + slam_toolbox
├── README.md
├── maps/
│   ├── warehouse_map.pgm      # occupancy grid saved from SLAM
│   └── warehouse_map.yaml     # resolution, origin, thresholds
├── docs/
│   └── Lab1_Report.pdf        # full lab report
└── figures/                   # map preview and Gazebo/RViz screenshots
```

## References

- [TurtleBot 4 User Manual: Simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- [TurtleBot 4 User Manual: Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
- [Nav2 documentation](https://docs.nav2.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## Author

**Ganesh Adhikari**, M.S. Computer Science, Texas Tech University
[LinkedIn](https://linkedin.com/in/adganesh) · [GitHub](https://github.com/adhikariganesh9813)

