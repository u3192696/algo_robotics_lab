# Succulence Rover — 12062 Algorithmic Robotics

Autonomous SLAM and navigation stack for the Succulence rover, built for **Operation Find Kevin**. The rover maps an unknown environment using a 2D lidar, localises itself using pose-graph SLAM, and navigates to a hardcoded goal coordinate via A\* path planning and a pure-pursuit controller. Supports both a Unity Mars simulation and a physical TurtleBot (iRobot Create 3 + RPLidar).

---

## System Architecture

```
Sensors (odom + scan)
        │
        ▼
   [ slam_node ]  ──── /succulence/map (nav_msgs/OccupancyGrid)
        │           └── /succulence/slam/odometry (nav_msgs/Odometry)
        │          └── /succulence/slam/path (nav_msgs/Path) → RViz
        ▼
 [ planner_node ]  ─── /succulence/plan (nav_msgs/Path)
        │
        ▼
[ navigator_node ] ─── /cmd_vel or /cmd_vel_unstamped (geometry_msgs/Twist)
        │
        ▼
     Wheels
```

| Node | Role | Key files |
|---|---|---|
| `slam_node` | Pose-graph SLAM: motion model, scan matching, Gauss-Newton optimisation, map rebuild | `slam_node.py`, `scan_matcher.py`, `pose_graph.py`, `graph_optimizer.py` |
| `planner_node` | A\* path planner over the SLAM occupancy grid, replans on a timer | `planner_node.py`, `astar.py` |
| `navigator_node` | Pure-pursuit path follower, 10–15 Hz control loop | `navigator_node.py`, `path_follower.py` |
| `motion_model_node` | Dead-reckoning odometry with covariance propagation (Week 5/6, not launched in mission stack) | `motion_model.py` |
| `occupancy_grid_mapper_node` | Bayesian log-odds occupancy grid from dead-reckoned poses (Week 5/6, not launched in mission stack) | `occupancy_grid_mapper.py` |

---

## Prerequisites

- **ROS 2 Humble** (or later)
- **Python 3.10+**
- Python dependencies: `numpy`, `scipy`

```bash
pip install numpy scipy
```

For the **simulation**: Unity Mars sim with the Succulence rover package configured and running.

For the **physical robot**: iRobot Create 3 base + RPLidar, with their respective ROS 2 drivers running and the robot visible on the network.

---

## Build

```bash
cd ~/your_ros2_ws
colcon build --packages-select succulence_rover_ros
source install/setup.bash
```

---

## Configuration

All parameters live in `config/`. Two files cover the two deployment targets:

| File | Target |
|---|---|
| `config/params_sim.yaml` | Unity Mars simulation |
| `config/params_physical.yaml` | Physical TurtleBot |

**Key parameters to set before launching:**

```yaml
# Goal coordinates (Kevin's location) — in both config files
planner_node:
  ros__parameters:
    goal:
      x: 3.18   # metres, map frame
      y: 0.90
```

Refer to the inline comments in each YAML file for a full description of every tunable parameter and its effect on system behaviour.

---

## Launch

### Simulation (Unity Mars)

Ensure the Unity sim is running and publishing `/succulence/odom` and `/succulence/scan`, then:

```bash
ros2 launch succulence_rover_ros mission_sim.launch.py
```

This starts:
- Two `tf2_ros` static transform publishers (`map → succulence/odom`, `succulence/base_link → succulence/lidar_link`)
- `slam_node`
- `planner_node`
- `navigator_node`

### Physical TurtleBot

Ensure the Create 3 and RPLidar drivers are running and publishing `/odom` and `/scan`, then:

```bash
ros2 launch succulence_rover_ros mission_physical.launch.py
```

This launch file:
1. Calls `/reset_pose` (iRobot Create 3 service) to zero the odometry — the autonomy stack waits until this completes.
2. Then starts `map_to_odom_publisher`, `slam_node`, `planner_node`, and `navigator_node`.

> **Note:** The physical robot publishes velocity commands on `/cmd_vel_unstamped` (not `/cmd_vel`). This is the Create 3 convention for unstamped Twist messages and is configured automatically via `params_physical.yaml`.

### Dead-Reckoning Only (Week 5/6 demo)

```bash
ros2 launch succulence_rover_ros dead_reckoning.launch.py
```

Starts `motion_model_node` and `occupancy_grid_mapper_node` only. The resulting map will show ghost walls from odometry drift — this is intentional and demonstrates why SLAM is needed.

---

## Visualisation (RViz 2)

Set the **Fixed Frame** to `map`, then add the following displays:

| # | Display type | Topic | Notes |
|---|---|---|---|
| 1 | Odometry | `/succulence/dead_reckoning/odometry` | Enable covariance — watch the ellipse grow |
| 2 | Path | `/succulence/dead_reckoning/path` | Dead-reckoning trajectory (red) |
| 3 | Map | `/succulence/map/odom_only` | Week 5/6 map — ghost walls visible |
| 4 | Path | `/succulence/slam/path` | SLAM-corrected trajectory (green) |
| 5 | Map | `/succulence/map` | SLAM-corrected map — walls aligned |
| 6 | Path | `/succulence/plan` | A\* planned path (yellow) |

---

## Verify Topics Are Flowing

```bash
ros2 topic list | grep succulence
ros2 topic hz /succulence/slam/odometry
ros2 topic hz /succulence/map
```

---

## Package Structure

```
succulence_rover_ros/
├── config/
│   ├── params_sim.yaml          # Simulation parameters
│   └── params_physical.yaml     # Physical TurtleBot parameters
├── launch/
│   ├── mission_sim.launch.py    # Full mission stack — sim
│   ├── mission_physical.launch.py  # Full mission stack — physical
│   └── dead_reckoning.launch.py # Week 5/6 demo only
└── succulence_rover_ros/
    ├── utils.py                 # SE(2) maths toolkit (provided)
    ├── motion_model.py          # Odometry + alpha noise model
    ├── occupancy_grid_mapper.py # Bayesian occupancy grid
    ├── scan_matcher.py          # Grid-correlation scan alignment
    ├── pose_graph.py            # Pose-graph data structure
    ├── graph_optimizer.py       # Gauss-Newton optimiser
    ├── slam_node.py             # Full SLAM pipeline node
    ├── astar.py                 # A* search (no ROS dependencies)
    ├── planner_node.py          # A* planner ROS 2 node
    ├── path_follower.py         # Pure-pursuit controller (no ROS)
    └── navigator_node.py        # Path follower ROS 2 node
```

---

## Troubleshooting

**No path found / planner keeps failing**
- Check that the goal coordinates are within the map bounds (`occupancy_grid.width/height` × `resolution`).
- Try reducing `planning.inflation_radius_cells` if narrow corridors are being inflated shut.
- Set `planning.treat_unknown_as_obstacle: false` (default) so the rover plans optimistically into unmapped space.

**Robot not moving / navigator stopped**
- Check `/succulence/plan` is being published: `ros2 topic hz /succulence/plan`
- The navigator has a plan staleness timeout (`control.plan_timeout`). If the planner is slow, increase this value.
- Verify SLAM odometry is flowing: `ros2 topic hz /succulence/slam/odometry`

**Map has ghost walls / drift visible**
- This is expected before SLAM has had time to optimise. Watch `/succulence/slam/path` snap into alignment after `slam.optimization_interval` keyframes.
- Reduce `slam.keyframe_distance` and `slam.keyframe_angle` to add more constraints faster.

**Physical robot does not move after launch**
- Confirm the `/reset_pose` service call completed successfully (check terminal output from the launch).
- Confirm `/cmd_vel_unstamped` is being published: `ros2 topic hz /cmd_vel_unstamped`

---

## Course Context

Built as part of **12062 Algorithmic Robotics** at the University of Canberra. The system implements the full sense–think–act loop covered across Weeks 5–12: motion model (Weeks 5/6), occupancy grid mapping (Week 6), scan matching (Week 7), pose-graph SLAM with Gauss-Newton optimisation (Week 11), and A\* planning with pure-pursuit navigation (Week 12).
