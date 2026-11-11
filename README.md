# social_nav

Social navigation experiments for a TurtleBot 4 in Gazebo. The robot drives around a simulation worlds with walking actors, using Nav2 for navigation. A LiDAR detector (DR-SPAAM) and a camera detector (YOLOv8) find the people around it.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- TurtleBot 4 Simulator & Nav2
- YOLO and DR-SPAAM

`setup.sh` clones these two packages for you:

- [actor_pose_plugin](https://github.com/ahmetraufoktay/actor_pose_plugin): publishes the actor poses from Gazebo
- [2d_lidar_person_detection](https://github.com/TeamSOBITS/2d_lidar_person_detection) (`jazzy-devel`): DR-SPAAM

## Setup

```bash
mkdir -p ~/social_ws/src && cd ~/social_ws/src
git clone https://github.com/ahmetraufoktay/social_nav.git
cd social_nav

source /opt/ros/jazzy/setup.bash
./setup.sh
```

`setup.sh` does the one-time work:

1. Clones the missing source packages into `src/`
2. Installs the ROS dependencies with `rosdep` (it asks for sudo)
3. Creates `.venv` and installs the packages from `requirements.txt` (ultralytics, torch, ...)
4. Downloads the DR-SPAAM weights

It is safe to run again; finished steps are skipped. It doesn't build or launch anything.

Then build:

```bash
cd ~/social_ws
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
ros2 launch social_nav simulation.launch.py
```

You don't need to activate the venv. The launch file starts the YOLO and DR-SPAAM nodes with `.venv/bin/python3`, so they find torch and ultralytics on their own. The venv path is written into the install directory at build time. If you move or recreate the venv, rebuild.

Launch arguments:

| Argument          | Default            | Description                                                      |
|-------------------|--------------------|------------------------------------------------------------------|
| `mode`            | `lifelong`         | `amcl` (fixed map), `mapping` (SLAM from scratch), `lifelong` (slam_toolbox lifelong) |
| `human_detection` | `true`             | YOLO + DR-SPAAM + fusion node                                    |
| `nav2`            | `true`             | Start Nav2                                                       |
| `rviz`            | `true`             | Start RViz                                                       |
| `auto_undock`     | `true`             | Undock the robot 20 s after start                                |
| `world`           | `depot`            | Gazebo world in `worlds/`                                        |
| `map`             | `maps/depot_map.yaml` | Map for `amcl` mode                                           |
| `python`          | `.venv/bin/python3` | Interpreter for the torch-based nodes                           |

Example:

```bash
ros2 launch social_nav simulation.launch.py mode:=amcl human_detection:=false
```

## Nodes

| Node                | Language | Input                                           | Output                                   |
|---------------------|----------|-------------------------------------------------|------------------------------------------|
| `yolo_node`         | Python   | `/oakd/rgb/preview/image_raw`                   | `/human_detections`, `/human_detections/image` |
| `dr_spaam_ros`      | Python   | `/scan`                                         | `/dr_spaam_detections`                   |
| `human_detector`    | C++      | YOLO + DR-SPAAM detections, depth point cloud   | fused human positions                    |
| `proxemics_checker` | C++      | `/actors/poses`, `/sim_ground_truth_pose`       | `proxemics_violations.csv`               |

`proxemics_checker` is not started by the launch file. Run it separately:

```bash
ros2 run social_nav proxemics_checker
```

## Notes

- The TurtleBot 4 LiDAR covers 360°. Set these in `2d_lidar_person_detection/dr_spaam_ros/config/dr_spaam_param.yaml`:
  ```yaml
  panoramic_scan: true
  conf_thresh: 0.8
  ```
- To run a single node by hand with the venv:
  ```bash
  ros2 run --prefix ~/social_ws/src/social_nav/.venv/bin/python3 social_nav yolo_node
  ```
- ROS Jazzy's `cv_bridge` needs numpy 1.x, so `requirements.txt` pins `numpy<2`.

## Layout

```
config/    Nav2, slam_toolbox and RViz config
launch/    simulation.launch.py
maps/      depot map (pgm + slam_toolbox posegraph)
models/    Gazebo models (depot, actor)
worlds/    Gazebo worlds
weights/   YOLOv8n weights
src/       C++ nodes
social_nav/ Python nodes
```

## Authors

- [Ahmet Rauf OKTAY](https://github.com/ahmetraufoktay/)
- [Fatih Eren SÖZEN](https://github.com/fatiheren26)
