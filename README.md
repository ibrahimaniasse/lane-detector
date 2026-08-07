# Lane Detector — Ramp World

ROS 2 Jazzy + Gazebo Harmonic lane-following package for a **TurtleBot3 Burger**
on a custom track featuring a ramp, sharp turns, and a physical support beam
that occludes the camera at one point.

The robot follows **yellow** (left) and **white** (right) lane markings using an
OpenCV bird's-eye-view pipeline inspired by the
[ROBOTIS AutoRace](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving)
approach: perspective warp → HSV color masking → sliding-window polynomial fit →
PD steering. A separate blind-maneuver node handles the beam-occluded section
by driving open-loop based on odometry position.

> **Forked from** [`laxmiprasad-vijaykumar/lane-detector`](https://github.com/laxmiprasad-vijaykumar/lane-detector)
> (originally ROS 1 Noetic). Taken over as an independent portfolio project,
> fully ported to **ROS 2 Jazzy + Gazebo Harmonic**.

---

## Architecture

```
                  /camera/image_raw
                        │
                        ▼
                 ┌──────────────┐
                 │ detect_lane  │   Bird's-eye warp, yellow/white masks,
                 │   .py        │   sliding-window fit, reliability scoring
                 └──────┬───────┘
                        │  /detect/lane_center (Float64)
                        ▼
                 ┌──────────────┐
                 │ control_lane │   PD controller → /cmd_vel
                 │   .py        │   Goes silent when blind_maneuver/active=True
                 └──────┬───────┘
                        │
        ┌───────────────┼───────────────┐
        │ /control/blind_maneuver/active│
        ▼               │               ▼
 ┌──────────────┐       │        ┌──────────────┐
 │ control_blind│ ──────┘        │   /cmd_vel   │
 │   .py        │                │  (to robot)  │
 └──────────────┘                └──────────────┘
   Subscribes to /odom.
   Triggers at a known (x,y) zone.
   Owns /cmd_vel during the maneuver.
```

| Node | Subscribes | Publishes |
|------|-----------|-----------|
| `detect_lane.py` | `/camera/image_raw` | `/detect/lane_center`, `/detect/image_lane`, `/detect/image_warped`, `/detect/yellow_line_reliability`, `/detect/white_line_reliability` |
| `control_lane.py` | `/detect/lane_center`, `/control/blind_maneuver/active` | `/cmd_vel` |
| `control_blind.py` | `/odom` | `/control/blind_maneuver/active`, `/cmd_vel` |

---

## Prerequisites

| Dependency | Version |
|-----------|----------|
| Ubuntu | 24.04 (Noble) |
| ROS | 2 Jazzy |
| Gazebo | Harmonic (gz-sim) |
| Python | 3.10+ |
| OpenCV | via `cv_bridge` |

```bash
# Install required ROS 2 packages
sudo apt install -y \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-cv-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Build & Run

```bash
# 1. Clone into your colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ibrahimaniasse/lane-detector.git

# 2. Build
cd ~/ros2_ws
colcon build --packages-select lane_detector

# 3. Source
source install/setup.bash

# 4. Launch everything (Gazebo Harmonic + robot + detection + control)
ros2 launch lane_detector ramp_world.launch.py
```

One command brings up the full stack. The robot starts driving and
following the lane markings autonomously after a ~5 second Gazebo startup delay.

---

## Key Parameters

All parameters are configurable via ROS params (set in the launch file or
at the command line with `_param:=value`).

### detect_lane.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~image_topic` | `/camera/image_raw` | Camera image topic |
| `~lookahead_frac` | `0.9` | Where to evaluate the lane fit (0=top, 1=bottom) |
| `~reliability_threshold` | `50` | Min reliability score (0–100) to trust a lane |
| `~auto_adjust_lightness` | `true` | ROBOTIS-style adaptive brightness floor |
| `~num_windows` | `20` | Sliding-window search bands |

### control_lane.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~kp` | `1.0` | Proportional gain |
| `~kd` | `0.5` | Derivative gain |
| `~linear_speed` | `0.2` | Base forward speed (m/s) |
| `~speed_falloff_exponent` | `2.2` | Power-law corner slowdown (ROBOTIS-style) |
| `~lost_lane_timeout` | `4.0` | Safety stop if no lane for this many seconds |

### control_blind.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~maneuver_trigger_x` | `1.02` | Trigger zone center X (m) |
| `~maneuver_trigger_y` | `2.75` | Trigger zone center Y (m) |
| `~maneuver_trigger_radius` | `0.3` | Trigger zone radius (m) |
| `~maneuver_angular_z` | `0.6` | Angular velocity during maneuver (rad/s) |
| `~maneuver_linear_x` | `0.06` | Linear velocity during maneuver (m/s) |
| `~maneuver_duration` | `2.0` | Maneuver duration (s) |

---

## Known Limitations

- **Gazebo Classic EOL**: This project uses Gazebo Classic 11.x, which is
  end-of-life.  It works fine with ROS Noetic but won't be ported to
  Gazebo Sim (Harmonic+) unless the project migrates to ROS 2.

- **Ramp collision geometry**: The collision boxes in `model.sdf` are
  first-pass approximations of the visual mesh.  If the robot clips through
  or hovers on the ramp, adjust the box dimensions and poses in
  `models/ramp_model/model.sdf`.

- **Blind maneuver is position-hardcoded**: The trigger zone and drive
  command are tuned for the specific track layout in `ramp_world.world`.
  A different track requires re-tuning these parameters.

---

## Credits

- **Original author**: [Laxmi Prasad Vijay Kumar](https://github.com/laxmiprasad-vijaykumar)
  — initial lane-detection and control pipeline.
- **Fork maintainer**: [Ibrahim Aniasse](https://github.com/ibrahimaniasse)
  — blind-maneuver node, packaging, ramp physics fix, documentation.

## License

[MIT](LICENSE)
