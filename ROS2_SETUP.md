# ROS 2 setup and reproducibility

This guide documents how to build the ROS 2 Python packages in this repository. It is a setup guide, **not a claim that the full stack has been reproduced or validated on a UAV**.

## Scope and prerequisites

- Ubuntu with a ROS 2 distribution compatible with the packages installed on your target computer. The original deployment distribution/version is not pinned in this repository; record the exact ROS 2 distribution and Ubuntu version used for any reproduction.
- ROS 2 environment sourced in the terminal.
- `colcon` and `rosdep` installed.
- Hardware-specific ROS drivers and dependencies available for the chosen ROS 2 distribution.

The `drone/` directory contains four ROS 2 Python packages: `drone_pkg`, `navigator_pkg`, `obstacle_avoidance`, and `uav_bringup`. The launch file also starts external packages including MAVROS, the YDLIDAR ROS 2 driver, and rosbridge.

## 1. Install workspace tools

Follow the official ROS 2 installation instructions for your Ubuntu/ROS 2 combination. In a ROS 2-sourced terminal, install the workspace tools if they are not already installed:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Initialize rosdep only if it has not already been initialized on this machine:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it has already been initialized, continue with `rosdep update`.

## 2. Resolve dependencies and build

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd drone
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Set `ROS_DISTRO` to the distribution actually installed. These commands are the intended workspace workflow; a successful build on your machine should be recorded with the OS, ROS 2 distribution, dependency versions, and build output. The repository's current GitHub Actions workflow checks Python syntax and unit tests only; it does not perform this ROS 2 build.

To confirm that the workspace packages are discoverable after a successful build:

```bash
ros2 pkg list | grep -E '^(drone_pkg|navigator_pkg|obstacle_avoidance|uav_bringup)$'
```

## 3. Inspect the launch arguments without starting the stack

After building and sourcing the workspace, inspect the launch arguments:

```bash
ros2 launch uav_bringup uav_bringup.launch.py --show-args
```

The launch file's defaults are hardware-specific: Pixhawk serial connection `/dev/ttyAMA0:57600`, LiDAR port `/dev/ttyUSB0`, optional GCS UDP passthrough `udp://@`, and rosbridge WebSocket port `9090`. Adjust these for the target machine only after checking the wiring and interface names.

## 4. Hardware launch — only after safety checks

**Do not use the full launch command as a hardware-free test.** The launch file starts MAVROS, the LiDAR driver, camera and environmental-sensor nodes, mission reception, navigation, obstacle avoidance, and rosbridge. It expects the relevant hardware and external ROS packages to be installed and configured.

Before attempting a hardware launch:

1. Keep propellers removed for initial bench checks and ensure the aircraft is physically secured.
2. Verify the Pixhawk serial device, baud rate, firmware, MAVROS configuration, and flight-mode behavior.
3. Verify the LiDAR device path and confirm that scan data is being published.
4. Check camera and environmental-sensor interfaces on the actual companion computer.
5. Keep a reliable manual/RC override and follow your lab's flight-safety procedure.
6. Do not expose the ground station or rosbridge ports to an untrusted network; see [SECURITY.md](SECURITY.md).

Only after those checks should a qualified operator decide whether to launch the complete stack. A successful process start does not prove safe navigation, obstacle avoidance, or flight readiness.

## 5. Current verification boundary

The repository does not currently include a documented ROS bag, a hardware-independent end-to-end test harness, or a pinned, clean ROS 2 deployment environment. Do not treat the following as verified solely from the Python CI result:

- Successful build on a fresh ROS 2 installation.
- End-to-end message flow between every node.
- Detection performance or real-time FPS on the Raspberry Pi.
- Flight-control behavior, obstacle avoidance safety, or GNSS-denied autonomy.

For a reproducible research result, record the exact hardware, OS/ROS versions, dependency versions, launch configuration, input data or rosbag, expected topic names, observed outputs, and any failure conditions. Publish confusion matrices, precision/recall, mAP, or FPS only when the model, dataset/split, and measurement procedure are available for independent review.
