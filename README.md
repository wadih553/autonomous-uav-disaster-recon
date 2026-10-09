# Autonomous UAV System for Rapid Disaster Reconnaissance

**Final-year project · La Sagesse University · 2025 · Grade: A+**  
**Technical lead:** Wadih Dahrouge · **Team:** Amine Batrouni, Houssam Hamdan  
**Supervisor:** Dr. Eng. Roy Abi Zeid Daou

A self-funded UAV prototype for disaster reconnaissance, combining ArduPilot/Pixhawk flight control, ROS 2 components, LiDAR-based obstacle sensing, onboard sensors, computer-vision experiments, and a web-based ground station.

The project was presented to the **Lebanese Civil Defence**. Its representative described the system as *“very interesting and indispensable”* for wildfire response. The prototype was developed under limited funding and constrained outdoor-testing conditions.

> **Research/engineering portfolio note:** This repository contains selected project code, system diagrams, test imagery, and documentation. It is **not currently a turnkey, fully reproducible deployment**; review the implementation status and limitations below before attempting to run it.

---

## Project motivation

Wildfires and other emergencies can put responders at risk and make rapid situational awareness difficult. This project explored how a UAV could support reconnaissance by combining waypoint missions, obstacle sensing, video streaming, and fire/smoke and human-detection experiments.

GNSS reliability and restricted opportunities for outdoor testing were important project constraints. The work used a mix of bench checks, mission simulation, and constrained real-world flight tests. **Full 3D SLAM and fully validated GNSS-denied autonomy were not completed in this iteration.**

## Reported prototype results

| Metric | Reported result |
|---|---:|
| Human-detection accuracy (YOLOv8) | 92% |
| Fire-detection accuracy | 80% |
| Flight endurance | 18 minutes |
| Assembled mass | 1.4 kg |
| GPS positioning accuracy during field testing | ±2–3 m |
| Obstacle-distance measurement error against ground truth | ±2 cm |
| Environmental-sensor figures | Humidity ±0.5%; temperature ±2–3%; air quality ±4.5 AQI |

These are **project-reported results**, not independently reproduced benchmarks. The repository does not currently provide a complete evaluation dataset and benchmark protocol for reproducing every figure.

### Detection example

<p align="center">
  <img src="detection_results.png" width="700" alt="Example fire/smoke and human-detection output">
</p>

## System architecture

<p align="center">
  <img src="architecture_block_diagram.png" width="650" alt="UAV system architecture block diagram">
</p>

- **Flight controller:** Pixhawk 2.4.8 running ArduCopter/ArduPilot firmware.
- **Companion computer and robotics middleware:** Raspberry Pi 4B and ROS 2 components.
- **Flight-control interface:** MAVLink/MAVROS.
- **Obstacle sensing:** YDLIDAR X4 Pro, with a front-facing camera for video.
- **Perception experiments:** YOLOv8-based human detection and a pretrained CNN for fire/smoke detection. The ground-station code contains the detection pipeline; deployment and performance depend on the runtime environment.
- **Ground station:** Flask, Flask-SocketIO, a web interface using Leaflet, and ROSBridge communication.
- **Additional sensing:** GPS/compass, landing-distance sensing, MQ-135 air-quality sensor, and SHT3-X temperature/humidity sensor.

<p align="center">
  <img src="wiring_schematic.png" width="650" alt="UAV wiring schematic">
</p>

## Flight testing

<p align="center">
  <img src="uav_flight_2m_obstacle_test.jpg" width="600" alt="UAV during an obstacle-avoidance flight test">
</p>

The reported test sequence included:
- Bench checks and IMU/ESC calibration.
- Mission simulation using QGroundControl/Mission Planner.
- Safety-constrained tests, including no-propeller mission validation.
- Constrained outdoor flights involving takeoff, waypoint navigation, obstacle-avoidance behaviour, and return-to-launch.

## Quick start (ground station)

The project is hardware- and ROS-version-dependent. Start with the ground station in isolation; do not connect flight-control hardware until the configuration and safety checks have been reviewed.

```bash
cd ground_station/server
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell:
# .venv\\Scripts\\Activate.ps1
python -m pip install -r requirements.txt
cp .env.example .env
```

The sample environment file is documentation for configuration; the app does not automatically load `.env`. Export the variables in your shell or use a trusted environment manager. Set a unique `UAV_SECRET_KEY`, configure the Raspberry Pi and ORS key if needed, and review [SECURITY.md](SECURITY.md) before running.

Run the current unit tests from the repository root:

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q
python -m compileall -q ground_station/server drone
```

The GitHub Actions workflow runs Python syntax checks and mission-planner unit tests. It does **not** build the ROS 2 workspace or verify real UAV operation.

## Repository structure

```text
.
├── drone/                  # ROS 2 package scaffolding and UAV-side components
├── ground_station/
│   ├── server/              # Flask API, mission planning, ROSBridge and detection modules
│   └── ui/                  # Web interface assets and templates
├── architecture_block_diagram.png
├── detection_results.png
├── wiring_schematic.png
├── uav_flight_2m_obstacle_test.jpg
├── Autonomous UAV system final PDF.pdf
└── Wadih Dahrouge UAV Technical Summary v2.docx
```

## Implementation status and limitations

- The repository includes ground-station Python modules and ROS 2 package scaffolding, but **the presence of source files does not mean the complete stack has been verified from a clean installation**.
- Full 3D SLAM, thermal-camera integration, and multi-UAV coordination remain future work.
- The Raspberry Pi's compute capacity and the prototype's approximately 18-minute endurance constrained operation.
- Detection metrics and flight-test claims should be interpreted in the context of the original project report; a reproducible benchmark suite is not included here.
- **Security:** the ground station exposes mission-control routes and should only be run in a trusted, isolated test network until authentication, restrictive CORS settings, safe secret management, and SSH host-key verification have been configured. Do not expose it directly to the public internet.

## Development checks

- Mission inputs are checked for finite numeric values, coordinate ranges, and waypoint-count limits on the ground-station side.
- Incoming ROS 2 mission payloads are independently validated before being written to disk.
- SSH fallback rejects unknown host keys instead of trusting them automatically.
- The ground-station HTTP server binds to localhost by default. Network exposure must be deliberate and restricted to a trusted, firewalled environment.
- RTL/LAND API responses indicate whether a request was sent; they do not confirm that the aircraft accepted the mode change. Always verify the vehicle state and retain manual control.

## Documentation

- [Final-year project report (PDF)](Autonomous%20UAV%20system%20final%20PDF.pdf)
- [UAV technical summary (DOCX)](Wadih%20Dahrouge%20UAV%20Technical%20Summary%20v2.docx)
- [Browse the ground-station source](ground_station/server/)
- [Browse the UAV-side code](drone/)

## Author

**Wadih Dahrouge** · Mechatronics Engineer  
Focus areas: UAV robotics, autonomous systems, ROS 2, perception, and navigation.

[Email](mailto:wadihdahrouge1@gmail.com) · [GitHub profile](https://github.com/wadih553)
