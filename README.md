# Omnicopter SLAM simulation

Standalone ROS 1 Noetic / Gazebo Classic project combining the Omnicopter
v1.0 flight stack with an RGB-D camera, an IMU, and RTAB-Map. Everything needed to build
the application is inside this repository. No other branch clone, existing
workspace, or prebuilt Omnicopter image is required.

## Run with Docker

From the project root:

```bash
docker compose up -d --build
```

Open [http://localhost:8080/](http://localhost:8080/) in a browser. Gazebo,
RTAB-Map's viewer, and a terminal run inside the container; Xpra displays their
windows in the browser. Use the toolbar's **Open Windows** menu to select a
window, or drag its title bar. Open a fresh browser tab if restarting the
container disconnects an existing browser session.

Gazebo starts paused. Start the optional C++ demonstration from your host:

```bash
docker compose exec simulator omni-ros rosrun omnicopter_slam flight_demo
```

The demonstration unpauses physics, waits for the fused pose, arms, takes off to 1.2 m, moves to (0,-1)
and (1,-1), turns to observe the surroundings, returns to (0,0), lands,
disarms, and pauses physics. The path is predefined; it does not use obstacle
avoidance or autonomous exploration. Simulation time may run slower than
wall time. The current world leaves this short path clear.

Run the demo from a freshly started simulation. To restart before another run:

```bash
docker compose restart simulator
```

## Six-degree-of-freedom stress test

Start from a fresh simulation and run the alternate route:

```bash
docker compose exec simulator omni-ros rosrun omnicopter_slam flight_demo --six-dof
```

Six pose targets combine translation in X, Y, and Z with roll, pitch, and
yaw. The route repeatedly reverses its attitude while moving, with airborne
waypoints between 0.90 and 2.05 m and yaw targets up to ±54°. It returns to
a level hover at the origin before landing and disarming.

### Visual odometry dropout test

Start a fresh simulator with the test-only reference stream enabled, then run:

```bash
OMNI_GROUND_TRUTH=true docker compose up -d --force-recreate
docker compose exec simulator omni-ros rosrun omnicopter_slam pose_dropout.py
```

The test pauses RTAB-Map visual odometry for 2 seconds during the third
six-axis leg. To change the outage duration, pass `_dropout_seconds:=0.45`
or another positive number. During the interruption, the test checks
for a fused-output gap of at most 0.10 s and pose errors no greater than
0.75 m and 20° against the aligned Gazebo ground truth. It also requires
visual odometry to resume within 1 second and the flight to finish and
disarm.

If port 8080 is occupied, select another host port:

```bash
WEB_PORT=8081 docker compose up -d
```

Then open `http://localhost:8081/`. The application display is bound to the
local machine. No host X server, host ROS installation, or physical drone is
needed. Linux images are selected for the host architecture. The original
standalone image and demonstration were exercised on Apple Silicon.

## Control manually

Instead of running the demonstration, open the container terminal:

```bash
docker compose exec simulator bash
```

ROS is already sourced there. Unpause, wait for a pose, arm, and take off:

```bash
rosservice call /gazebo/unpause_physics
rostopic echo -n 1 /pose_full
rosservice call /ArmDisarm
rosservice call /lift_off '{height: 1.2, duration: 8.0}'
```

Wait for each maneuver to finish in simulation time before sending the next:

```bash
rosservice call /go_to '{x: 0.0, y: -1.0, z: 1.2, yaw: 0.0, duration: 8.0}'
rosservice call /rotate_to '{roll: 0.0, pitch: 0.0, yaw: 0.5236, duration: 7.0}'
rosservice call /land '{height_1: 0.5, duration_1: 4.0, height_2: 0.05, duration_2: 4.0}'
```

After landing, call `/ArmDisarm` again to disarm. This service toggles state;
it is not an idempotent "arm" command. Successful maneuver responses indicate
that a trajectory was accepted, not that the physical motion has completed.
Commands use the local visual odometry frame, in metres and radians. Its
origin and heading come from RTAB-Map's initial camera pose.

## Pose estimation

The Gazebo IMU is rigidly aligned with the body and supplies gyro and
accelerometer measurements on `/omnicopter/imu/data_raw`. A Madgwick filter
estimates attitude from these samples. RTAB-Map uses the change in IMU
attitude to predict camera rotation between frames.

The pose filter combines visual pose and linear velocity with the IMU rates
and acceleration. It removes gravity using the IMU attitude at the
acceleration sample timestamp and skips camera updates flagged as lost.
After consecutive registration failures, RTAB-Map can reseed local
odometry from the filter's predicted pose.

The filter publishes `/omnicopter/odometry/filtered`. The simulator adapter
converts this estimate to `/pose_full` for the controller and maneuver planner.
The mapping launch disables the ground-truth odometry publisher by default.
Validation runs can enable `/omnicopter/ground_truth/odometry` independently
of the estimator, controller, and maneuver planner.
RTAB-Map builds the map from its camera odometry, so map loop closures do not
move the local frame used for flight commands. With `slam:=false`, the
visual odometry and pose filter still run while mapping and the viewer are disabled.

## Source origins

Upstream repository: [RISC-NYUAD/omnicopter](https://github.com/RISC-NYUAD/omnicopter).

- Flight packages, simulator plugin C++, and dependencies: branch `v1.0`.
- Mapping-world geometry and triangle-wall assets: branch `icuas2026`.
- The Dockerfile adapts the local `v1-simulation` setup into an independent
  build, installing RTAB-Map directly rather than extending a local image.

Integration additions:

- RGB-D sensor configuration and fixed camera transforms for the v1.0 model.
- Local patterned materials replacing unavailable `wall_materials` assets.
- Central cube offset, sunlight, and retained v1.0 physics settings.
- `omnicopter_slam` launch files and C++ demonstration service client.
- Docker Compose and four shell helpers for building and running the project.
- The controller mass override lives in the integration launch file.

The maneuver, controller, firmware and simulator plugin C++ files are based
on v1.0. Unused Python joystick/calculation helpers and the old
UAV launch files referring to them are omitted. The supplied runtime uses
`omnicopter_slam/launch/simulate.launch` as its entrypoint.


## Assets and configuration

The checked-in camera uses 320×240 images at a requested 10 Hz, a 60° horizontal
field of view, a 15 m far clip, and a fixed mounting at (0.65, 0, 0.20) in body
coordinates. Its settings
are in `src/AeRoSim/uav_simulator/models/omnicopter/model.sdf`. RTAB-Map's
occupancy-grid range remains 8 m. The sensor range is a simulation setting,
not a specification for a physical camera.

The world reuses the lab's later walls and objects. The central cube is moved
4 m along X to leave the v1.0 starting location clear. Missing lab textures
are replaced by checked-in patterns. Each wall panel has its own arrangement
of high-contrast features to help RTAB-Map distinguish revisits to different
parts of the room.

`simulate.launch` overrides the controller mass to 2.87 kg, matching the
bundled v1.0 body, IMU link and rotors. The upstream default was 4.1 kg.
The original flight service and message names are retained.

The map database is `/home/omni/.ros/omnicopter.db` in the Compose `maps`
volume. Stopping or recreating the container preserves it. There is no map
deletion on startup. Software rendering is enabled for portability.
