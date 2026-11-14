# Omnicopter SLAM simulation

Standalone ROS 1 Noetic / Gazebo Classic project combining the Omnicopter
v1.0 flight stack with an RGB-D camera and RTAB-Map. Everything needed to build
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

The demonstration unpauses physics, arms, takes off to 1.2 m, moves to (0,-1)
and (1,-1), turns to observe the surroundings, returns to (0,0), lands,
disarms, and pauses physics. The path is predefined; it does not use obstacle
avoidance or autonomous exploration. Simulation time may run slower than
wall time. The current world leaves this short path clear.

Run the demo from a freshly started simulation. To restart before another run:

```bash
docker compose restart simulator
```

If port 8080 is occupied, select another host port:

```bash
WEB_PORT=8081 docker compose up -d
```

Then open `http://localhost:8081/`. The application display is bound to the
local machine. No host X server, host ROS installation, or physical drone is
needed. Linux images are selected for the host architecture; native ARM64
operation is verified on Apple Silicon. Other hosts need their own run check.
The standalone image was built from this repository and its C++ demonstration
completed through landing and disarm. The pinned base image and GUI package
versions are available for AMD64, but AMD64 execution was not performed here.

## Control manually

Instead of running the demonstration, open the container terminal:

```bash
docker compose exec simulator bash
```

ROS is already sourced there. Unpause, arm, and take off:

```bash
rosservice call /gazebo/unpause_physics
rosservice call /ArmDisarm
rosservice call /lift_off '{height: 1.2, duration: 8.0}'
```

Wait for each maneuver to finish in simulation time before sending the next:

```bash
rosservice call /go_to '{x: 0.0, y: -1.0, z: 1.2, yaw: 0.0, duration: 8.0}'
rosservice call /rotate_to '{roll: 0.0, pitch: 0.0, yaw: 0.5236, duration: 7.0}'
rosservice call /land '{height_1: 0.5, duration_1: 4.0, height_2: 0.2, duration_2: 4.0}'
```

After landing, call `/ArmDisarm` again to disarm. This service toggles state;
it is not an idempotent "arm" command. Successful maneuver responses indicate
that a trajectory was accepted, not that the physical motion has completed.
Commands use the simulator world axes and metres/radians.

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

The original maneuver, controller, firmware and simulator plugin C++ files
are copied from v1.0. Unused Python joystick/calculation helpers and the old
UAV launch files referring to them are omitted. The supplied runtime uses
`omnicopter_slam/launch/simulate.launch` as its entrypoint.


## Assets and configuration

The checked-in camera uses 320×240 images at a requested 10 Hz, a 15 m far
clip, and a fixed mounting at (0.65, 0, 0.20) in body coordinates. Its settings
are in `src/AeRoSim/uav_simulator/models/omnicopter/model.sdf`. RTAB-Map's
occupancy-grid range remains 8 m. The sensor range is a simulation setting,
not a specification for a physical camera.

The world reuses the lab's later walls and objects. The central cube is moved
4 m along X to leave the v1.0 starting location clear. Missing lab textures
are replaced by a checked-in deterministic patterned texture. No asset
generator is needed at build time or runtime.

`simulate.launch` overrides the controller mass to 2.87 kg, matching the
bundled v1.0 body, IMU link and rotors. The upstream default was 4.1 kg.
Flight C++ and the original service/message names are retained.

The map database is `/home/omni/.ros/omnicopter.db` in the Compose `maps`
volume. Stopping or recreating the container preserves it. There is no map
deletion on startup. Software rendering is enabled for portability.
