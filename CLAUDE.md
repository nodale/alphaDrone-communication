# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a drone communication and control system for a thesis project. It coordinates between a Vicon motion capture system, a drone running PX4/MAVLink, an obstacle avoidance controller (Hongi), and a visualization server.

## Running Services

The system runs as a set of `systemd` services coordinated via shared memory. To manage them:

```bash
# Restart all services
bash scripts/restart.sh

# Stop services
bash scripts/stop.sh

# Individual service control
sudo systemctl start|stop|restart <service_name>
```

Services: `vicon_data_collector`, `estimation_data_collector`, `setpoint_data_collector`, `triangulator`, `hongi_process`, `viser`, `nn_inferrer`, `nn_online_learner`

## Running Individual Scripts (Development)

All Python scripts in `alpha/` are run directly:

```bash
cd alpha
python vicon_collector.py         # Receives Vicon pose data via UDP
python estimator_collector.py     # Reads MAVLink odometry from /dev/ttyTHS1
python active_com.py              # Main loop: logging, MAVLink commands, keyboard control
python hongi_process.py           # Neural network controller inference
python triangulator.py            # Obstacle distance computation
python baby_sitter.py             # Viser 3D visualization
python joeystick_sender.py        # Joystick input sender (run on remote PC)
python vicon_sender.py            # Vicon data sender (run on Vicon PC)
python -m nn.inferrer             # NN inferrer: SHM → model → zarr log (toggle: i)
python -m nn.online_learner       # NN online learner: generational training (toggle: u)
```

## Running Tests

```bash
cd alpha
python tests/hongi_controller_test.py    # Test Hongi controller inference
python tests/triangulator_test.py        # Benchmark triangle distance computation
python tests/clock_test.py
python tests/custom_mavlink_msg_test.py
```

## Architecture

### Inter-Process Communication via Shared Memory

All processes exchange data through named POSIX shared memory segments (Python `multiprocessing.shared_memory`). Key segments:

| Name | Shape/Type | Owner (creates) | Readers |
|------|-----------|-----------------|---------|
| `vicon_state` | `(4, 6) float64` — pose (x,y,z,roll,pitch,yaw) for drone + 3 obstacles | `quick_vicon.py` | `active_com`, `triangulator`, `baby_sitter` |
| `vicon_init_state` | `(4, 6) float64` — initial pose reference | `quick_vicon.py` | `active_com` |
| `estimated_state` | `(13,) float64` — pos(3) + quat(4) + vel(3) + angvel(3) | `quick_mavlink.py` (create=True) | `hongi_process`, `baby_sitter` |
| `general_setpoint` | `(6,) float64` — (x,y,z,vx,vy,vz) | `quick_mavlink.py` | `hongi_process`, `active_com` |
| `actuation` | `(4,) float64` | `quick_mavlink.py` | `active_com` |
| `low_level_control` | `(4,) float64` | `hongi_process` | `active_com` |
| `obstacle_corners` | `(4, 4, 3) float64` | `quick_vicon.py` | `triangulator`, `active_com` |
| `dist` | `(1,) float64` — min obstacle distance | `triangulator` | `active_com` |
| `joeystick_state_setpoint` | `(4,) float64` | `quick_keyboard.py` | `active_com` |

Processes call `resource_tracker.unregister()` on shared memory they did not create to prevent auto-cleanup on exit.

### Process Roles

- **`vicon_sender.py`** — runs on the Vicon PC; reads from ViconDataStream SDK and sends msgpack UDP to `192.168.1.10:8020`
- **`vicon_collector.py`** — receives Vicon UDP, writes to `vicon_state` and `obstacle_corners` shared memory
- **`estimator_collector.py`** — reads MAVLink from serial `/dev/ttyTHS1`, publishes odometry to `estimated_state` and actuation to `actuation`
- **`hongi_process.py`** — loads `QuickHongi` (PyTorch neural network), reads `estimated_state` + `general_setpoint`, writes motor commands to `low_level_control`
- **`triangulator.py`** — computes min 2D distance from drone to obstacle edges using Numba JIT, writes to `dist`
- **`active_com.py`** — 200 Hz main loop: sends MAVLink odometry/commands to drone at `udpout:192.168.0.3:14561`, logs data to HDF5, handles keyboard arming/modes
- **`baby_sitter.py`** — Viser 3D visualization server on port 8080
- **`joeystick_sender.py`** — runs on remote PC with joystick; sends UDP to `192.168.1.10:8001`

### State Representation

`estimated_state` layout: `[x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]` (indices 0-12).

Written by `estimator_collector.py` in this order: pos(0:3), vel(3:6), quat(6:10), angvel(10:13).
This matches the Thesis-NN 23-dim feature layout (pos·vel·quat·angvel are the first 13 dims).

### Flight Log Format (HDF5)

Logs saved to `alpha/flight_log.h5` (and archived in `alpha/past_logs/`). Keys:
- `vicon`: `(N, 7)` — `(timestamp_us, x, y, z, roll, pitch, yaw)`
- `actuation`: `(N, 4)` — motor commands (front-right, front-left, bottom-left, bottom-right)
- `corner`: `(N, M, 4, 3)` — obstacle corner positions
- `obstacle`: `(N, M, 6)` — obstacle CoM poses
- `setpoint`: `(N, 6)` — `(x, y, z, vx, vy, vz)`
- `minimum_distance`: `(N, 1)`

Use `alpha/log_viewer.py` to replay/visualize logs. Use `alpha/tests/log_cropper.py` to crop logs.

### Network Topology

| Link | Protocol | Address |
|------|----------|---------|
| Vicon PC → Onboard PC | UDP msgpack | `192.168.1.10:8020` |
| Joystick PC → Onboard PC | UDP struct | `192.168.1.10:8001` |
| Onboard PC → Drone | MAVLink UDP | `udpout:192.168.0.3:14561` |
| Drone → Onboard PC | MAVLink serial | `/dev/ttyTHS1` at 921600 baud |

### Key Libraries

- `pymavlink` — MAVLink communication (`include/quick_mavlink.py`)
- `vicon_dssdk` — Vicon DataStream SDK (vicon_sender only)
- `numba` — JIT-compiled distance computations in triangulator
- `torch` — Hongi neural network controller
- `viser` — 3D visualization
- `h5py` — flight log storage
- `msgpack` — Vicon UDP serialization
