import time
import threading
import h5py
import numpy as np
import viser
from viser.transforms import SO3

# =========================
# Load HDF5 data
# =========================
FILENAME = "past_logs/20260324_140012_LOG.h5"

with h5py.File(FILENAME, "r") as f:
    print("Keys:", list(f.keys()))
    for k in f.keys():
        print(k, f[k].shape)

with h5py.File(FILENAME, "r") as f:
    vic = f["vicon"][:]  # shape (N, 7): time, x, y, z, roll, pitch, yaw

# =========================
# Extract data
# =========================
t_est = vic[:, 0]
t_est = t_est - t_est[0]          # normalize time
pos_est = vic[:, 1:4]             # x, y, z
rpy_est = vic[:, 4:7]             # roll, pitch, yaw in radians

N = vic.shape[0]

# Convert roll-pitch-yaw to quaternions for viser
def rpy_to_quat(rpy):
    roll, pitch, yaw = rpy
    # Following aerospace convention: ZYX
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz])

quat_est = np.array([rpy_to_quat(rpy_est[i]) for i in range(N)])

# =========================
# Start viser server
# =========================
server = viser.ViserServer()

# =========================
# Scene objects
# =========================
traj_points = server.scene.add_point_cloud(
    name="trajectory",
    points=pos_est,
    colors=np.tile([0.2, 0.6, 1.0], (N, 1)),
    point_size=0.02,
)

body = server.scene.add_frame(
    name="body",
    axes_length=0.3,
    axes_radius=0.02,
)

# =========================
# GUI controls
# =========================
with server.gui.add_folder("Playback"):
    btn_start = server.gui.add_button("▶ Start")
    btn_pause = server.gui.add_button("⏸ Pause")
    btn_replay = server.gui.add_button("⟲ Replay")

    time_slider = server.gui.add_slider(
        "Time (s)",
        min=0.0,
        max=float(t_est[-1]),
        step=0.001,
        initial_value=0.0,
    )

# =========================
# Playback state
# =========================
playing = False
idx = 0
lock = threading.Lock()

# =========================
# GUI callbacks
# =========================
@btn_start.on_click
def _start(_):
    global playing
    playing = True

@btn_pause.on_click
def _pause(_):
    global playing
    playing = False

@btn_replay.on_click
def _replay(_):
    global idx, playing
    with lock:
        idx = 0
        time_slider.value = 0.0
    playing = True

@time_slider.on_update
def _slider_update(event):
    global idx, playing
    playing = False  # pause while scrubbing
    with lock:
        idx = np.searchsorted(t_est, event.value)
        idx = int(np.clip(idx, 0, N - 1))

# =========================
# Playback loop
# =========================
def playback_loop():
    global idx, playing
    last_wall_time = time.time()

    while True:
        time.sleep(0.005)

        if not playing:
            last_wall_time = time.time()
            continue

        with lock:
            if idx >= N - 1:
                playing = False
                continue

            dt_log = t_est[idx + 1] - t_est[idx]
            dt_wall = time.time() - last_wall_time

            if dt_wall < dt_log:
                continue

            last_wall_time = time.time()
            idx += 1

            time_slider.value = float(t_est[idx])

            p = pos_est[idx]
            qw, qx, qy, qz = quat_est[idx]

            body.position = p
            body.orientation = SO3.from_quaternion(
                np.array([qw, qx, qy, qz])
            )

# =========================
# Start playback thread
# =========================
threading.Thread(target=playback_loop, daemon=True).start()
threading.Event().wait()
