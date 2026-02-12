import time
import threading
import h5py
import numpy as np
import viser
from viser.transforms import SO3

# =========================
# Load HDF5 data
# =========================
FILENAME = "flight_log.h5"

with h5py.File(FILENAME, "r") as f:
    est = f["estimated"][:]  # (N, 11)

t_est = est[:, 0]
pos_est = est[:, 1:4]
quat_est = est[:, 7:11]  # qx qy qz qw

# Normalize time
t_est = t_est - t_est[0]
N = len(t_est)

# =========================
# Start viser server
# =========================
server = viser.ViserServer()

# =========================
# Scene objects (VALID APIs)
# =========================

# Trajectory as point cloud
traj_points = server.scene.add_point_cloud(
    name="trajectory",
    points=pos_est,
    colors=np.tile([0.2, 0.6, 1.0], (N, 1)),
    point_size=0.02,
)

# Body frame (pose)
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
    global idx
    with lock:
        idx = np.searchsorted(t_est, event.value)
        idx = np.clip(idx, 0, N - 1)

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

            # Update slider
            time_slider.value = float(t_est[idx])

            # Update pose
            p = pos_est[idx]
            qx, qy, qz, qw = quat_est[idx]

            body.position = p
            body.orientation = SO3.from_quaternion(
                np.array([qw, qx, qy, qz])  # viser expects w,x,y,z
            )

# =========================
# Start playback thread
# =========================
threading.Thread(target=playback_loop, daemon=True).start()
threading.Event().wait()
print("Viser server running. Open the printed URL.")

