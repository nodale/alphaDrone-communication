import time
import threading
import h5py
import numpy as np
import viser
from viser.transforms import SO3

FILENAME = "past_logs/20260324_140012_LOG.h5"

with h5py.File(FILENAME, "r") as f:
    print("Keys:", list(f.keys()))
    for k in f.keys():
        print(k, f[k].shape)

with h5py.File(FILENAME, "r") as f:
    vic = f["vicon"][:]  # shape (N, 7): time, x, y, z, roll, pitch, yaw

t_est = vic[:, 0]
t_est = t_est - t_est[0]
pos_est = vic[:, 1:4]  
rpy_est = vic[:, 4:7] 

N = vic.shape[0]

def rpy_to_quat(rpy):
    roll, pitch, yaw = rpy
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

server = viser.ViserServer()
server.scene.add_grid(name="xy", plane="xy", width=10, height=10, cell_size=1.0)
server.scene.add_grid(name="xz", plane="xz", width=10, height=10, cell_size=1.0)
server.scene.add_grid(name="yz", plane="yz", width=10, height=10, cell_size=1.0)
server.scene.set_up_direction("-z")

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

with server.gui.add_folder("Playback"):
    btn_start = server.gui.add_button("Start")
    btn_pause = server.gui.add_button("Pause")
    btn_replay = server.gui.add_button(" Replay")

    time_slider = server.gui.add_slider(
        "Time (s)",
        min=0.0,
        max=float(t_est[-1]),
        step=0.001,
        initial_value=0.0,
    )

playing = False
idx = 0
lock = threading.Lock()

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
    playing = False

    with lock:
        new_t = time_slider.value

        idx = np.searchsorted(t_est, new_t)
        idx = int(np.clip(idx, 0, N - 1))

        p = pos_est[idx]
        qw, qx, qy, qz = quat_est[idx]
        body.position = p
        body.orientation = SO3(wxyz=np.array([qw, qx, qy, qz]))
        traj_points.points = pos_est[: idx + 1]
        traj_points.colors = np.tile([0.2, 0.6, 1.0], (idx + 1, 1))

def playback_loop():
    global idx, playing
    last_wall_time = time.time()

    while True:
        time.sleep(0.005)

        if playing is False:
            last_wall_time = time.time()
            continue

        if playing is True:
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
            body.orientation = SO3.from_quaternion(np.array([qw, qx, qy, qz]))

            traj_points.points = pos_est[:idx + 1]
            traj_points.colors = np.tile([0.2, 0.6, 1.0], (idx + 1, 1))

threading.Thread(target=playback_loop, daemon=True).start()
threading.Event().wait()
