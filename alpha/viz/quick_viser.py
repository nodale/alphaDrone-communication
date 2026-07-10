from pathlib import Path

import numpy as np
import viser

try:
    from viser_urdf import ViserUrdf
    _URDF_AVAILABLE = True
except ImportError:
    _URDF_AVAILABLE = False

_TRAIL    = 50
_PT_SIZE  = 0.01
_HDG_LEN  = 0.3     # metres


def _rpy_to_wxyz(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy[0] / 2, rpy[1] / 2, rpy[2] / 2
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array(
        [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
         cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy],
        dtype=np.float32,
    )


def _quat_rotate(wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = wxyz
    return np.array([
        (1-2*(y*y+z*z))*v[0] + 2*(x*y-z*w)*v[1] + 2*(x*z+y*w)*v[2],
          2*(x*y+z*w)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z-x*w)*v[2],
          2*(x*z-y*w)*v[0] + 2*(y*z+x*w)*v[1] + (1-2*(x*x+y*y))*v[2],
    ], dtype=np.float32)


class DroneViser:
    """3D visualizer for drone state. No shared memory knowledge.

    state_est  : (13,) — x,y,z, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz
    vicon_state: (N,6) — per-object x,y,z, rx,ry,rz (Euler XYZ)
    setpoint   : (6,)  — x,y,z, vx,vy,vz
    """

    def __init__(self, port: int = 8080, urdf_path: str = None, verbose: bool = False):
        self.server = viser.ViserServer(port=port, verbose=verbose)
        self.server.scene.set_up_direction("-z")

        for plane in ("xy", "xz", "yz"):
            self.server.scene.add_grid(name=plane, plane=plane, width=10, height=10, cell_size=1.0)

        # Rolling position trails
        self._est_trail = np.zeros((_TRAIL, 3), dtype=np.float32)
        self._vic_trail = np.zeros((_TRAIL, 3), dtype=np.float32)
        self._n_est = self._n_vic = 0
        self._cloud_est = self.server.scene.add_point_cloud(
            "trail/ekf2",  self._est_trail, colors=(255, 0,   0), point_size=_PT_SIZE)
        self._cloud_vic = self.server.scene.add_point_cloud(
            "trail/vicon", self._vic_trail, colors=(0,   0, 255), point_size=_PT_SIZE)
        self._cloud_sp  = self.server.scene.add_point_cloud(
            "trail/sp",    np.zeros((1, 3), dtype=np.float32), colors=(0, 255, 0), point_size=_PT_SIZE)

        # Velocity arrow (estimated only — vicon carries no velocity data)
        self._vel = self.server.scene.add_line_segments(
            "velocity/ekf2", np.zeros((1, 2, 3), dtype=np.float32), colors=(255, 80, 0), line_width=2)

        # Heading arrows
        self._hdg_est = self.server.scene.add_line_segments(
            "heading/ekf2",  np.zeros((1, 2, 3), dtype=np.float32), colors=(0, 255,   0), line_width=4)
        self._hdg_vic = self.server.scene.add_line_segments(
            "heading/vicon", np.zeros((1, 2, 3), dtype=np.float32), colors=(0, 200, 255), line_width=4)

        # Drone pose frames — URDF attaches to these if available
        self._frame_est = self.server.scene.add_frame("drone/ekf2")
        self._frame_vic = self.server.scene.add_frame("drone/vicon")

        self._urdf_est = self._urdf_vic = None
        if urdf_path and _URDF_AVAILABLE:
            self._urdf_est = ViserUrdf(self.server, Path(urdf_path), root_node_name="drone/ekf2")
            self._urdf_vic = ViserUrdf(self.server, Path(urdf_path), root_node_name="drone/vicon")
            self._urdf_est.update_cfg({})
            self._urdf_vic.update_cfg({})
        elif urdf_path:
            print("[viser] viser_urdf not installed — URDF model disabled, showing frame axes")

        # Obstacles
        self._obstacles: dict = {}

        # Status panel
        self._status = self.server.gui.add_text("Status", " ", multiline=True, disabled=True)

    # ------------------------------------------------------------------ #

    def _push_trail(self, trail: np.ndarray, n: int, pos: np.ndarray) -> int:
        if n < _TRAIL:
            trail[n] = pos
            return n + 1
        trail[:-1] = trail[1:]
        trail[-1] = pos
        return n

    def update(self, state_est: np.ndarray, vicon_state: np.ndarray,
               setpoint: np.ndarray, obstacles: np.ndarray) -> None:
        pos_est  = state_est[:3].astype(np.float32)
        vel_est  = state_est[3:6].astype(np.float32)
        wxyz_est = state_est[6:10].astype(np.float32)   # already qw,qx,qy,qz
        pos_vic  = vicon_state[0, :3].astype(np.float32)
        wxyz_vic = _rpy_to_wxyz(vicon_state[0, 3:6])

        # Trails
        self._n_est = self._push_trail(self._est_trail, self._n_est, pos_est)
        self._n_vic = self._push_trail(self._vic_trail, self._n_vic, pos_vic)
        self._cloud_est.points = self._est_trail[:self._n_est]
        self._cloud_vic.points = self._vic_trail[:self._n_vic]
        self._cloud_sp.points  = setpoint[:3].reshape(1, 3).astype(np.float32)

        # Velocity
        self._vel.points = np.array([[pos_est, pos_est + vel_est]], dtype=np.float32)

        # Heading
        fwd = np.array([_HDG_LEN, 0.0, 0.0], dtype=np.float32)
        self._hdg_est.points = np.array([[pos_est, pos_est + _quat_rotate(wxyz_est, fwd)]], dtype=np.float32)
        self._hdg_vic.points = np.array([[pos_vic, pos_vic + _quat_rotate(wxyz_vic, fwd)]], dtype=np.float32)

        # Drone model
        self._frame_est.position = pos_est
        self._frame_est.wxyz     = wxyz_est
        self._frame_vic.position = pos_vic
        self._frame_vic.wxyz     = wxyz_vic

        # Obstacles
        current = set()
        for i, box in enumerate(obstacles):
            name = f"obstacle/box_{i}"
            current.add(name)
            if name in self._obstacles:
                self._obstacles[name].position = box[:3].astype(np.float32)
            else:
                self._obstacles[name] = self.server.scene.add_box(
                    name=name, dimensions=(0.2, 0.2, 0.2),
                    position=box[:3], wxyz=(1, 0, 0, 0), color=(5, 5, 5),
                    cast_shadow=False, receive_shadow=False,
                )
        for gone in set(self._obstacles) - current:
            self.server.scene.remove(gone)
            del self._obstacles[gone]

        self._status.value = (
            f"vic  pos: {pos_vic[0]:.3f}  {pos_vic[1]:.3f}  {pos_vic[2]:.3f}\n"
            f"vic  rot: {vicon_state[0,3]:.3f}  {vicon_state[0,4]:.3f}  {vicon_state[0,5]:.3f}\n"
            f"est  pos: {pos_est[0]:.3f}  {pos_est[1]:.3f}  {pos_est[2]:.3f}\n"
            f"est quat: {wxyz_est[0]:.3f}  {wxyz_est[1]:.3f}  {wxyz_est[2]:.3f}  {wxyz_est[3]:.3f}\n"
        )
