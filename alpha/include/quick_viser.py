from dataclasses import dataclass

import numpy as np
import viser

#state is (x, y, z, vx, vy, vz, roll, pitch, yaw, roll_ang_vel, pitch_ang_vel, yaw_ang_vel)


@dataclass
class QuickViser:
    POINT_SIZE : float = 0.01
    MAX_POINTS : int = 50
    xf : float = 0.158
    xb : float = 0.158
    yl : float = 0.158
    yr : float = 0.158

    def __init__(self, port=8080, verbose=True):
        #cross offset setup
        self.cross_offset = np.array([
            [ self.xf,  self.yr, 0.0],   
            [ self.xf, -self.yl, 0.0],   
            [-self.xb,  self.yl, 0.0],   
            [-self.xb, -self.yr, 0.0],   
        ], dtype=np.float32)

        #viser setup
        self.server = viser.ViserServer(port=port, verbose=verbose)
        self.server.scene.set_up_direction("-z") #adheres to PX4 orientation

        #adding grids
        self.server.scene.add_grid(name="xy", plane="xy", width=10, height=10, cell_size=1.0)
        self.server.scene.add_grid(name="xz", plane="xz", width=10, height=10, cell_size=1.0)
        self.server.scene.add_grid(name="yz", plane="yz", width=10, height=10, cell_size=1.0)

        #point clouds for the positions
        self.est_odo_points = np.zeros((self.MAX_POINTS, 3), dtype=np.float32)
        self.est_num_points = 0 
        self.vic_odo_points = np.zeros((self.MAX_POINTS, 3), dtype=np.float32)
        self.vic_num_points = 0

        self.estimated_point_cloud_handle = self.server.scene.add_point_cloud(
            name="EKF2",
            points=self.est_odo_points,
            colors=(255, 0, 0),
            point_size=self.POINT_SIZE,
        )

        self.vicon_point_cloud_handle = self.server.scene.add_point_cloud(
            name="vicon",
            points=self.vic_odo_points,
            colors=(0, 0, 255),
            point_size=self.POINT_SIZE,
        )

        #adding lines for velocity
        self.vel_line_est = np.zeros((1, 2, 3), dtype=np.float32)
        self.vel_handle_est = self.server.scene.add_line_segments(
            name="velocity_est",
            points=self.vel_line_est,
            colors=(255, 20, 0),
            line_width=2
        )

        self.vel_line_vic = np.zeros((1, 2, 3), dtype=np.float32)
        self.vel_handle_vic = self.server.scene.add_line_segments(
            name="velocity_vic",
            points=self.vel_line_vic,
            colors=(0, 20, 255),
            line_width=2
        )

        #add X of the drones
        self.x_lines_est = np.zeros((4, 2, 3), dtype=np.float32)
        self.x_handle_est = self.server.scene.add_line_segments(
            name="drone_est",
            points=self.x_lines_est,
            colors=(100, 20, 20),
            line_width=2
        )

        self.x_lines_vic = np.zeros((4, 2, 3), dtype=np.float32)
        self.x_handle_vic = self.server.scene.add_line_segments(
            name="drone_vic",
            points=self.x_lines_vic,
            colors=(20, 20, 100),
            line_width=2
        )

        #add actuation lines
        self.act_lines_est = np.zeros((4, 2, 3), dtype=np.float32)
        self.act_handle_est = self.server.scene.add_line_segments(
            name="actuation_est",
            points=self.act_lines_est,
            colors=(255, 150, 0),
            line_width=4,
        )

        self.act_lines_vic = np.zeros((4, 2, 3), dtype=np.float32)
        self.act_handle_vic = self.server.scene.add_line_segments(
            name="actuation_vic",
            points=self.act_lines_vic,
            colors=(0, 150, 255),
            line_width=4,
        )

    def update_point_clouds(self, state_est, state_vic):
        if state_est is not None:
            if self.est_num_points < self.MAX_POINTS:
                self.est_odo_points[self.est_num_points] = [state_est[0], state_est[1], state_est[2]]
                self.est_num_points += 1
            else:
                self.est_odo_points[:-1] = self.est_odo_points[1:]
                self.est_odo_points[-1] = [state_est[0], state_est[1], state_est[2]]

            self.estimated_point_cloud_handle.points = self.est_odo_points[:self.est_num_points]

        if state_vic is not None:
            if self.vic_num_points < self.MAX_POINTS:
                self.vic_odo_points[self.vic_num_points] = [state_vic[0], state_vic[1], state_vic[2]]
                self.vic_num_points += 1
            else:
                self.vic_odo_points[:-1] = self.vic_odo_points[1:]
                self.vic_odo_points[-1] = [state_vic.x, state_vic.y, state_vic.z]

            self.vicon_point_cloud_handle.points = self.vic_odo_points[:self.vic_num_points]

    def update_velocity_lines(self, state_est, state_vic):
        if state_est is not None:
            self.vel_line_est[0, 0] = [state_est[0], state_est[1], state_est[2]]   
            self.vel_line_est[0, 1] = [state_est[0] + state_est[3]*1, state_est[1] + state_est[4]*1, state_est[2] + state_est[5]*1]
            self.vel_handle_est.points = self.vel_line_est

        if state_vic is not None:
            self.vel_line_vic[0, 0] = [state_vic[0], state_vic[1], state_vic[2]]   
            self.vel_line_vic[0, 1] = [state_vic[0] + state_vic[3]*1, state_vic[1] + state_vic[4]*1, state_vic[2] + state_vic[5]*1]
            self.vel_handle_vic.points = self.vel_line_vic
        
    def _rpy_to_rot(self, roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ], dtype=np.float32)

        return R

    def _transform_points(self, state):
        R = self._rpy_to_rot(state[6], state[7], state[8])
        points_world = (R @ self.cross_offset.T).T + np.array((state[0], state[1], state[2]), dtype=np.float32)
        return points_world

    def _gen_x(self, points):
        lines = np.zeros((4, 2, 3), dtype=np.float32)
        lines[0] = [points[0], points[3]]
        lines[1] = [points[1], points[2]]
        lines[2] = [points[0], points[2]]
        lines[3] = [points[1], points[3]]
        return lines

    def update_x(self, state_est, state_vic):
        if state_est is not None:
            center = [state_est[0], state_est[1], state_est[2]]

            self.est_x_world = self._transform_points(state_est)
            x_lines_est = self._gen_x(self.est_x_world)
            self.x_handle_est.points = x_lines_est

        if state_vic is not None:
            center = [state_vic[0], state_vic[1], state_vic[2]]

            self.vic_x_world = self._transform_points(state_vic)
            x_lines_vic = self._gen_x(self.vic_x_world)
            self.x_handle_vic.points = x_lines_vic


    def _gen_actuation(
            self,
            x,
            state,
            actuation,
            scale=0.15
        ):
        R = self._rpy_to_rot(state[6], state[7], state[8])

        thrust_dir_body = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        thrust_dir_world = R @ thrust_dir_body

        lines = np.zeros((4, 2, 3), dtype=np.float32)

        for i in range(4):
            start = x[i]
            end = start + thrust_dir_world * actuation[i] * scale

            lines[i, 0] = start
            lines[i, 1] = end

        return lines

    def update_actuation(self, state_est, state_vic, actuation):
        if actuation is not None:
            if state_est is not None:
                self.act_lines_est = self._gen_actuation(
                        self.est_x_world,
                        state_est,
                        actuation,
                        0.15
                    )
                self.act_handle_est.points = self.act_lines_est

            if state_vic is not None:
                self.act_lines_vic = self._gen_actuation(
                        self.vic_x_world,
                        state_vic,
                        actuation,
                        0.15
                    )
                self.act_handle_vic.points = self.act_lines_vic
