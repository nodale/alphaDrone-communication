from include.quick_mavlink import QuickMav
from pymavlink import mavutil
import time
import viser
import socket
import msgpack
import numpy as np
import threading
import sys
import select
import h5py


quit_flag = False
pause_flag = True
reboot_flag = False
arm_flag = False
kill_flag = False
force_kill_flag = False

def keyboard_listener():
    global quit_flag
    global pause_flag
    global reboot_flag
    global arm_flag
    global kill_flag
    global force_kill_flag

    while not quit_flag:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key.lower() == 'q':
                quit_flag = True
                print("QUITTING")
            if key.lower() == 'p':
                pause_flag = True
                print("PAUSING")
            if key.lower() == 'o':
                pause_flag = False
                print("CONTINUING")
            if key.lower() == 'r':
                reboot_flag = True
                print("REBOOTING")
            if key.lower() == 'l':
                arm_flag = True
                print("ARMING")
            if key.lower() == 'j':
                kill_flag = True
                print("KILLING")
            if key.lower() == 'k':
                force_kill_flag = True
                print("FORCE KILLING")



HOST = "10.183.217.138"
PORT = 8020

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.setblocking(False)

def getViconData():
    try:
        payload, addr = sock.recvfrom(2048)
        msg = msgpack.unpackb(payload, raw=False)

        x, y, z = msg[2:5]
        vx, vy, vz = msg[5:8]
        q = msg[8:12]
        return (x, y, z, vx, vy, vz, q)

    except BlockingIOError:
        return None

    except Exception as e:
        print("Vicon error:", e)
        return None

def rpy_to_rot(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ], dtype=np.float32)

    return R

def generate_leg_x_body(xf, xb, y_offset):
    points = np.array([
        [ xf,  y_offset, 0.0],   
        [ xf, -y_offset, 0.0],   
        [-xb,  y_offset, 0.0],   
        [-xb, -y_offset, 0.0],   
    ], dtype=np.float32)
    return points

def quat_to_rpy(q):
    w, x, y, z = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def transform_points(points_body, center, roll, pitch, yaw):
    R = rpy_to_rot(roll, pitch, yaw)
    points_world = (R @ points_body.T).T + np.array(center, dtype=np.float32)
    return points_world

def generate_x_lines(points):
    lines = np.zeros((4, 2, 3), dtype=np.float32)
    lines[0] = [points[0], points[3]]
    lines[1] = [points[1], points[2]]
    lines[2] = [points[0], points[2]]
    lines[3] = [points[1], points[3]]
    return lines

def build_actuation_arrows(
    center,
    roll, pitch, yaw,
    actuation,
    scale=0.2
):
    dirs_body = build_actuation_arrows_from_legs()
    R = rpy_to_rot(roll, pitch, yaw)

    lines = np.zeros((4, 2, 3), dtype=np.float32)

    for i in range(4):
        mag = actuation[i]
        dir_body = dirs_body[i] * mag * scale
        dir_world = R @ dir_body

        lines[i, 0] = center
        lines[i, 1] = center + dir_world

    return lines


def append_row(dset, idx, row):
    dset.resize(idx + 1, axis=0)
    dset[idx] = row

def build_actuation_arrows_from_legs(
    leg_points_world,
    roll, pitch, yaw,
    actuation,
    scale=0.15
):
    R = rpy_to_rot(roll, pitch, yaw)

    thrust_dir_body = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    thrust_dir_world = R @ thrust_dir_body

    lines = np.zeros((4, 2, 3), dtype=np.float32)

    for i in range(4):
        start = leg_points_world[i]
        end = start + thrust_dir_world * actuation[i] * scale

        lines[i, 0] = start
        lines[i, 1] = end

    return lines

def main():
    global arm_flag
    global kill_flag
    global force_kill_flag

    MAX_POINTS = 50  
    POINT_SIZE = 0.01

    server = viser.ViserServer(port=8080, verbose=True)

    server.scene.set_up_direction('-z')

    server.scene.add_grid(name="grid_xy", plane="xy", width=10, height=10, cell_size=1.0)
    server.scene.add_grid(name="grid_xz", plane="xz", width=10, height=10, cell_size=1.0)
    server.scene.add_grid(name="grid_yz", plane="yz", width=10, height=10, cell_size=1.0)

    est_odo_points = np.zeros((MAX_POINTS, 3), dtype=np.float32)
    est_num_points = 0 
    vic_odo_points = np.zeros((MAX_POINTS, 3), dtype=np.float32)
    vic_num_points = 0

    status_handle = server.gui.add_text("Status", " ", multiline=True, disabled=True)

    estimated_point_cloud_handle = server.scene.add_point_cloud(
        name="estimated",
        points=est_odo_points,
        colors=(255, 0, 0),
        point_size=POINT_SIZE,
    )

    vicon_point_cloud_handle = server.scene.add_point_cloud(
        name="vicon",
        points=vic_odo_points,
        colors=(0, 0, 255),
        point_size=POINT_SIZE,
    )

    vel_line_est = np.zeros((1, 2, 3), dtype=np.float32)
    vel_handle_est = server.scene.add_line_segments(
        name="velocity_est",
        points=vel_line_est,
        colors=(255, 20, 0),
        line_width=2
    )

    vel_line_vic = np.zeros((1, 2, 3), dtype=np.float32)
    vel_handle_vic = server.scene.add_line_segments(
        name="velocity_vic",
        points=vel_line_vic,
        colors=(0, 20, 255),
        line_width=2
    )

    xf, xb, y_offset = 0.158, 0.158, 0.16

    x_lines_est = np.zeros((4, 2, 3), dtype=np.float32)
    x_handle_est = server.scene.add_line_segments(
        name="drone_est",
        points=x_lines_est,
        colors=(100, 50, 50),
        line_width=2
    )

    x_lines_vic = np.zeros((4, 2, 3), dtype=np.float32)
    x_handle_vic = server.scene.add_line_segments(
        name="drone_vic",
        points=x_lines_vic,
        colors=(20, 20, 100),
        line_width=2
    )

    act_lines_est = np.zeros((4, 2, 3), dtype=np.float32)
    act_handle_est = server.scene.add_line_segments(
        name="actuation_est",
        points=act_lines_est,
        colors=(255, 150, 0),
        line_width=4,
    )

    act_lines_vic = np.zeros((4, 2, 3), dtype=np.float32)
    act_handle_vic = server.scene.add_line_segments(
        name="actuation_vic",
        points=act_lines_vic,
        colors=(0, 150, 255),
        line_width=4,
    )



    h5file = h5py.File("flight_log.h5", "w")

    est_ds = h5file.create_dataset(
        "estimated",
        shape=(0, 11),
        maxshape=(None, 11),
        dtype=np.float32,
        chunks=True
    )

    vic_ds = h5file.create_dataset(
        "vicon",
        shape=(0, 11),
        maxshape=(None, 11),
        dtype=np.float32,
        chunks=True
    )

    est_idx = 0
    vic_idx = 0
    latest_actuation = np.zeros(4, dtype=np.float32)
    t0 = time.time()



    #com = QuickMav(address="udpout:192.168.0.3:14550", baudrate=200000)
    com = QuickMav(address="/dev/ttyTHS1", baudrate=921600)
    com.sendHeartbeat()

    threading.Thread(target=keyboard_listener, daemon=True).start()

    print("Press 'q' + Enter to quit")
    print("Press 'o' + Enter to continue/start recording")
    print("Press 'p' + Enter to pause recording")
    print("Press 'r' + Enter to reboot pixhawk")
    print("Press 'l' + Enter to arm the drone")
    print("Press 'j' + Enter to kill the drone")
    print("Press 'k' + Enter to forcefully kill the drone")

    try:
        while not quit_flag:
            if arm_flag == True:
                arm_flag = False
                com.arm()
            if kill_flag == True:
                com.disarm()
                kill_flag = False
            if force_kill_flag == True:
                com.forceDisarm()
                force_kill_flag = False


            est_odo = com.get("ODOMETRY", block=False)
            if est_odo is not None:
                x, y, z = est_odo.x, est_odo.y, est_odo.z
                vx, vy, vz = est_odo.vx, est_odo.vy, est_odo.vz
                q = est_odo.q

                roll, pitch, yaw = quat_to_rpy(q) 

                if est_num_points < MAX_POINTS:
                    est_odo_points[est_num_points] = [x, y, z]
                    est_num_points += 1
                else:
                    est_odo_points[:-1] = est_odo_points[1:]
                    est_odo_points[-1] = [x, y, z]

                estimated_point_cloud_handle.points = est_odo_points[:est_num_points]

                vel_line_est[0, 0] = [x, y, z]   
                vel_line_est[0, 1] = [x + vx*1, y + vy*1, z + vz*1]
                vel_handle_est.points = vel_line_est

                center = [est_odo.x, est_odo.y, est_odo.z]

                points_body = generate_leg_x_body(xf, xb, y_offset)
                points_world = transform_points(
                    points_body,
                    center,
                    roll,
                    pitch,
                    yaw
                )

                x_lines_est = generate_x_lines(points_world)
                x_handle_est.points = x_lines_est

                if latest_actuation is not None:
                    act_lines_est = build_actuation_arrows_from_legs(
                        leg_points_world=points_world,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        actuation=latest_actuation,
                        scale=0.15
                    )
                    act_handle_est.points = act_lines_est

                if pause_flag == False:
                    t = time.time() - t0

                    row = np.array([
                        t,
                        x, y, z,
                        vx, vy, vz,
                        q[1], q[2], q[3], q[0]  
                    ], dtype=np.float32)

                    append_row(est_ds, est_idx, row)
                    est_idx += 1
                    h5file.flush()


            vic_odo = getViconData()
            if vic_odo is not None:
                x, y, z = vic_odo[0:3]
                vx, vy, vz = vic_odo[3:6]
                roll, pitch, yaw = quat_to_rpy(vic_odo[6])


                if vic_num_points < MAX_POINTS:
                    vic_odo_points[vic_num_points] = [x, y, z]
                    vic_num_points += 1
                else:
                    vic_odo_points[:-1] = vic_odo_points[1:]
                    vic_odo_points[-1] = [x, y, z]

                vicon_point_cloud_handle.points = vic_odo_points[:vic_num_points]

                vel_line_vic[0, 0] = [x, y, z]
                vel_line_vic[0, 1] = [x + vx*0.5, y + vy*0.5, z + vz*0.5]
                vel_handle_vic.points = vel_line_vic

                center = vic_odo[:3]

                points_body = generate_leg_x_body(xf, xb, y_offset)
                points_world = transform_points(
                    points_body,
                    center,
                    roll,
                    pitch,
                    yaw
                )

                x_lines_vic = generate_x_lines(points_world)
                x_handle_vic.points = x_lines_vic

                if latest_actuation is not None:
                    act_lines_vic = build_actuation_arrows_from_legs(
                        leg_points_world=points_world,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        actuation=latest_actuation,
                        scale=0.15
                    )
                    act_handle_vic.points = act_lines_vic

                if pause_flag == False:
                    t = time.time() - t0

                    q = vic_odo[6]
                    row = np.array([
                        t,
                        x, y, z,
                        vx, vy, vz,
                        q[1], q[2], q[3], q[0]
                    ], dtype=np.float32)

                    append_row(vic_ds, vic_idx, row)
                    vic_idx += 1
                    h5file.flush()

            johnny_status = com.get("JOHNNY_STATUS", block=False)
            if johnny_status is not None:
                latest_actuation = np.array(johnny_status.actuation, dtype=np.float32)

            mode = com.get("HEARTBEAT", block=False)
            if mode is not None:
                print(mode)

            status_msg = (
                f"Pause: {pause_flag}\n"
                f"Arm: {arm_flag}\n"
                f"Kill: {kill_flag}\n"
                f"Force Kill: {force_kill_flag}\n"
                f"Latest Actuation: {latest_actuation}\n"
            )

            status_handle.value = status_msg

            time.sleep(0.025) 

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        h5file.close()
        com.master.close()
        print("CONNECTION CLOSED")

if __name__ == "__main__":
    main()

