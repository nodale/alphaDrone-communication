import time
import numpy as np

from include.quick_viser import QuickViser


def main():
    vis = QuickViser(
        port=8080,
        verbose=False
    )

    print("Viser server running...")
    print("Open http://localhost:8080 in your browser")

    try:
        while True:
            state_est = vis.est_state.copy()
            state_vic = vis.vic_state.copy()

            vis.update_point_clouds(state_est, state_vic)
            vis.update_velocity_lines(state_est, state_vic)
            vis.update_x(state_est, state_vic)
            vis.update_heading(state_est, state_vic)

            vis.update_status(
                f"""
EST position: {state_est[0]:.3f}, {state_est[1]:.3f}, {state_est[2]:.3f}
VICON position: {state_vic[0]:.3f}, {state_vic[1]:.3f}, {state_vic[2]:.3f}
"""
            )

            time.sleep(0.02) 

    except KeyboardInterrupt:
        print("Stopping visualization...")

    finally:
        vis.shm_est.close()
        vis.shm_vic.close()


if __name__ == "__main__":
    main()
