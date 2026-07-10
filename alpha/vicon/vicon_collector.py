import time
import traceback

from multiprocessing import shared_memory
from vicon.quick_vicon import QuickVicon

def main():
    vicon = QuickVicon(
        address='192.168.1.10',
        port=8020,
        block=False
    )

    while True:
        try:
            vicon.update()

            #print("Received state:", vicon.shared_state.copy())

        except Exception as e:
            print("Error in main loop:", e)
            #vicon.sock.close()
            vicon.shm.close()
            vicon.shm.unlink()
            vicon.init_shm.close()
            vicon.init_shm.unlink()
            traceback.print_exc()


if __name__ == "__main__":
    main()
