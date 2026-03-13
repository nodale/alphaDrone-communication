import time
import traceback

from multiprocessing import shared_memory
from include.quick_vicon import QuickVicon   

def main():
    vicon = QuickVicon(
        address="10.183.217.138",
        port=8020,
        block=True
    )

    while True:
        try:
            vicon.update_state()

            #print("Received state:", vicon.shared_state.copy())

        except Exception as e:        
            print("Error in main loop:", e)
            vicon.sock.close()
            vicon.shm.close()
            vicon.shm.unlink()
            vicon.init_shm.close()
            vicon.init_shm.unlink()
            traceback.print_exc()


if __name__ == "__main__":
    main()
