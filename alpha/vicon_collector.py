import time
import traceback

from multiprocessing import shared_memory
from include.quick_vicon import QuickVicon   

def main():
    vicon = QuickVicon(
        address="10.183.217.138",
        port=8020,
        block=False
    )

    while True:
        try:
            data = vicon.get_data()

            if data is not None:
                vicon.update_state()


            #print("Received state:", vicon.shared_state.copy())

        except Exception as e:        
            #print("Error in main loop:", e)
            vicon.sock.close()
            vicon.shm.close()
            vicon.shm.unlink()
            traceback.print_exc()

    #except KeyboardInterrupt:
    #    #print("\nStopping Vicon listener...")
    #    vicon.sock.close()
    #    vicon.shm.close()
    #    vicon.shm.unlink()
    #    time.sleep(1)

    #finally:
    #    vicon.sock.close()
    #    vicon.shm.close()
    #    vicon.shm.unlink()
    #    time.sleep(1)


if __name__ == "__main__":
    main()
