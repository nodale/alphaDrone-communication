import time
import traceback
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
                vicon._publish_data()
                #print("Received state:", data)

            time.sleep(0.01)

        except Exception as e:        
            #print("Error in main loop:", e)
            vicon.sock.close()
            vicon.shm.close()
            vicon.shm.unlink()
            traceback.print_exc()
            time.sleep(1)

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
