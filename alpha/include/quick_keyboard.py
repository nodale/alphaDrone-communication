from dataclasses import dataclass

import numpy as np
import threading
import sys
import select
import h5py
import time

@dataclass
class QuickKeyboard:
    quit_flag : bool = False
    pause_flag : bool = True
    reboot_flag : bool = False
    arm_flag : bool = False
    kill_flag : bool = False
    force_kill_flag : bool = False

    set_to_lift_flag : bool = False
    set_to_land_flag : bool = False
    set_to_active_flag : bool = False
    traverse_square_flag : bool = False
    traverse_eight_flag : bool = False

    def __init__(self, file="LOG.h5"):
        self.writer = h5py.File(file, "w")

        self.est_ds = self.writer.create_dataset(
            "estimated",
            shape=(0, 11),
            maxshape=(None, 11),
            dtype=np.float32,
            chunks=True
        )
        self.est_idx = 0

        self.vic_ds = self.writer.create_dataset(
            "vicon",
            shape=(0, 11),
            maxshape=(None, 11),
            dtype=np.float32,
            chunks=True
        )
        self.vic_idx = 0

        self.init_time = time.time()

        print("Press 'q' + Enter to quit")
        print("Press 'o' + Enter to continue/start recording")
        print("Press 'p' + Enter to pause recording")
        print("Press 'r' + Enter to reboot pixhawk")
        print("Press 'l' + Enter to arm the drone")
        print("Press 'j' + Enter to kill the drone")
        print("Press 'k' + Enter to forcefully kill the drone")

        print("Press 'm' + Enter to set the drone to lift")
        print("Press 'n' + Enter to set the drone to land")
        print("Press 'b' + Enter to set the drone to activate")
        print("Press 'v' + Enter to set the drone to traverse square")
        print("Press 'c' + Enter to set the drone to traverse eight")

    def _keyboard_listener(self):
        while not self.quit_flag:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    self.quit_flag = True
                    print("QUITTING")
                if key.lower() == 'p':
                    self.pause_flag = True
                    print("PAUSING")
                if key.lower() == 'o':
                    self.pause_flag = False
                    print("CONTINUING")
                if key.lower() == 'r':
                    self.reboot_flag = True
                    print("REBOOTING")
                if key.lower() == 'l':
                    self.arm_flag = True
                    print("ARMING")
                if key.lower() == 'j':
                    self.kill_flag = True
                    print("KILLING")
                if key.lower() == 'k':
                    self.force_kill_flag = True
                    print("FORCE KILLING")

                if key.lower() == 'm':
                    self.set_to_lift_flag = True
                    self.set_to_land_flag = False
                    self.traverse_square_flag = False
                    self.traverse_eight_flag = False
                    print("SET TO LIFT")
                if key.lower() == 'n':
                    self.set_to_lift_flag = False
                    self.set_to_land_flag = True
                    self.traverse_square_flag = False
                    self.traverse_eight_flag = False
                    print("SET TO LAND")
                if key.lower() == 'b':
                    self.set_to_active_flag = True
                    print("SET TO ACTIVE")
                if key.lower() == 'v':
                    self.set_to_lift_flag = False
                    self.set_to_land_flag = False
                    self.traverse_square_flag = True
                    self.traverse_eight_flag = False
                    print("TRAVERSING SQUARE")
                if key.lower() == 'c':
                    self.set_to_lift_flag = False
                    self.set_to_land_flag = False
                    self.traverse_square_flag = False
                    self.traverse_eight_flag = True
                    print("TRAVERSING EIGHT")

    def start(self):
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

