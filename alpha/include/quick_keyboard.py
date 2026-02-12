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

    def start(self):
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

