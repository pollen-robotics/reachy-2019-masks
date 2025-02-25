import time
import numpy as np 

from collections import deque
from threading import Thread, Event


class Head_Controller(object):
    def __init__(
        self,
        initial_position,
        cb,
        alpha=1,
        update_period=0.02,
        nb_last_update_used=10,
        overlap_factor=5,
        pid_params=[0, 0, 0, 0, 0, 0]
    ):
        self.origin = np.array(initial_position)
        self.target = np.array(initial_position)

        self.alpha = np.clip(alpha, 0, 1)
        self.overlap_factor = overlap_factor
        self.last_update = deque([], nb_last_update_used)

        self.t0 = time.time()
        self.dt = update_period

        self.running = Event()
        self._t = None
        self.cb = cb

        self.pid_params = pid_params

        self._time = [time.time()]

    def start(self):
        if not self.is_running():
            self._t = Thread(target=self._run)
            self._t.daemon = True
            self._t.start()
            self.running.wait()

    def stop(self):
        if self.is_running():
            self.running.clear()
            self._t.join()

    def is_running(self):
        if self._t is not None:
            return self._t.is_alive()

    def _run(self):
        self.running.set()

        while self.running.is_set():
            self.cb(self.interpolate())
            time.sleep(0.01)

    def set_new_target(self, new_target):

        self._time.append(time.time())
        self.origin = self.interpolate()
        self.t0 = time.time()
        self.target = self.alpha * new_target + (1 - self.alpha) * self.target

        self.last_update.append(self.t0)

    def interpolate(self):
        direction = self.target - self.origin
        t = np.clip((time.time() - self.t0) / self.dt, 0, 1)

        return self.origin + direction * t

    def track(self, cmd_yz, prev_yz, goal, input_controller):

        cmd_y, cmd_z = cmd_yz
        prev_y, prev_z = prev_yz
        xM, yM = input_controller

        Kpy, Kpz, Kiy, Kiz, Kdy, Kdz = self.pid_params

        target = np.array([yM, xM]) - goal

        cmd_z += np.round(-target[0] * Kpz + (target[0] - prev_z)*self.dt*Kdz, 3)
        cmd_y += np.round(-target[1] * Kpy + (target[1] - prev_y)*self.dt*Kdy, 3)

        self.set_new_target([cmd_y, cmd_z])
        return cmd_y, cmd_z
