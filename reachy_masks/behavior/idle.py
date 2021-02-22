import numpy as np
import time
import concurrent.futures

from threading import Thread, Event

from . import mask_actions as fa


class Idle(object):
    def __init__(self, reachy, hand_empty):
        self.reachy = reachy
        self.hand_empty = hand_empty

        self.behaviors = {
            'look_around': (0.5, 0.5, fa.look_around),
            'read_flyer': (0.0, 0.1, fa.read_flyer),
            'stretch_head': (0.1, 0.1, fa.stretch_head),
            'fall_asleep': (0.1, 0.1, fa.fall_asleep),
            'lonely': (0.1, 0.1, fa.lonely),
            'look_hand': (0.1, 0.0, fa.look_hand),
            'waiting': (0.1, 0.1, fa.waiting),
        }

        self._t = None

        self.y_look_at = 0
        self.z_look_at = 0

    def play(self, behavior_name, wait):
        if self.is_playing():
            self.wait_for_end_of_play()

        _, _, behavior_func = self.behaviors[behavior_name]

        #self._t = Thread(target=behavior_func, args=[self.reachy])
        #self._t.start()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(behavior_func, self.reachy)
            self.y_look_at,self.z_look_at = future.result()

        #while not self._t.is_alive():
         #   time.sleep(0.01)

        #if wait:
         #   self._t.join()

    def is_playing(self):
        if self._t is None:
            return False
        return self._t.is_alive()

    def wait_for_end_of_play(self):
        if self.is_playing():
            self._t.join()


class IdleForever(object):
    def __init__(self, idle_behavior):
        self.idle_behavior = idle_behavior

        self._t = None
        self.running = False

    def _play_random_behavior_forever(self):
        names = list(self.idle_behavior.behaviors.keys())
        if self.idle_behavior.hand_empty:
            p = [v[0] for v in self.idle_behavior.behaviors.values()]
        else:
            p = [v[1] for v in self.idle_behavior.behaviors.values()]

        while self.running:
            behavior_name = np.random.choice(names, p=p)
            self.idle_behavior.play(behavior_name, wait=True)

    def start(self):
        if self._t is not None:
            return

        self.running = True

        self._t = Thread(target=self._play_random_behavior_forever)
        self._t.start()

        while not self._t.is_alive():
            time.sleep(0.01)

    def stop(self):
        self.running = False

        if self._t is not None:
            self._t.join()
            self._t = None


if __name__ == '__main__':
    reachy = ...

    idle = Idle(reachy, behaviors=['look_at', 'look_flyers', ...])
    idle.play('look_around', wait=False)
    idle.is_playing()
    idle.play('lonely', wait=False)
    idle.wait_for_end_of_play()

    idle_forever = IdleForever(idle)
    idle_forever.start()
    idle_forever.stop()
