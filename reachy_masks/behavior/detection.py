from PIL import Image

from edgetpu.detection.engine import DetectionEngine
from threading import Thread
import numpy as np
import time

import cv2 as cv


class Detection(object):

    def __init__(self, reachy, path_to_model):
        self.reachy = reachy
        self.engine = DetectionEngine(path_to_model)

        self._t = None
        self.running = False
        self._somebody_detected = False
        self._face_target = [0, 0, 0]  # xM, yM, size
        self._face_emb = [0, 0, 0, 0, 0]  # x1, y1, x2, y2, size
        self._image = None
        self._somebody_detected = False
        self._time = [time.time()]
        self._img_index = 0

    def start(self):
        if self._t is not None:
            return

        self.running = True

        self._t = Thread(target=self.detect)
        self._t.start()

        while not self._t.is_alive():
            time.sleep(0.01)

    def stop(self):

        self.running = False
        self._t = None

    def is_playing(self):
        if self._t is None:
            return False
        return self._t.is_alive()
    
    def detect(self):

        while self.running:
            self._time.append(time.time())
            self._image = self.reachy.head.get_image()
            pil_img = Image.fromarray(self._image)

            candidates = self.engine.detect_with_image(pil_img, relative_coord=False)

            cv.imwrite("/home/pi/dev/reachy-masks/reachy_masks/tmp_img/img." + str(self._img_index) + ".jpg", self._image)
            self._img_index += 1

            if not candidates:
                self._somebody_detected = False

            else:
                self._somebody_detected = True
                sizes, faces, face_emb = [], [], []
                for candidate in candidates:
                    [[x1, y1], [x2, y2]] = candidate.bounding_box
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    sizes.append((x2-x1)*(y2-y1))
                    faces.append([int((x1+x2)/2), int((y1+y2)/2)])
                    face_emb.append([x1, y1, x2, y2])

                self._face_emb[0], self._face_emb[1], self._face_emb[2], self._face_emb[3] = face_emb[np.argmax(max(sizes))]
                self._face_target[0], self._face_target[1] = faces[np.argmax(max(sizes))]
                self._face_target[2] = max(sizes)

            time.sleep(0.01)
