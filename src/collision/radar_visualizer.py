import math
import numpy as np
try:
    import cv2
except ImportError:
    pass


IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3

class RadarVisualizer(Object):
    def __init__():
        pass

    def update(self, current_state):

		img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
	    cv2.line(img, (10, 0), (IMG_WIDTH/2 - 5, IMG_HEIGHT), (100, 255, 255))
	    cv2.line(img, (IMG_WIDTH - 10, 0), (IMG_WIDTH/2 + 5, IMG_HEIGHT), (100, 255, 255))
	    cv2.circle(img, (25, 25), 5, (255, 255, 255))

	    cv2.imshow("Circles!", img)
