import math
import numpy as np
try:
    import cv2
except ImportError:
    pass

IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3

class DsrcVisualizer(object):
    def __init__(self):
        self.img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
        self.center_x = None
        self.center_y = None
        
    def update(self, current_state):
        if not current_state['dsrc']:
            return

        msg = current_state['dsrc']['message']
        
        if not self.center_x and msg:
            self.center_x = msg['long']
            self.center_y = msg['lat']

        #print self.center_x - msg['long']
        x_pos = IMG_WIDTH/2 + (self.center_x - msg['long'])*300000
        y_pos = IMG_HEIGHT/2 + (self.center_y - msg['lat'])*300000

        cv2.circle(self.img, (int(x_pos), int(y_pos)), 5, (255,255,255))

        cv2.imshow("DSRC", self.img)
        
