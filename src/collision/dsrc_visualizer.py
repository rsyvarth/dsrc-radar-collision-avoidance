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
        
        self.center = None
        self.points = []
        
    def update(self, current_state):
        if not current_state['dsrc']:
            return

        msg = current_state['dsrc']['message']
        
        if not msg:
            return

        self.img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
        self.center = (msg['long'], msg['lat'])
        self.points.append((msg['long'], msg['lat']))

        if len(self.points) > 1000:
            self.points.pop(0)
        
        for point in self.points:
            #print self.center_x - msg['long']
            x_pos = IMG_WIDTH/2 - (self.center[0] - point[0])*300000
            y_pos = IMG_HEIGHT/2 + (self.center[1] - point[1])*300000

            color = (255,255,255) if point != self.center else (0,0,255)
            
            cv2.circle(self.img, (int(x_pos), int(y_pos)), 5, color)

        cv2.imshow("DSRC", self.img)
        
