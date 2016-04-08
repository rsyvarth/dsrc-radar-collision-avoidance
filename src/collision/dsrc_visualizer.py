import math
import numpy as np
try:
    import cv2
except ImportError:
    pass
'''
IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3
'''

class DsrcVisualizer(object):

    def __init__(self, width, height, channels):
        self.center = None
        self.points = []
        self.remote_points = []
        self.width = width
        self.height = height
        self.channels = channels
        self.img = np.zeros((self.height, self.width, self.channels), np.uint8)
        self.center_x = None
        self.center_y = None
        #img = np.zeros((self.width, self.height, self.channels), np.uint8)
        cv2.imshow("DSRC", self.img)

        self.speed = 0;
        self.remote_speed = 0;

        
    def update(self, current_state):
        if not current_state['dsrc']:
            return

        msg = current_state['dsrc']['message']
        
        if not msg:
            return


        self.img = np.zeros((self.height, self.width, self.channels), np.uint8)
        self.center = (msg['long'], msg['lat'])
        self.points.append((msg['long'], msg['lat']))
        self.speed = msg['speed']

        if len(self.points) > 1000:
            self.points.pop(0)
        
        for point in current_state['dsrc']['remote_messages']:
            self.remote_points.append((point['long'], point['lat']))
            self.remote_speed = point['speed']

            if len(self.remote_points) > 1000:
                self.remote_points.pop(0)

        

        for point in self.remote_points:
            x_pos = self.width/2 - (self.center[0] - point[0])*300000
            y_pos = self.height/2 + (self.center[1] - point[1])*300000

            cv2.circle(self.img, (int(x_pos), int(y_pos)), 5, (100,100,100))

        for point in self.points:
            #print self.center_x - msg['long']
            x_pos = self.width/2 - (self.center[0] - point[0])*300000
            y_pos = self.height/2 + (self.center[1] - point[1])*300000

            color = (255,255,255) if point != self.center else (0,0,255)
            
            cv2.circle(self.img, (int(x_pos), int(y_pos)), 5, color)

        cv2.putText(self.img, "Local Speed : %s" % self.speed, (10,15), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255))
        cv2.putText(self.img, "Remote Speed: %s" % self.remote_speed, (10,30), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255))

        cv2.imshow("DSRC", self.img)
        
