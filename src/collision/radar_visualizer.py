import math
import numpy as np
try:
    import cv2
except ImportError:
    print "Error importing opencv"
    pass

'''
IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3
'''

class RadarVisualizer(object):
    def __init__(self, width, height, channels):
        self.width = width
        self.height = height
        self.channels = channels
        img = np.zeros((self.height, self.width, self.channels), np.uint8)
        cv2.imshow("Radar", img)
        pass

    def update(self, current_state):
        #print "Drawing"
	img = np.zeros((self.height, self.width, self.channels), np.uint8)
	cv2.line(img, (10, 0), (self.width/2 - 5, self.height), (100, 255, 255))
	cv2.line(img, (self.width - 10, 0), (self.width/2 + 5, self.height), (100, 255, 255))

        objs = []
        if current_state['radar']:
            objs = current_state['radar']['entities']

        for track in objs:
            track_number = track['track_number']
            track_range = track[track_number+'_track_range']
            track_angle = (float(track[track_number+'_track_angle'])+90.0)*math.pi/180

            x_pos = math.cos(track_angle)*track_range*4
            y_pos = math.sin(track_angle)*track_range*4

            cv2.circle(img, (self.width/2 + int(x_pos), self.height - int(y_pos) - 10), 5, (255, 255, 255))

        cv2.imshow("Radar", img)
        #cv2.waitKey(1)

