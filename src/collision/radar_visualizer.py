import math
import numpy as np
try:
    import cv2
except ImportError:
    print "Error importing opencv"
    pass


IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3

class RadarVisualizer(object):
    def __init__(self):
        pass
        #img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
        #cv2.imshow("Circles!", img)

    def update(self, current_state):
        #print "Drawing"
	img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
	cv2.line(img, (10, 0), (IMG_WIDTH/2 - 5, IMG_HEIGHT), (100, 255, 255))
	cv2.line(img, (IMG_WIDTH - 10, 0), (IMG_WIDTH/2 + 5, IMG_HEIGHT), (100, 255, 255))

        objs = []
        if current_state['radar']:
            objs = current_state['radar']['entities']

        for track in objs:
            track_number = track['track_number']
            track_range = track[track_number+'_track_range']
            track_angle = (float(track[track_number+'_track_angle'])+90.0)*math.pi/180

            x_pos = math.cos(track_angle)*track_range*4
            y_pos = math.sin(track_angle)*track_range*4
            
            cv2.circle(img, (IMG_WIDTH/2 + int(x_pos), IMG_HEIGHT - int(y_pos) - 10), 5, (255, 255, 255))

        cv2.imshow("Circles!", img)
        #cv2.waitKey(1)
        
