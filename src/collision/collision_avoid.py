from multiprocessing import Process
from Queue import Empty
import logging
import numpy as np
import math

CHANNELS = 3
ENABLE = True
try:
    import cv2
except ImportError:
    ENABLE = False

from collision_constants import CollisionConstants
from dsrc_visualizer import DsrcVisualizer
from radar_visualizer import RadarVisualizer
from video_overlay_visualizer import VideoOverlayVisualizer

class CollisionAvoidance(Process):
    """ Takes combined data and displays predicted collisions.

    This package is included mainly for the purpose of demonstration.
    It takes the information provided by our Combiner and calculates any predicted
    collisions displaying warnings in a simple UI.
    """

    def __init__(self, queue, video_file):
        """Setup the CA class, just empty state for now"""
        Process.__init__(self)
        self.queue = queue
        self.video_file = video_file
        self.current_state = None
        self.constants = CollisionConstants()


    def position_windows(self):
        cv2.moveWindow('Data Visualizer', 0, 0)
        ret, img = self.video_viz.camera.read()
        if not ret:
            return

        img_height, img_width = img.shape[:2]
        self.radar_viz = RadarVisualizer(img_height/2,img_height/2, CHANNELS)
        self.dsrc_viz = DsrcVisualizer(img_height/2,img_height/2, CHANNELS)
        cv2.moveWindow('Radar', img_width, 0)
        cv2.moveWindow('DSRC', img_width, img_height/2)
        return

    def run(self):
        print "Starting visualizer"
        if not ENABLE:
            print "No open cv found, giving up"
            return

        # Init vizualizers

        self.video_viz = VideoOverlayVisualizer(self.video_file, 1, 1, 0, 78.58, \
        .022, .00616)

        self.position_windows()
        while True:
            if self.queue.qsize() > 2:
                print 'WARNING: The queue depth is %s, we are behind real time!' % self.queue.qsize()
            try:
                # Get without waiting, throws exception if no new data
                self.current_state = self.queue.get(False)
            except Empty:
                pass
                #print 'no data'

            if not self.current_state:
            #    print "no data"
                continue


            if not self.video_viz.update(self.current_state):
                break

            self.radar_viz.update(self.current_state)
            self.dsrc_viz.update(self.current_state)

            # Display current frames for 1ms
            cv2.waitKey(40)

        # Cleanup on windows
        #cv2.destroyAllWindows()
