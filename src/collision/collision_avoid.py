from multiprocessing import Process
from Queue import Empty
import logging
import numpy as np
import math

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

    def run(self):
        print "Starting visualizer"
        if not ENABLE:
            print "No open cv found, giving up"
            return

        # Init vizualizers
        #video_viz = VideoOverlayVisualizer(self.video_file, 1, 1, 0, 160, .002, .127)
        radar_viz = RadarVisualizer()
        dsrc_viz = DsrcVisualizer()
    
        while True:
            try:
                # Get without waiting, throws exception if no new data
                self.current_state = self.queue.get(False)
            except Empty:
                pass
                #print 'no data'

            if not self.current_state:
            #    print "no data"
                continue

            #video_viz.update(self.current_state)
            radar_viz.update(self.current_state)
            dsrc_viz.update(self.current_state)

            # Display current frames for 1ms
            cv2.waitKey(1)
            
        # Cleanup on windows
        cv2.destroyAllWindows()
