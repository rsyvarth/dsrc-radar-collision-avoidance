from multiprocessing import Process
from Queue import Empty
import logging
import numpy as np
import math
import signal
import timeit

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
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        print 'You pressed Ctrl+C!'
        self.terminate()



    def position_windows(self):
        cv2.moveWindow('Data Visualizer', 0, 0)

        img_height = int(self.video_viz.video_height)
        img_width = int(self.video_viz.video_width)

        self.radar_viz = RadarVisualizer(img_height/2 - 10, img_height/2 - 10, CHANNELS)
        self.dsrc_viz = DsrcVisualizer(img_height/2 - 10, img_height/2 - 10, CHANNELS)
        cv2.moveWindow('Radar', img_width + 5, 0)
        cv2.moveWindow('DSRC', img_width + 5, img_height/2 + 15)

    def run(self):
        print "Starting visualizer"
        if not ENABLE:
            print "No open cv found, giving up"
            return

        # Init vizualizers
        # self.video_viz = VideoOverlayVisualizer(self.video_file, 1.2446, -0.4, 0, 94.4, \
        # 21.0/1000, 4.55/1000)
        self.video_viz = VideoOverlayVisualizer(self.video_file, 1.2446, -0.4, 0, 64.6, \
        28.0/1000, 4.55/1000)
        self.position_windows()

        delay_remainder = 0.0;
        start_time = timeit.default_timer()

        while True:
            # Start timing to record how long our drawing takes
            start_frame_time = timeit.default_timer()

            # Get the last element from the data queue (get rid of any old data too)
            while self.queue.qsize() > 1:
                try:
                    self.current_state = self.queue.get(False)
                except Empty:
                    break

            # if we don't have data yet, don't bother visualizing
            if not self.current_state:
                continue

            # If we are behind in the video compared to real time, grab a new
            # frame until we are back in time
            while self.video_viz.camera.get(cv2.CAP_PROP_POS_MSEC) < (timeit.default_timer() - start_time) * 1000:
                self.video_viz.camera.grab() # skip frames

            # Run the video visualization, it returns false at the end of the video
            # thus we kill the visualization at that point
            if not self.video_viz.update(self.current_state):
                print "End of video, killing viz"
                break

            # run our other visualizations
            self.radar_viz.update(self.current_state)
            self.dsrc_viz.update(self.current_state)

            # Calculate how long to delay before next frame
            single_frame_duration = float(1000)/self.video_viz.fps
            vis_drawing_time = (timeit.default_timer() - start_frame_time) * 1000
            delay_before_next_frame = single_frame_duration - vis_drawing_time + delay_remainder

            # if the amount of time we are supposed to delay is < 1ms then drop
            # frames until our delay is possible. Basically if it took us more than
            # single_frame_duration to draw the visualization on the current frame
            # then we need to drop frames subsequent frames until we have a delay
            # time > 0 (we are "catching up" with the video's natural frame rate)
            frame_count = 1
            while delay_before_next_frame < 1:
                delay_before_next_frame = delay_before_next_frame + single_frame_duration
                self.video_viz.camera.grab() # Get the next frame (aka skip it)

            # Keep track of the fractional part of our delay as a remainder so we don't
            # lose precision over time (if we are supposed to delay 1.5ms then 1.5ms we want
            # to delay 1ms then 2ms so the total time will still be 3ms, we can't delay fractional
            # milliseconds unfortunately)
            delay_remainder = int(delay_before_next_frame) - delay_before_next_frame

            # Wait until it is time to display the next frame
            cv2.waitKey(int(delay_before_next_frame))
