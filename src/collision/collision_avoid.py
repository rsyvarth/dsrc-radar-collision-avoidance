from multiprocessing import Process
import logging
import numpy as np
import cv2
from . import CollisionConstants
from Queue import Empty

class CollisionAvoidance(Process):
    """ Takes combined data and displays predicted collisions.

    This package is included mainly for the purpose of demonstration.
    It takes the information provided by our Combiner and calculates any predicted
    collisions displaying warnings in a simple UI.
    """

    def __init__(self, queue):
        """Setup the CA class, just empty state for now"""
        Process.__init__(self)
        self.queue = queue
        self.current_state = None
        self.constants = CollisionConstants()

    def run(self):
        print "Starting visualizer"

        while True:
            try:
                self.current_state = self.queue.get(timeout=0.5)
                # print self.current_state
                print 'got data'
            except Empty:
                print 'no data'
                pass

    def new_data_handler(self, data):
        """Called whenever new data arrives from the Combiner"""
        logging.info("Collision_avoid::new_data_handler() %s" % data)
        self.current_state = data

        self.analyze_state()

    def analyze_state(self):
        """Eventually this will do cool math to detect collisions"""
        pass

    def display(self, videofile, distance_from_radar, focal_length, sensor_size):
        """videopath: path to video that we will be using for image processing"""
        track_objects = self.current_state["entities"]
        """
        To draw on the object:
        1. Get track info from track_objects: track_width, track_range, track_angle?
        2. Get info for the image: total image size (pixels)
        3. Calculate where in the image the object would be: camera vs. radar position, object sizes in terms of pixels at different positions, anything else?
        4. Based on projected size (height), calculate top and bottom pixel values, as well as middle point
        5. Calculate top left and bottom right corners of the track object
        6. Draw the appropriate rectangle
        """
        camera = cv2.VideoCapture(videofile)
        while True:
            # Loop until the video is done
            ret, img = camera.read()
            if not ret:
                # We have reached the end of the video
                break

            for track in track_objects:
                # Step 1
                track_number = track["track_number"]
                track_width = track[track_number+"_track_width"]
                track_range = track[track_number+"_track_range"]
                track_angle = track[track_number+"_track_angle"]

                # Step 2
                img_height, img_width = img.shape[:2]

                # Step 3
                # Formula using: obj_width(pixels) = (focal length(mm) * obj width(mm) * img_width(pixels)) / (track_range(mm) * sensor width(mm)?)
                # Step 4
                # Step 5
                # Step 6
