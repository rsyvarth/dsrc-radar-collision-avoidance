"""
convert_grayscale.py

Takes in a filepath from the command line and plays the video in grayscale
using opencv.

"""

import numpy as np
import cv2
import sys

def main(videopath):
    """ Coverts a movie file to grayscale and plays it.

    :videopath: path to the video you would like to play
    :returns: nothing

    """
    cap = cv2.VideoCapture(videopath)

    while(cap.isOpened()):
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv[1])
