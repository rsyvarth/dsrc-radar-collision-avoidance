"""
drawing_over_movie.py

Draws a green box around the largest red object in the movie where the filepath
is passed in through the command line.

"""

import numpy as np
import cv2
import sys

def main(videopath):
    """ Draws a green box around largest red obj in movie

    :videopath: filepath to the video
    :returns: nothing

    """
    cap = cv2.VideoCapture(videopath)

    while(cap.isOpened()):
        ret, frame = cap.read()

        # find the red color game in the frame
        upper = np.array([65, 65, 255])
        lower = np.array([0, 0, 200])
        mask = cv2.inRange(frame, lower, upper)

        # find contours in the masked frame and keep the largest one
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea)

        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.05 * peri, True)

        # draw a green bounding box surrounding the red game
        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 4)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv[1])

