"""
drawing.py

Draws a green box around the largest red object in an image where the filepath
is passed in through the command line.

"""

import numpy as np
import cv2
import sys

def main(imgpath):
    """ Draws a green box around largest red obj in image

    :imgpath: filepath to the image
    :returns: nothing

    """
    # load the games image
    image = cv2.imread(imgpath)

    # find the red color game in the image
    upper = np.array([65, 65, 255])
    lower = np.array([0, 0, 200])
    mask = cv2.inRange(image, lower, upper)

    # find contours in the masked image and keep the largest one
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    # approximate the contour
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.05 * peri, True)

    # draw a green bounding box surrounding the red game
    cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
    cv2.imshow("Image", image)
    cv2.waitKey(0)

if __name__ == "__main__":
    main(sys.argv[1])

