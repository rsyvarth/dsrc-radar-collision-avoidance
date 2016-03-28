"""
drawing_over_movie.py

Draws a green box around the largest red object in the movie where the filepath
is passed in through the command line.

"""

import numpy as np
import cv2
import time as t

IMG_WIDTH = 512
IMG_HEIGHT = 512
IMG_CHANNELS = 3
def main():

    # draw a black image on the screen
    img = np.zeros((IMG_WIDTH, IMG_HEIGHT, IMG_CHANNELS), np.uint8)
    cv2.imshow("Circles!", img)

    for px_width in range(IMG_WIDTH - 1):
        # draw a 15 px black circle behind and a white circle in front
        cv2.circle(img,(px_width, px_width), 15, (0, 0, 0), -1)
        cv2.circle(img,(px_width + 1, px_width + 1), 15, (255, 255, 255), -1)
        cv2.imshow("Circles!", img)

        # 33 ms delay in between drawing
        cv2.waitKey(33)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

