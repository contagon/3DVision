import cv2
import os
import argparse
from time import time

"""
This file was used to save images. The files task2.py and task3.py were used to do the actual processing.
"""
# Displays countdown and returns last number upon finishing
def countdown(wait, i=None):
    prev = time()
    while wait > 0:
        # take image
        ret, image_l = left.read()
        ret, image_r = right.read()

        # put countdown on it
        # image, text, location, font, font_scale, color, line_thickness
        cv2.putText(image_r, str(round(wait,1)), (0, 100), font, 4, (0, 0, 255), 2)
        if i is not None:
            cv2.putText(image_r, str(i), (0, 175), font, 2, (0, 0, 255), 2)

        # display
        cv2.imshow('calibrate', cv2.hconcat([image_l, image_r]))
        if cv2.waitKey(10) == ord('q'):
            break

        # tick countdown
        curr = time()
        if curr - prev >= 0.1:
            prev = curr
            wait = wait - 0.1

    ret, image_l = left.read()
    ret, image_r = right.read()
    return image_l, image_r      

if __name__ == "__main__":
    # Parse through arguments
    parser = argparse.ArgumentParser(description="Camera Calibrater")
    parser.add_argument("-o", "--outfolder", type=str, default="images", help="Location to save images to")
    parser.add_argument("-n", "--num", type=int, default=40, help="Number of images to take")
    parser.add_argument("--start_wait", type=float, default=5, help="Seconds to get setup after starting")
    parser.add_argument("--wait", type=float, default=1, help="Seconds to wait between each image")
    parser.add_argument("-l", "--cam_index_l", type=int, default=0, help="Camera index")
    parser.add_argument("-r", "--cam_index_r", type=int, default=1, help="Camera index")
    args = vars(parser.parse_args())

    os.makedirs(args['outfolder'], exist_ok=True)

    # set a few globals
    left  = cv2.VideoCapture(args['cam_index_l'])
    right = cv2.VideoCapture(args['cam_index_r'])
    font = cv2.FONT_HERSHEY_SIMPLEX 

    # iterate through all images
    for i in range(args['num']):
        if i == 0:
            image_l, image_r = countdown(args['start_wait'], i+1)
        else:
            image_l, image_r = countdown(args['wait'], i+1)

        # save image
        cv2.imwrite(os.path.join(args['outfolder'], f'L_{i:03d}.jpg'), image_l)
        cv2.imwrite(os.path.join(args['outfolder'], f'R_{i:03d}.jpg'), image_r)

    # When everything done, release the capture
    left.release()
    right.release()
    cv2.destroyAllWindows()