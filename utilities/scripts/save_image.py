#!/usr/bin/env python3

import sys

import rospy
import cv2 as cv
import numpy as np
import sensor_msgs.msg

class ImageSaver():

    def __init__(self, filename):
        self._filename = filename
        self._save_image = False
        self._saved_image = False
        self._image_counter = 1
        rospy.init_node("image_saver", anonymous=False)
        rospy.Subscriber("/drone/out/image_rect_color", sensor_msgs.msg.Image, self._image_cb)

    def start(self):
        while not rospy.is_shutdown():

            ans = input("Press enter to save current image (\"exit\" to quit) ")
            if ans.lower() == "exit":
                sys.exit(0)

            self._save_image = True

    def _image_cb(self, msg: sensor_msgs.msg.Image):
        if self._save_image:
            print("Saving image")
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            cv.imwrite(f"{self._filename}_{self._image_counter}.png", img)
            self._save_image = False
            self._image_counter += 1

def main():
    args = sys.argv

    if len(args) != 2:
        print("Usage: save_image <filename_prefix>")
        sys.exit(1)

    saver = ImageSaver(args[1])
    saver.start()

if __name__ == "__main__":
    main()