#!/usr/bin/env python

""" cam_processing.py - Version 0.1
    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).


"""
# to do: argument parser, img size input

from threading import Thread
import time

import roslib;

roslib.load_manifest('cam_processing')
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


SHOW = False
FILTER_FRAMES = True
REDUCE_NOISE = True
RECORD = False

class CamProcessing():
    def __init__(self):
        self.node_name = "video_processing"

        rospy.init_node(self.node_name, anonymous=True)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Relevant Image parameter
        self.image_width = int(rospy.get_param('image_width', 640))
        self.image_height = int(rospy.get_param('image_height', 480))

        # Create empty image
        self.img_prev = np.zeros([self.image_height, self.image_width, 3], dtype=np.uint8)
        self.processed_frame = np.zeros([self.image_height, self.image_width, 3], dtype=np.uint8)

        # Subscribe and publish to the camera image and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/image_raw_denoised", Image)

        if RECORD:
            self.out = cv2.VideoWriter("video.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25, (640, 480))

        rospy.loginfo("Waiting for image topics...")

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        # Write the frame into the file 'output.avi'
        if RECORD:
            self.out.write(frame)

        # Filter faulty frames
        if FILTER_FRAMES:
            if self.is_valid(frame):
                self.processed_frame = frame
            else:
                rospy.loginfo("Corrupted frame dropped")

        # Process the frame using the process_image() function
        if REDUCE_NOISE:
            self.processed_frame = self.process_image(self.processed_frame)
            self.img_prev = self.processed_frame

        # concatenate image Horizontally
        if SHOW:
            img_concatenate = np.concatenate((frame, self.processed_frame), axis=1)
            # Display the images.
            cv2.imshow(self.cv_window_name, img_concatenate)

            # Process any keyboard commands
            self.keystroke = cv2.waitKey(5)
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    # The user has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")

    def is_valid(self, image):
        # Convert image to HSV color space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Calculate histogram of saturation channel
        s = cv2.calcHist([image], [1], None, [256], [0, 256])

        # Calculate percentage of pixels with saturation >= p
        p = 0.05
        s_perc = np.sum(s[int(p * 255):-1]) / np.prod(image.shape[0:2])

        # Percentage threshold; above: valid image, below: noise
        s_thr = 0.5
        return s_perc > s_thr


    def process_image(self, frame):
        # Denoising
        # frame_denoised = cv2.fastNlMeansDenoisingColored(frame, None, 10, 10, 7, 21)  # very slow!

        # Blur
        #frame_denoised = cv2.blur(frame, (3, 3))
        frame_denoised = cv2.medianBlur(frame, 3)

        # overlapping frames
        # frame_denoised = (frame+self.img_prev)/2

        # Gray image
        # grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur the image
        # grey = cv2.blur(grey, (7, 7))

        # Compute edges using the Canny edge filter
        # edges = cv2.Canny(grey, 15.0, 30.0)

        #publish denoied image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_denoised, "bgr8"))
        except CvBridgeError as e:
            print(e)

        return frame_denoised

    def process_depth_image(self, frame):
        # Just return the raw image
        return frame

    def cleanup(self):
        print("Shutting down vision node.")
        # self.out.release()
        cv2.destroyAllWindows()


def main(args):
    try:
        CamProcessing()
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)