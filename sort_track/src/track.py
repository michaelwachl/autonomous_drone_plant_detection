#!/usr/bin/env python

"""
ROS node to track objects using SORT TRACKER and YOLOv4 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculated tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
No delay here
"""


import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sort import sort 
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sort_track.msg import TrackedBoundingBoxes


color_list = [(255, 70, 70), (70, 70, 255), (70, 255, 70), (150, 255, 100)]

class YoloTracker(object):
	def __init__(self):

		# Gets the necessary parameters from .yaml file
		self.loop_rate = rospy.get_param("~loop_rate")
		self.show_image = rospy.get_param("~show_image")
		self.camera_topic = rospy.get_param("~camera_topic")
		self.detection_topic = rospy.get_param("~detection_topic")
		self.tracker_topic = rospy.get_param('~tracker_topic')
		self.cost_threshold = rospy.get_param('~cost_threhold')
		self.min_hits = rospy.get_param('~min_hits')
		self.max_age = rospy.get_param('~max_age')

		global tracker
		tracker = sort.Sort(max_age=self.max_age, min_hits=self.min_hits)
		# Subscribe to image topic
		self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.callback_image)
		# Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
		self.sub_detection = rospy.Subscriber(self.detection_topic, BoundingBoxes, self.callback_det)
		# Publish results of object tracking
		self.pub_trackers = rospy.Publisher(self.tracker_topic, TrackedBoundingBoxes, queue_size=10)


	def callback_det(self, data):
		global detections
		global trackers
		global track
		detections = []
		trackers = []
		track = []
		ids = []
		msg.bounding_boxes = data.bounding_boxes
		for box in data.bounding_boxes:
			detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, round(box.probability, 2), box.id]))
		detections = np.array(detections)
		#Call the tracker
		trackers = tracker.update(detections)
		trackers = np.array(trackers, dtype='int')
		track = trackers
		#get id's
		for tra in track:
			ids.append(tra[4])

		msg.track_ids = ids


	def callback_image(self, data):

		if self.show_image:
			#Display Image
			bridge = CvBridge()
			cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
			#TO DO: FIND BETTER AND MORE ACCURATE WAY TO SHOW BOUNDING BOXES!!
			#Detection bounding box
			if 'detections' in globals():
				for det in detections:
					cv2.rectangle(cv_rgb, (int(det[0]), int(det[1])), (int(det[2]), int(det[3])), color_list[int(det[5])], 2)
					cv2.putText(cv_rgb, "yolo", (int(det[0]), int(det[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_list[int(det[5])], lineType=cv2.LINE_AA)
			#Tracker bounding box
			if 'track' in globals():
				for tra in track:
					cv2.rectangle(cv_rgb, (tra[0], tra[1]), (tra[2], tra[3]), (255, 255, 255), 1)
					cv2.putText(cv_rgb , str(tra[4]), (tra[2], tra[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
					cv2.imshow("YOLO+SORT", cv_rgb)
			cv2.waitKey(3)


def main():
	# Initialize ROS node
	rospy.init_node('sort_tracker', anonymous=False)

	global tracker
	global msg
	msg = TrackedBoundingBoxes()

	# Create tracker object
	tracker_yolo = YoloTracker()
	rate = rospy.Rate(tracker_yolo.loop_rate)

	while not rospy.is_shutdown():
		# Publish tracket objects
		tracker_yolo.pub_trackers.publish(msg)
		print(msg)
		rate.sleep()
		#rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
