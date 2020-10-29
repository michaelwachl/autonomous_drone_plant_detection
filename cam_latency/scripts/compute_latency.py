#!/usr/bin/env python
import sys
import time

import rospy, rostopic
import numpy as np
import cv2

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from sensor_msgs.msg import Image, CompressedImage, Imu
from tello_driver.msg import TelloStatus
from cv_bridge import CvBridge, CvBridgeError

from topic_info import TopicInfo


# Globals
SAMPLES = 1000
SHOW_PLOT = False
line = None
diff_ms = []
diff_comp_ms = []

bw_imu = []
bw_img_comp = []
bw_status = []

hz_imu = []
hz_img_comp = []
hz_status = []

# Frame counters
i = 0
times = []

i_comp = 0
times_comp = []


class TopicMonitoring(object):
	def __init__(self):
		rospy.init_node('tello_latency', anonymous=False)

		# Open an interactive plot
		#self.fig, self.ax = plt.subplots()
		#self.plt.title('Transmission latency [ms]')
		#self.ax.set_ylim(0,10)

		# Subscribe to topics
		self.img_dec = rospy.Subscriber('/tello/camera/image_raw', Image, self.handle_img)
		self.img_comp = rospy.Subscriber('/tello/image_raw/h264', CompressedImage, self.handle_comp_img)
		# publish
		self.pub_img_center = rospy.Publisher('/tello/camera/image_raw_center', Image)

		self.bridge = CvBridge()

		# Topic monitoring
		#self.topic_info_image = TopicInfo('/tello/camera/image_raw', Image)
		self.topic_info_image = TopicInfo('/tello/camera/image_raw', 'sensor_msgs/Image')
		self.topic_info_image_comp = TopicInfo('/tello/image_raw/h264', 'sensor_msgs/CompressedImage')
		self.topic_info_imu = TopicInfo('/tello/imu', 'sensor_msgs/Imu')
		self.topic_info_status = TopicInfo('/tello/status', 'tello_driver/TelloStatus')
		self.topic_infos = [self.topic_info_image, self.topic_info_image_comp, self.topic_info_imu, self.topic_info_status]

		# Bool to toggle pausing
		self.pause = False

	def start_monitoring(self):
		self.topic_info_image.start_monitoring()
		self.topic_info_image_comp.start_monitoring()
		self.topic_info_imu.start_monitoring()
		self.topic_info_status.start_monitoring()

	def update_topics_data(self):
		for topic_info in self.topic_infos:
			if topic_info.monitoring:
				# update rate
				rate, _, _, _ = topic_info.get_hz()
				rate_text = '%1.2f' % rate if rate != None else 'unknown'

				# update bandwidth
				bytes_per_s, _, _, _ = topic_info.get_bw()
				if bytes_per_s is None:
					bandwidth_text = 'unknown'
				elif bytes_per_s < 1000:
					bandwidth_text = '%.2fB/s' % bytes_per_s
				elif bytes_per_s < 1000000:
					bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
				else:
					bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)

			else:
				rate_text = ''
				bytes_per_s = None
				bandwidth_text = ''
				value_text = 'not monitored' if topic_info.error is None else topic_info.error

			print('Name: ', topic_info._topic_name, ' Rate: ', rate_text, ' BW: ', bandwidth_text)


	def handle_img(self, msg):
		global i
		global times
		now = rospy.Time.now().to_sec()

		# Calculate difference in recvd vs trans time
		msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
		diff = rospy.Time.now().to_sec() - msg_time.to_sec()
		times.append(diff)
		i += 1
		diff_ms.append(diff*1e3)

		self.add_center_line(msg)

		# print('Latency image: ', diff_ms[-1])


	def add_center_line(self, msg):

		cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		width, height = 960, 720
		x1, y1 = 0, 0
		x2, y2 = 200, 400   

		line_thickness = 2
		cv2.line(cv_image, (1, 363), (959, 363), (100, 255, 50), thickness=line_thickness)
		cv2.line(cv_image, (487, 1), (487, 719), (100, 255, 50), thickness=line_thickness)

		img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
		self.pub_img_center.publish(img_msg)


	def handle_comp_img(self, msg):
		global i_comp
		global times_comp
		now = rospy.Time.now().to_sec()

		# Calculate difference in recvd vs trans time
		msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
		diff = rospy.Time.now().to_sec() - msg_time.to_sec()
		times_comp.append(diff)
		i_comp += 1
		diff_comp_ms.append(diff*1e3)

		# print('Latency image compressed: ', diff_comp_ms[-1])


	def animate(i):
		global diff_ms

		if len(diff_ms) > 0:
			xdata = line.get_xdata()
			ydata = line.get_ydata()

			N = len(xdata)+1

			xdata.append(N)
			ydata.append(diff_ms[-1])

			line.set_data(xdata, ydata)

			ax = plt.gca()
			ax.set_xlim(0, N)

			diff_ms = []

		return [line]

	def _init():
		line.set_data([], [])
		return [line]

def main():
	monitor = TopicMonitoring()
	monitor.start_monitoring()

	try:
		rate = rospy.Rate(15)
		while not rospy.is_shutdown():
			monitor.update_topics_data()
			now = rospy.Time.now().to_sec()
			# Print out the current time for the camera to record
			sys.stdout.write("\r")
			sys.stdout.write("{:2f}".format(now))
			sys.stdout.flush()
			rate.sleep()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated.")


	if SHOW_PLOT:
		global line
		line = ax.plot([],[], linewidth=0.6, animated=False)[0]
		animate = animation.FuncAnimation(fig, _animate, init_func=_init, frames=None,
								interval=10, blit=True)

		# plt.show acts as rospy.spin()
		plt.show()

	while(True):
		now = rospy.Time.now().to_sec()
		# Print out the current time for the camera to record
		sys.stdout.write("\r")
		sys.stdout.write("{:2f}".format(now))
		sys.stdout.flush()

	rospy.spin()


if __name__ == '__main__':
    main()