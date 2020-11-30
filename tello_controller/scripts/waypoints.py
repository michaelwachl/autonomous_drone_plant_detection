import rospy      
import math               
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion, PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

class WayPoints(object):
	"""This class creates a circle path"""
	# initialization of circle path
	def __init__(self):
		self.path_msg = Path()						# init path
		self.path_msg.header.seq = 0
		self.path_msg.header.frame_id = 'world'
		self.waypoint_1 = PoseStamped()				# Pose added to path
		self.waypoint_2 = PoseStamped()				# Pose added to path
		self.waypoint_3 = PoseStamped()				# Pose added to path
		self.waypoint_4 = PoseStamped()  			# Pose added to path
		self.waypoint_5 = PoseStamped()  			# Pose added to path
		self.waypoint_6 = PoseStamped()  			# Pose added to path
		self.waypoint_7 = PoseStamped()  			# Pose added to path

		self.waypoint_1.header.seq = 0
		self.waypoint_1.header.frame_id = "world"
		self.waypoint_1.pose.position.x = 0.0
		self.waypoint_1.pose.position.z = 1.8
		#self.waypoint.pose.orientation.z = 0.7
		#self.waypoint.pose.orientation.w = 0.7

		self.waypoint_2.header.seq = 1
		self.waypoint_2.header.frame_id = "world"
		self.waypoint_2.pose.position.x = 4.7
		self.waypoint_2.pose.position.y = 0.0
		self.waypoint_2.pose.position.z = 1.85
		self.waypoint_2.pose.orientation.z = -0.7
		self.waypoint_2.pose.orientation.w = 0.7

		self.waypoint_3.header.seq = 2
		self.waypoint_3.header.frame_id = "world"
		self.waypoint_3.pose.position.x = 4.7
		self.waypoint_3.pose.position.y = -4.5
		self.waypoint_3.pose.position.z = 1.85
		self.waypoint_3.pose.orientation.z = 0.0
		self.waypoint_3.pose.orientation.w = 0.0
		self.waypoint_3.pose.orientation.y = 1.0

		self.waypoint_4.header.seq = 3
		self.waypoint_4.header.frame_id = "world"
		self.waypoint_4.pose.position.x = 1.1
		self.waypoint_4.pose.position.y = -4.5
		self.waypoint_4.pose.position.z = 1.85
		self.waypoint_4.pose.orientation.z = 0.7
		self.waypoint_4.pose.orientation.w = 0.7

		self.waypoint_5.header.seq = 4
		self.waypoint_5.header.frame_id = "world"
		self.waypoint_5.pose.position.x = 1.1
		self.waypoint_5.pose.position.y = 0.8
		self.waypoint_5.pose.position.z = 1.85
		#self.waypoint_5.pose.orientation.z = 0.7
		#self.waypoint_5.pose.orientation.w = 0.7

		self.waypoint_6.header.seq = 5
		self.waypoint_6.header.frame_id = "world"
		self.waypoint_6.pose.position.x = 0.0
		self.waypoint_6.pose.position.y = 0.8
		self.waypoint_6.pose.position.z = 0.5
		#self.waypoint_6.pose.orientation.z = 0.7
		#self.waypoint_6.pose.orientation.w = 0.7

		self.waypoint_7.header.seq = 5
		self.waypoint_7.header.frame_id = "world"
		self.waypoint_7.pose.position.x = 0.0
		self.waypoint_7.pose.position.y = 0.8
		self.waypoint_7.pose.position.z = 0.5

		self.path_msg.poses.append(deepcopy(self.waypoint_1))
		self.path_msg.poses.append(deepcopy(self.waypoint_2))
		self.path_msg.poses.append(deepcopy(self.waypoint_3))
		self.path_msg.poses.append(deepcopy(self.waypoint_4))
		self.path_msg.poses.append(deepcopy(self.waypoint_5))
		self.path_msg.poses.append(deepcopy(self.waypoint_6))
		self.path_msg.poses.append(deepcopy(self.waypoint_7))

	# get path array
	def get_path(self):
		self.path_msg.header.stamp = rospy.Time.now()
		return self.path_msg
