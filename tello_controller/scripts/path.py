import rospy      
import math               
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy
from math import pi

class PathObject(object):
	"""This class creates a circle path"""
	# initialization of circle path
	def __init__(self, current_pose, center_pose, segments, radius, height):
		self.starting_point = current_pose			# current position of drone
		self.center	= center_pose					# center point as PoseStamped
		self.segments = segments                	# number of segments
		self.radius = radius                    	# weight of integral control
		self.height = height
		self.path_msg = Path()						# init path
		self.waypoint = PoseStamped()				# Pose added to path
		self.first_yaw = 0.0
		self.first_pose = PoseStamped()
		self.dx = 0.0
		self.dy = 0.0
		self.dz = 0.0

	def normalize_angle_positive(self, angle):
		""" Normalizes the angle to be 0 to 2*pi
            It takes and returns radians. """
		return angle % (2.0 * pi)

	def normalize_angle(self, angle):
		"""
		Normalizes the angle to be -pi to +pi
        It takes and returns radians.
        """
		a = self.normalize_angle_positive(angle)
		if a > pi:
			a -= 2.0 * pi
		return a

	def calculate_first_yaw(self):
		self.dx = self.center.pose.position.x - self.starting_point.pose.position.x
		self.dy = self.center.pose.position.y - self.starting_point.pose.position.y
		self.dz = self.center.pose.position.z - self.starting_point.pose.position.z
		self.first_yaw = math.atan2(self.dy, self.dx)
		print('First yaw: ', self.first_yaw)

	def calculate_first_pose(self):
		self.first_pose = deepcopy(self.center)
		self.first_pose.pose.position.x += self.radius * math.cos(self.first_yaw - math.pi)
		self.first_pose.pose.position.y += self.radius * math.sin(self.first_yaw - math.pi)
		self.first_pose.pose.position.z = self.height

	#  calculate all points
	def calculate_path(self):
		starting_orientation_list = [self.starting_point.pose.orientation.x, self.starting_point.pose.orientation.y,
									 self.starting_point.pose.orientation.z, self.starting_point.pose.orientation.w]
		starting_euler_angles = euler_from_quaternion(starting_orientation_list)
		starting_yaw = starting_euler_angles[2]
		segment_angle = abs(2*pi/self.segments)

		# x0 = self.radius*math.cos(starting_yaw) + self.starting_point.pose.position.x
		# y0 = self.radius*math.sin(starting_yaw) + self.starting_point.pose.position.y
		increment_yaw = self.first_yaw - math.pi
		#increment_angles = starting_euler_angles
		# print(increment_angles)
		self.waypoint = self.first_pose 		# same z, roll and pitch as starting position
		self.waypoint.header.stamp = rospy.Time.now()
		self.path_msg.poses.append(deepcopy(self.waypoint))
		for i in range(self.segments):
			increment_yaw += segment_angle
			increment_yaw = self.normalize_angle(increment_yaw)
			self.waypoint.pose.position.x = self.radius*math.cos(increment_yaw) + self.center.pose.position.x
			self.waypoint.pose.position.y = self.radius*math.sin(increment_yaw) + self.center.pose.position.y
			self.waypoint.pose.position.z = self.height

			q = quaternion_from_euler(0.0, 0.0, (increment_yaw - math.pi))
			self.waypoint.pose.orientation = Quaternion(*q)

			self.waypoint.header.seq += 1
			self.waypoint.header.stamp = rospy.Time.now()
			self.path_msg.poses.append(deepcopy(self.waypoint))

		home_pose = PoseStamped()
		home_pose.header.seq = self.waypoint.header.seq +1
		home_pose.header.frame_id = "world"
		home_pose.pose.position.z = 0.5

		self.path_msg.poses.append(deepcopy(home_pose))
		"""
		self.waypoint = self.starting_point
		self.waypoint.pose.position.x = 1.0
		self.waypoint.header.seq += 1
		self.path_msg.poses.append(deepcopy(self.waypoint))
		self.waypoint.pose.position.x = 2.0
		self.waypoint.header.seq += 1
		self.path_msg.poses.append(deepcopy(self.waypoint))
		"""


	# get path array
	def get_path(self):
		self.calculate_first_yaw()
		self.calculate_first_pose()
		self.calculate_path()
		self.path_msg.header.seq += 1
		self.path_msg.header.frame_id = 'world'
		self.path_msg.header.stamp = rospy.Time.now()
		return self.path_msg
