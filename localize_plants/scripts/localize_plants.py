#!/usr/bin/env python
import sys
import rospy
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion, PoseArray
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import String, Empty, Int8, Header
from sensor_msgs.msg import Image, ChannelFloat32
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, ObjectCount
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from copy import deepcopy
import math
import numpy as np
import cv2

class PlantLocalizer(object):
    """Controller to fly Tello in 3D coordinates"""
    def __init__(self):
        # parameter for calculation
        self.min_line_length = 0.5
        self.max_line_length = 7.0
        self.line_step = 0.02

        # initialize ROS node
        rospy.init_node('localize_plant_node', anonymous=True)
        # get node name
        self.ns_node = 'localize_plant/'   # rospy.get_namespace()

        # define variables
        self.current_pose = PoseStamped()
        self.current_yaw = 0.0
        self.slam_control = False
        self.first_pointcould = PointCloud()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        self.first_pointcould.header = header
        self.points = None
        self.first_pointcould.header = header
        self.second_pointcould = PointCloud()
        self.second_pointcould.header.frame_id = 'map'
        self.line_point = Point32()
        self.ns_tello = 'tello/'
        self.ns_darknet = 'darknet_ros/'
        self.ns_plant_loc = 'plant_localizer/'

        # 90 deg around y-axis
        self.pos_listener = TransformListener()
        self.rot_camera= np.array([[0.0, 0.0, -1.0],
                                   [0.0, 1.0, 0.0],
                                   [1.0, 0.0, 0.0]],dtype=np.float32)

        self.slam_pose_topic = 'orb_slam_2_ros/pose'

        self.msg_boundingBoxes = BoundingBoxes()

        # ROS Subs und Pubs initialize
        rospy.Subscriber(self.ns_tello+'odom', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber(self.ns_darknet+'found_object', ObjectCount, self.cb_objects, queue_size=1)
        rospy.Subscriber(self.ns_darknet+'bounding_boxes', BoundingBoxes, self.cb_bounding_boxes, queue_size=1)

        self.pub_first_pointcloud = rospy.Publisher(self.ns_plant_loc + "/label_1_pointcloud", PointCloud, queue_size=10)
        self.pub_second_pointcloud = rospy.Publisher(self.ns_plant_loc + "/label_2_pointcloud", PointCloud, queue_size=10)

        # container for
        plant_locations= []

        # camera matrix for tello single cam
        """              f  x  0 
         camera_matrix = 0  f  y
                         cx cy 1       
        
        f: focal length
        c: optical center     
        """
        self.camera_matrix = np.array([[924.873180, 0.000000  , 486.997346],
                                        [0.000000  , 923.504522, 364.308527],
                                        [0.000000  , 0.000000  , 1.000000]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.034749, 0.071514, 0.000363, 0.003131, 0.000000], dtype=np.float32)

        # invert camera matrix for projetion to 3D coordinates
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)


        test = np.zeros((3, 1, 2), dtype=np.float32)
        test[0,:] = [486.99, 364.30]
        test[1, :] = [0.0, 0.0]
        test[2, :] = [600.0, 50.0]
        print(test)
        test = self.undistort_and_project(test)
        print(test)
        print('print init done')
        self.pub_first_pointcloud.publish(self.first_pointcould)
        # keep process alive
        rospy.spin()

    def calculate_line(self, point, label):
        element_number = int((self.max_line_length - self.min_line_length)/self.line_step)
        arr = np.empty((element_number,3),  dtype=np.float32)
        i= 0
        if label == 0:
            for scale in np.arange(self.min_line_length, self.max_line_length, self.line_step):
                # arr[i,:] = scale*point
                self.line_point.x = scale * point[0]
                self.line_point.y = scale * point[1]
                self.line_point.z = scale * point[2]
                self.first_pointcould.points.append(deepcopy(self.line_point))
                # i += 1
        elif label == 1:
            for scale in np.arange(self.min_line_length, self.max_line_length, self.line_step):

                self.second_pointcould.points.append(deepcopy(self.line_point))

        print(arr)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        # self.points = pc2.create_cloud_xyz32(header, arr)

    def transform_to_map(self):
        """try to transfrom from current camera frame to map frame
        """
        try:
            (trans, rot) = self.pos_listener.lookupTransform('camera_frame', 'map', rospy.Time(0))
            print('trans: ', trans, ' | rot: ', rot)
            """
            euler_angles = euler_from_quaternion(tello_rot)
            self.ui.label_internal_x.setText('%.2f' % tello_trans[0])
            self.ui.label_internal_y.setText('%.2f' % tello_trans[1])
            self.ui.label_internal_z.setText('%.2f' % tello_trans[2])
            self.ui.label_internal_yaw.setText('%.2f' % euler_angles[2])
            """
        except (LookupException, ConnectivityException, ExtrapolationException):
            print('No Position Data available')


    def undistort_and_project(self, uv_pts):
        """Converts 2D pixel images points to the spherical coordinate system.
        First pixel coordinates are undistorted with calibration matrix and distortion coefficiants.
        Inverse matrix of Intrinsics of camera are taken into account to transform into cartesian vertices and
        then converted to spherical coordinates.
        """
        # (u, v) is the pixel input point
        num_pts = uv_pts.size / 2
        uv_pts.shape = (num_pts, 1, 2)

        # do the actual undistort
        uv_pts = cv2.undistortPoints(uv_pts, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)

        # transform to homogeneus coordiantes, add 1
        pts_h = cv2.convertPointsToHomogeneous(np.float32(uv_pts))
        pts_h.shape = (num_pts, 3)
        print(pts_h)
        xyz_pts = np.zeros((num_pts, 3), dtype=np.float32)
        for i in range(num_pts):
            xyz_pts[i] = self.camera_matrix_inv.dot(pts_h[i])
            # rotate around y-axis to 

        # self.calculate_and_add_lines(xyz_pts[0,:],0)
        return xyz_pts

    def calculate_center_points(self, bounding_boxes):
        center_points = []
        for box in bounding_boxes:
            #   --> x
            #   |
            #   v  y
            center_x = box.xmin + int((box.xmax-box.xmin)/2)
            center_y = box.ymin + int((box.ymax-box.ymin)/2)
            center_point = [center_x, center_y, box.id]
            center_points.append(center_point)

        print('center_points', center_point)
        return center_points

    def calculate_XYZ(self, u, v):
        """ Solve from image pixels to find world points """
        uv_1 = np.array([[u, v, 1]], dtype=np.float32)
        uv_1 = uv_1.T

    def cb_objects(self, msg):
        object_number = msg.count

    def cb_bounding_boxes(self, msg):
        print('bounding box received')
        if len(msg.bounding_boxes)>0:
            center_points = self.calculate_center_points(msg.bounding_boxes)
            projected_points = self.undistort_and_project(center_points)

    def cb_odom(self, msg):
        # print('odom callback')
        if self.slam_control == False:
            self.current_pose = msg.pose.pose
            # inverse yaw
            self.current_pose.orientation.w = -self.current_pose.orientation.w
            current_orientation = self.current_pose.orientation
            current_orientation_list = [current_orientation.x, current_orientation.y, current_orientation.z,
                                       current_orientation.w]
            current_angles = euler_from_quaternion(current_orientation_list)
            self.current_yaw = current_angles[2]

    def cb_slam_position(self, msg):
        if self.slam_control:
            self.slam_pos = msg.pose.position
            self.slam_quaternion = msg.pose.orientation
            self.slam_orientation_rad_list = euler_from_quaternion(
                [self.slam_quaternion.x, self.slam_quaternion.y, self.slam_quaternion.z, self.slam_quaternion.w])
            self.slam_orientation_rad = Point(self.slam_orientation_rad_list[0], self.slam_orientation_rad_list[1],
                                              self.slam_orientation_rad_list[2])


if __name__ == '__main__':
    # Create controller
    # start up the controller node and run until shutdown by interrupt
    try:
        controller = PlantLocalizer()
    except rospy.ROSInterruptException:
        rospy.loginfo("Plant Localizer node terminated.")
