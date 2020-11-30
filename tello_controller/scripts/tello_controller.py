#!/usr/bin/env python
import sys
import rospy
import threading
import time
from pid import PID
from path import PathObject
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion, PoseArray
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty, Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from mission_state import MissionState
from controller_log import ControllerLog
from copy import deepcopy
import math
from waypoints import WayPoints


class TelloController(object):
    """Controller to fly Tello in 3D coordinates"""
    def __init__(self):

        # initialize ROS node
        rospy.init_node('tello_controller_node', anonymous=True)
        # get node name
        self.ns_node = 'tello_controller/'   # rospy.get_namespace()

        print(self.ns_node)
        # load/set parameters on parameter server
        tello_frame = rospy.get_param("~frame", "base_link")
        self.control_rate = rospy.get_param("~frequency", 20.0)
        self.stabalize_time = rospy.get_param("~stabalize_time", 5.0)
        self.log_controller = rospy.get_param("~log_controller", True)
        self.show_trail = rospy.get_param("~show_trail", True)
        # create flight PID controllers in X, Y and Z
        self.pid_x = PID("PID_x",
                    rospy.get_param("~PIDs/X/kp", 3.0),
                    rospy.get_param("~PIDs/X/kd", 0.1),
                    rospy.get_param("~PIDs/X/ki", 0),
                    rospy.get_param("~PIDs/X/minOutput", -0.3),
                    rospy.get_param("~PIDs/X/maxOutput", 0.3),
                    rospy.get_param("~PIDs/X/integratorMin", -0.1),
                    rospy.get_param("~PIDs/X/integratorMax", 0.1),
                    False)
        self.pid_y = PID("PID_y",
                    rospy.get_param("~PIDs/Y/kp", 2.0),
                    rospy.get_param("~PIDs/Y/kd", 0.1),
                    rospy.get_param("~PIDs/Y/ki", 0),
                    rospy.get_param("~PIDs/Y/minOutput", -0.3),
                    rospy.get_param("~PIDs/Y/maxOutput", 0.3),
                    rospy.get_param("~PIDs/Y/integratorMin", -0.1),
                    rospy.get_param("~PIDs/Y/integratorMax", 0.1),
                    False)
        self.pid_z = PID("PID_z",
                    rospy.get_param("~PIDs/Z/kp", 3.0),
                    rospy.get_param("~PIDs/Z/kd", 0.1),
                    rospy.get_param("~PIDs/Z/ki", 0),
                    rospy.get_param("~PIDs/Z/minOutput", -0.4),
                    rospy.get_param("~PIDs/Z/maxOutput", 0.4),
                    rospy.get_param("~PIDs/Z/integratorMin", -0.1),
                    rospy.get_param("~PIDs/Z/integratorMax", 0.1),
                    False)
        self.pid_yaw = PID("PID_yaw",
                    rospy.get_param("~PIDs/Yaw/kp", 3.0),
                    rospy.get_param("~PIDs/Yaw/kd", 0.1),
                    rospy.get_param("~PIDs/Yaw/ki", 0),
                    rospy.get_param("~PIDs/Yaw/minOutput", -0.7),
                    rospy.get_param("~PIDs/Yaw/maxOutput", 0.7),
                    rospy.get_param("~PIDs/Yaw/integratorMin", -0.1),
                    rospy.get_param("~PIDs/Yaw/integratorMax", 0.1),
                    True)

        self.target_error_x = rospy.get_param("~PIDs/target_error_x", 0.1)
        self.target_error_y = rospy.get_param("~PIDs/target_error_y", 0.1)
        self.target_error_z = rospy.get_param("~PIDs/target_error_z", 0.1)
        self.target_error_yaw = rospy.get_param("~PIDs/target_error_yaw", 0.17)

        # test path/ point
        test_way_pt = PoseStamped()
        test_way_pt.header.frame_id = "world"
        test_way_pt.header.stamp = rospy.Time.now()
        test_way_pt.pose.position.x = 1.0
        test_way_pt.pose.position.y = 0.0
        test_way_pt.pose.position.z = 1.0
        #test_way_pt.pose.orientation.z = 0.7
        #test_way_pt.pose.orientation.w = 0.7

        test_point = PoseStamped()
        test_center = PoseStamped()
        test_point.header.frame_id = "world"
        test_center.header.frame_id = "world"
        test_center.pose.position.x = 0.0
        test_center.pose.position.y = 0.0
        # test_path = PathObject(test_point, test_center, 20, 1.2, 1.8)      # begin, center, seg, radius, height
        self.path_msg = Path()  # init path
        self.path_msg.header.seq = 0
        self.path_msg.header.frame_id = 'world'
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.poses.append(deepcopy(test_way_pt))

        #self.path_msg = test_path.get_path()
        test_wayp = WayPoints()
        self.path_msg = test_wayp.get_path()
        # init trail
        self.trail_msg = Path()  # init path

        # define variables
        self.current_pose = PoseStamped()
        self.current_yaw = 0.0
        self.target_position = self.path_msg.poses[0]
        self.target = True                              # initialize target sighting to False
        self.last_goal_reached = False
        self.slam_control = False
        self.ns_tello = 'tello/'
        self.hover_position = None
        self.mission_state = MissionState()

        self.slam_pose_topic = '/orb_slam2_mono/pose'
        self.slam_pos = PoseStamped()
        self.real_world_scale = 1.5

        # subscribe to target pose
        self.fly_msg = Twist()
        self.fly_tello_msg = Twist()

        # Initialize the tf listener
        self.listener = TransformListener()

        # ROS Subs und Pubs initialize
        rospy.Subscriber(self.ns_node+'target_pose', PoseStamped, self.cb_target_pose, queue_size=1)
        rospy.Subscriber(self.ns_tello+'odom', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber(self.slam_pose_topic, PoseStamped, self.cb_slam_position, queue_size=1)

        self.pub_velocity = rospy.Publisher(self.ns_tello+'cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher(self.ns_tello+'takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.ns_tello+'land', Empty, queue_size=1)
        self.pub_path = rospy.Publisher(self.ns_node+'target_path', Path, queue_size=1)
        self.pub_trail = rospy.Publisher(self.ns_node + 'trail_path', Path, queue_size=1)
        self.pub_target = rospy.Publisher(self.ns_node+'target_pose', PoseStamped, queue_size=1)
        self.pub_command = rospy.Publisher(self.ns_node+'mission_command', String, queue_size=1)
        self.pub_real_world_scale = rospy.Publisher(self.ns_node+'slam_real_world_scale', Float32, queue_size=1)

        # init logger
        self.log_controller = ControllerLog()

        # create additional thread for conroller
        self.last_iteration_time = time.time()
        self.thread = threading.Thread(target=self.control_iterator, args=())
        self.thread.start()

        # create additional thread for conroller
        self.thread_log = threading.Thread(target=self.log_iterator, args=())
        self.thread_log.start()

        # keep process alive
        rospy.spin()

    def cb_odom(self, msg):
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
        self.slam_pos.pose = msg.pose
        # print("current slam z:%s " % self.slam_pos.pose.position.z)
        self.slam_quaternion = msg.pose.orientation
        self.slam_orientation_rad_list = euler_from_quaternion(
            [self.slam_quaternion.x, self.slam_quaternion.y, self.slam_quaternion.z, self.slam_quaternion.w])
        self.slam_orientation_rad = Point(self.slam_orientation_rad_list[0], self.slam_orientation_rad_list[1],
                                          self.slam_orientation_rad_list[2])

    # This callback function puts the target PoseStamped message into a local variable.
    def cb_target_pose(self, msg):
        self.target_position = msg
        self.target = True
        if self.mission_state.current_state == self.mission_state.GO_HOME:
            # Override path message
            self.path_msg = Path()  # init path
            self.path_msg.header.seq = 0
            self.path_msg.header.frame_id = 'world'
            self.path_msg.poses.append(self.target_position)

    # Callback for Tello state update
    def update_tello_state(self, msg):
        self.tello_state = msg.data

    # This function gets the current transform between the target_frame and the source_frame.
    def get_transform(self):
        # try to look up current position
        try:
            t = self.listener.getLatestCommonTime('odom', 'base_link')
            (tello_trans, tello_rot) = self.listener.lookupTransform('odom', 'base_link', t)    # source, target, time
            return tello_trans, tello_rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("Waiting for transform")

    # This function checks if current position is close enough to target
    def pid_reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

    # This function calls the reset function for all the PID controllers.
    def close_enough(self):
        if not self.path_msg.poses:
            rospy.loginfo("path empty")
            # path is empty
            self.pid_reset()
            # self.mission_state.stop = True
        elif abs(self.pid_x.error) < self.target_error_x and abs(self.pid_y.error) < self.target_error_y and \
                abs(self.pid_z.error) < self.target_error_z and len(self.path_msg.poses) > 1:
            rospy.loginfo("Goal %s reached, new goal %s", self.path_msg.poses[0], self.path_msg.poses[1])
            self.pid_reset()
            # delete pose
            self.path_msg.poses.pop(0)
            self.target_position = self.path_msg.poses[0]
            self.pub_target.publish(self.target_position)
            self.pub_path.publish(self.path_msg)
        elif abs(self.pid_x.error) < self.target_error_x and abs(self.pid_y.error) < self.target_error_y and \
                abs(self.pid_z.error) < self.target_error_z and len(self.path_msg.poses) == 1:
            rospy.loginfo("Last goal reached")
            self.target = False
            self.pid_reset()
            self.zero_velocity()
            self.path_msg.poses.pop(0)
            self.last_goal_reached = True

    def calculate_control(self):
        # try to look up current position
        # if self.mission_state.is_flying:
        # calculate PID control value for Tello x control
        x_vel = self.pid_x.update(self.current_pose.position.x, self.target_position.pose.position.x)
        self.fly_msg.linear.x = x_vel

        # calculate PID control value for Tello y control
        y_vel = self.pid_y.update(self.current_pose.position.y, self.target_position.pose.position.y)
        self.fly_msg.linear.y = y_vel
        # calculate PID control value for Tello z control
        z_vel = self.pid_z.update(self.current_pose.position.z, self.target_position.pose.position.z)
        self.fly_msg.linear.z = z_vel
        # calculate PID control value for Tello yaw control
        target_orientation = self.target_position.pose.orientation
        target_orientation_list = [target_orientation.x, target_orientation.y, target_orientation.z,
                                   target_orientation.w]
        target_angles = euler_from_quaternion(target_orientation_list)
        z_ang = self.pid_yaw.update(self.current_yaw, target_angles[2])
        self.fly_msg.angular.z = z_ang
        # rospy.loginfo('ERROR x: %.2f | error y: %.2f | error z: %.2f | error yaw: %.2f',
        #              self.pid_x.error, self.pid_y.error, self.pid_z.error, self.pid_yaw.error)
        # rospy.loginfo('VEL x: %.2f |  y: %.2f |  z: %.2f |  yaw: %.2f', x_vel, y_vel, z_vel, z_ang)
        if self.log_controller:
            self.log_controller.current_x.append(deepcopy(self.current_pose.position.x))
            self.log_controller.target_x.append(deepcopy(self.target_position.pose.position.x))
            self.log_controller.current_y.append(deepcopy(self.current_pose.position.y))
            self.log_controller.target_y.append(deepcopy(self.target_position.pose.position.y))
            self.log_controller.current_z.append(deepcopy(self.current_pose.position.z))
            self.log_controller.target_z.append(deepcopy(self.target_position.pose.position.z))
            self.log_controller.current_yaw.append(deepcopy(self.current_yaw))
            self.log_controller.target_yaw.append(deepcopy(target_angles[2]))
            self.log_controller.error_x.append(deepcopy(self.pid_x.error))
            self.log_controller.error_y.append(deepcopy(self.pid_y.error))
            self.log_controller.error_z.append(deepcopy(self.pid_z.error))
            self.log_controller.error_yaw.append(deepcopy(self.pid_yaw.error))
            self.log_controller.time_axis.append(time.time())
            
    def calibrate_word_scale(self):
        rospy.loginfo('starting calibration of SLAM real world scale')
        # set and go to down position
        self.target_position = self.mission_state.down_pose
        self.calculate_control()
        rospy.loginfo('go to lower position')
        while abs(self.pid_z.error) > 0.05:
            if self.mission_state.stop:
                rospy.loginfo('calibration stopped')
                return
            self.calculate_control()
            # Flip commands for tello!
            self.process_and_flip_command()
            self.pub_velocity.publish(self.fly_tello_msg)
            rospy.sleep(0.1)
        # measure 5 points
        slam_z_down = []
        tello_z_down = []
        rospy.loginfo('measure height values')
        for i in range(5):
            slam_z_down.append(self.slam_pos.pose.position.z)
            tello_z_down.append(self.current_pose.position.z)
            rospy.sleep(0.2)
        # set and go to up position
        self.target_position = self.mission_state.up_pose
        self.calculate_control()
        rospy.loginfo('go to upper position')
        while abs(self.pid_z.error) > 0.05:
            if self.mission_state.stop:
                rospy.loginfo('calibration stopped')
                return
            self.calculate_control()
            # Flip commands for tello!
            self.process_and_flip_command()
            self.pub_velocity.publish(self.fly_tello_msg)
            rospy.sleep(0.1)
        # measure 5 points
        slam_z_up = []
        tello_z_up = []
        rospy.loginfo('measure height values')
        for i in range(5):
            slam_z_up.append(self.slam_pos.pose.position.z)
            tello_z_up.append(self.current_pose.position.z)
            rospy.sleep(0.2)
        # calculate real world scale
        rospy.loginfo('calculate scale')
        try:
            tello_up_mean = sum(tello_z_up) / len(tello_z_up)
            tello_down_mean = sum(tello_z_down) / len(tello_z_down)
            slam_up_mean = sum(slam_z_up) / len(slam_z_up)
            slam_down_mean = sum(slam_z_down) / len(slam_z_down)
            self.real_world_scale = (tello_up_mean - tello_down_mean) / (slam_up_mean - slam_down_mean)
            self.pub_real_world_scale.publish(self.real_world_scale)
            print('Diff Odom: %s' % (tello_up_mean - tello_down_mean))
            print('Diff SLAM: %s' % (slam_up_mean - slam_down_mean))
            print("Scale: %s" % self.real_world_scale)
            rospy.loginfo('scale published')
            self.mission_state.current_state = self.mission_state.STOPPED
            self.mission_state.stop = True
        except ZeroDivisionError:
            rospy.loginfo('Error: ZeroDivisionError')

    def process_and_flip_command(self):
        # rotate vel commands to drone frame around z
        vel_x = math.cos(self.current_yaw) * self.fly_msg.linear.x + \
                                math.sin(self.current_yaw) * self.fly_msg.linear.y
        vel_y = -math.sin(self.current_yaw) * self.fly_msg.linear.x + \
                                math.cos(self.current_yaw) * self.fly_msg.linear.y

        self.fly_msg.linear.x = vel_x
        self.fly_msg.linear.y = vel_y
        self.fly_tello_msg.linear.x = -self.fly_msg.linear.y
        self.fly_tello_msg.linear.y = self.fly_msg.linear.x
        self.fly_tello_msg.linear.z = self.fly_msg.linear.z
        self.fly_tello_msg.angular.z = -self.fly_msg.angular.z

    def zero_velocity(self):
        self.fly_msg.linear.x = 0.0
        self.fly_msg.linear.y = 0.0
        self.fly_msg.linear.z = 0.0
        self.fly_msg.angular.z = 0.0
        self.pub_velocity.publish(self.fly_tello_msg)

    # This process handles all of the flight command messages.
    def control_iterator(self):
        # delta time is in a fraction of a second (0.02 sec for 50 Hz)
        dt = time.time() - self.last_iteration_time
        # print("dt is %f" % dt)

        #print(self.mission_state.current_state)
        # reset all to zero if stop flag
        if self.mission_state.stop:
            # reduce pitch and roll to zero
            rospy.loginfo("Mission stopped")
            self.zero_velocity()
        # take off
        elif self.mission_state.current_state == self.mission_state.TAKEOFF and not self.mission_state.is_flying:
            #publish target and path
            self.pub_target.publish(self.target_position)
            self.pub_path.publish(self.path_msg)
            # takeoff
            empty = Empty()
            timeout = time.time() + 10    # wait max. 10s for takeoff
            self.pub_takeoff.publish(empty)
            rospy.loginfo("Take off sent")
            rospy.sleep(0.1)    # give time to publish
            while time.time() < timeout and not rospy.is_shutdown():
                if self.mission_state.is_flying or self.mission_state.stop:
                    break
        # land
        elif self.mission_state.current_state == self.mission_state.LANDING and self.mission_state.is_flying:
            # land
            self.zero_velocity()
            empty = Empty()
            timeout = time.time() + 10  # wait max. 10s for takeoff
            self.pub_land.publish(empty)
            rospy.loginfo("Landing sent")
            rospy.sleep(0.1)  # give time to publish
            # wait for landing
            while time.time() < timeout and not rospy.is_shutdown():
                if not self.mission_state.is_flying or self.mission_state.stop:
                    break
        # fly to a single target position
        elif self.mission_state.current_state == self.mission_state.SINGLE_TARGET and self.target: #and \
                # self.mission_state.is_flying and :
            # print("calculate control, current target: ", self.target_position)
            # calculate velocities
            self.calculate_control()
            # check goal
            self.close_enough()
            # Flip commands for tello!
            self.process_and_flip_command()
            self.pub_velocity.publish(self.fly_tello_msg)
        elif self.mission_state.current_state == self.mission_state.CALIBRATE:
            self.calibrate_word_scale()

        elif self.mission_state.current_state == self.mission_state.GO_HOME and self.target:
            rospy.loginfo("Going home")
            # calculate velocities
            self.calculate_control()
            # check goal
            self.close_enough()
            # Flip commands for tello!
            self.process_and_flip_command()
            self.pub_velocity.publish(self.fly_tello_msg)
            # if home reached land
            if self.last_goal_reached:
                self.mission_state.stop = True
                self.mission_state.current_state = self.mission_state.LANDING

        # do next iteration of not shutdown
        if not rospy.is_shutdown():
            timer = max(1.0/self.control_rate - (time.time() - self.last_iteration_time), 1/self.control_rate/2)
            self.last_iteration_time = time.time()
            threading.Timer(timer, self.control_iterator).start()

    def log_iterator(self):
        if self.mission_state.current_state == self.mission_state.SINGLE_TARGET and self.target and self.show_trail:
            temp_pose = PoseStamped()
            temp_pose.header.frame_id = 'world'
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.pose = self.current_pose
            self.trail_msg.poses.append(deepcopy(temp_pose))
            self.trail_msg.header.seq += 1
            self.trail_msg.header.frame_id = 'world'
            self.trail_msg.header.stamp = rospy.Time.now()
            self.pub_trail.publish(self.trail_msg)
            # do next iteration of not shutdown
        if not rospy.is_shutdown():
            timer = 1.0     # 1 Hz
            threading.Timer(timer, self.log_iterator).start()


if __name__ == '__main__':
    # Create controller
    # start up the controller node and run until shutdown by interrupt
    try:
        controller = TelloController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tello controller node terminated.")
