#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException


class TelloController(object):
    """Controller to fly Tello in 3D coordinates"""

    def __init__(self):

        self.namespace = 'tello/'
        self.hover_position = None

        # subscribe to target pose
        self.target_position = PoseStamped()
        self.target_angles = [0.0, 0.0, 0.0]

        # Initialize the tf listener
        self.listener = TransformListener()

        # initialize publisher for tello command velocity (geometry_Twist)
        self.fly_msg = self.fly_tello_msg = Twist()                           # set the fly command
        self.velocity_pub = rospy.Publisher(self.namespace+'cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher(self.namespace+'takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher(self.namespace+'land', Empty, queue_size=1)


    # This function gets the current transform between the target_frame and the source_frame.
    def get_transform(self):
        # try to look up current position
        try:
            t = self.listener.getLatestCommonTime('odom', 'base_link')
            (tello_trans, tello_rot) = self.listener.lookupTransform('odom', 'base_link', t)    # source, target, time
            return tello_trans, tello_rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.loginfo("Waiting for transform")


    # This process handles all of the flight command messages.
    def iteration(self, event):
        try:
            # delta time is in a fraction of a second (0.02 sec for 50 Hz)
            dt = float(rospy.Time.to_sec(event.current_real)) - float(rospy.Time.to_sec(event.last_real))
        except (AttributeError, TypeError):
            dt = 0
        # print("dt is %f" % dt)
        # try to look up current position
        try:
            (tello_trans, tello_rot) = self.get_transform()
            euler_current = euler_from_quaternion(tello_rot)
        except (AttributeError, TypeError):
            print('Waiting for topic')
            return

        # receive tello state
        rospy.loginfo("tello trans %f %f %f", tello_trans[0], tello_trans[1], tello_trans[2])


        rospy.loginfo("Flying to target!")

        self.fly_msg.linear.x = 0.0 # your value here

        self.fly_msg.linear.y = 0.0 # your value here

        self.fly_msg.linear.z = 0.0 # your value here

        # rospy.loginfo("Target orientation is %f", self.target_angles[2])
        self.fly_msg.angular.z = 0.0 # your value here


        ##### Stop ########
        # do nothing

        # self.fly_msg.linear.x = 0.0
        # self.fly_msg.linear.y = 0.0
        # self.fly_msg.linear.z = 0.0
        # self.fly_msg.angular.z = 0.0

        # record values to log file
        rospy.loginfo("Current position: %f %f %f  yaw: %f", tello_trans[0], tello_trans[1], tello_trans[2],
                      euler_current[2])

        rospy.loginfo("Velocity msg published x %f y %f z % f yaw %f", self.fly_msg.linear.x, self.fly_msg.linear.y,
                      self.fly_msg.linear.z, self.fly_msg.angular.z)

        # publish command velocity message to Tello
        # Flip commands for tello!
        self.fly_tello_msg.linear.x = self.fly_msg.linear.y
        self.fly_tello_msg.linear.y = -self.fly_msg.linear.x
        self.fly_tello_msg.linear.z = self.fly_msg.linear.z
        self.fly_tello_msg.angular.z = -self.fly_msg.angular.z

        self.velocity_pub.publish(self.fly_tello_msg)


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('tello_groundstation_controller', anonymous=True)
    # set parameters on parameter server
    tello_frame = rospy.get_param("~frame", "base_link")
    frequency = rospy.get_param("frequency", 20.0)

    # start up the controller node and run until shutdown by interrupt
    try:
        controller = TelloController()
        rospy.Timer(rospy.Duration(1.0/frequency), controller.iteration)
        rospy.spin()                  # keep process alive

    except rospy.ROSInterruptException:
        rospy.loginfo("Tello groundsation controller node terminated.")
