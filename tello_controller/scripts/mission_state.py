import rospy
from std_msgs.msg import String, Empty
from tello_driver.msg import TelloStatus
from geometry_msgs.msg import PoseStamped

class MissionState:
    def __init__(self):
        # initializing states
        self.READY = 'ready'
        self.STOPPED = 'stopped'
        self.TAKEOFF = 'takeoff'
        self.CALIBRATE = 'calibrate'
        self.DETECT = 'detect'
        self.CIRCLE = 'circle'
        self.GO_HOME = 'go_home'
        self.LANDING = 'landing'
        self.HOVERING = 'hovering'
        self.SINGLE_TARGET = 'single_target'

        # setting current state of the system
        self.current_state = self.READY
        self.calibrated = False

        # home pose
        self.home_pose = PoseStamped()
        self.home_pose.header.seq = 0
        self.home_pose.header.frame_id = "world"
        self.home_pose.pose.position.z = 0.5

        # stopped flag to denote that iteration is stopped due to bad
        # input against which transition was not defined.
        self.stop = False
        self.is_flying = False
        self.fly_mode = 0
        self.battery_low = False
        self.battery_threshold = 20
        self.ns_tello = 'tello/'
        self.ns_node = 'tello_controller/'
        ns_control = ''
        self.sub_status = rospy.Subscriber(self.ns_tello+'status', TelloStatus, self.cb_status)
        self.sub_mission_command = rospy.Subscriber(self.ns_node+'mission_command', String, self.cb_mission_command)
        self.sub_mission_stop = rospy.Subscriber(self.ns_node + 'stop', Empty, self.cb_stop, queue_size=1)
        self.pub_mission_state = rospy.Publisher(self.ns_node+'mission_state', String, queue_size=1)
        self.pub_home_pose = rospy.Publisher(self.ns_node+'target_pose', PoseStamped, queue_size=1)

    def cb_stop(self, msg):
        self.stop = True
        self.current_state = self.STOPPED
        self.pub_mission_state()
        rospy.loginfo("mission stopped")

    def cb_mission_command(self, msg):
        if msg.data == 'stop':
            self.stop = True
            self.current_state = self.STOPPED
        elif msg.data == 'start':
            self.stop = False
            self.current_state = self.TAKEOFF
            rospy.loginfo("state takeoff set")
        elif msg.data == 'calibrate':
            self.current_state = self.CALIBRATE
        elif msg.data == 'land':
            self.stop = False
            self.current_state = self.LANDING
        elif msg.data == 'home':
            self.current_state = self.GO_HOME
            self.stop = False
            self.pub_home_pose.publish(self.home_pose)
        elif msg.data == 'circle':
            self.current_state = self.CIRCLE
        elif msg.data == 'detect':
            self.current_state = self.DETECT
        elif msg.data == 'target':
            self.stop = False
            self.current_state = self.SINGLE_TARGET

        self.publish_mission_state()

    def cb_status(self, msg):
        # check if drone is flying
        """
        int fly mode:
        1: not connected
        6: connected and on ground/ hovering
        11: taking off
        12: landing
        """
        self.fly_mode = msg.fly_mode
        # check if flying
        if msg.is_flying:
            self.is_flying = True
            rospy.sleep(5)  # give time to calibrate after launch
        else:
            self.is_flying = False

        # check battery level
        if msg.battery_percentage <= 20:
            self.battery_low = True
            self.current_state = self.GO_HOME
        else:
            self.battery_low = False

    def publish_mission_state(self):
        self.pub_mission_state.publish(self.current_state)