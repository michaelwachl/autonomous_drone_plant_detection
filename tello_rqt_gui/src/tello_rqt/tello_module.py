#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""A module-level docstring

Notice the comment above the docstring specifying the encoding.
Docstrings do appear in the bytecode, so you can access this through
the ``__doc__`` attribute. This is also what you'll see if you call
help() on a module or any other Python object.
"""

__author__ = 'wachl'

import os
import time
import rospy
import rospkg
import threading
import multiprocessing
import sys
import traceback
# import av

# Import Generated UI from Python file
from resource.TelloPlugin import Ui_TelloPlugin
import time
# Import connection state
from tello_driver.msg import TelloStatus
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import state

import numpy as np

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout
from python_qt_binding.QtCore import QProcess, pyqtSignal, QThread, QObject, QSize, QTime, QTimer, QEvent
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon, QPixmap, QImage

import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import euler_from_quaternion


class TelloPlugin(Plugin):

    STATE_DISCONNECTED = state.State('disconnected')
    STATE_CONNECTING = state.State('connecting')
    STATE_CONNECTED = state.State('connected')
    STATE_FLYING = state.State('flying')
    STATE_QUIT = state.State('quit')

    # define signals ROS updates
    sig_connection_changed = pyqtSignal()
    sig_update_position = pyqtSignal()
    sig_battery_percentage = pyqtSignal(int)
    sig_wifi_percentage = pyqtSignal(int)
    sig_height = pyqtSignal(str)
    sig_speed = pyqtSignal(str)
    sig_flight_time = pyqtSignal(str)
    sig_remaining_time = pyqtSignal(str)
    sig_battery_low = pyqtSignal(str)
    sig_fly_mode = pyqtSignal(str)
    sig_fast_mode = pyqtSignal(str)
    sig_is_flying = pyqtSignal(str)
    sig_remaining_time = pyqtSignal(str)
    sig_foto_info = pyqtSignal(str)
    sig_image = pyqtSignal(object)

    # sig_mission_state = pyqtSignal(str)


    def __init__(self, context):
        super(TelloPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TelloPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)


        # load main widget
        self._widget = QWidget()
        self.ui = Ui_TelloPlugin()
        self.ui.setupUi(self._widget)
        self.ui.tabWidget.setCurrentIndex(0)
        self._widget.setObjectName('TelloPluginUi')

        # Add left and right key arrow blocking
        filter = KeySuppress()
        self.ui.tabWidget.installEventFilter(filter)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        print("Tello widget added")

        # Tello node
        self.bridge = CvBridge()
        self.pos_listener = TransformListener()

        # Add recorder and Recording Timer
        self.recording = False
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.update_record_time)
        self.record_time = QTime(00,00,00)
        self.video_recorder = None 
        self.VIDEO_TYPE = {'avi': cv2.VideoWriter_fourcc(*'XVID'), 'mp4': cv2.VideoWriter_fourcc(*'MP4V')}

        # Callbacks GUI items
        self.ui.connect_button.clicked[bool].connect(self.handle_connect_clicked)			
        self.ui.record_button.clicked[bool].connect(self.handle_record_clicked)
        self.ui.foto_button.clicked[bool].connect(self.handle_foto_clicked)
        self.ui.stop_button.clicked[bool].connect(self.handle_stop_clicked)
        self.ui.start_button.clicked[bool].connect(self.handle_start_clicked)
        self.ui.land_button.clicked[bool].connect(self.handle_land_clicked)
        self.ui.target_button.clicked[bool].connect(self.handle_target_clicked)

        # Set icons
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        icon_path = rospack.get_path('tello_rqt_gui') + '/src/tello_rqt/resource/icons/'
        self.ui.record_button.setText('')   # override text with icon
        self.ui.record_button.setIcon(QIcon(QPixmap(icon_path + 'video.png')))		
        self.ui.record_button.setIconSize(QSize(25,25))	
        self.ui.foto_button.setText('')   # override text with icon
        self.ui.foto_button.setIcon(QIcon(QPixmap(icon_path + 'foto.png')))      
        self.ui.foto_button.setIconSize(QSize(25,25)) 
        self.ui.label_record_time.setText('record')


        # Set bar values to 0
        self.ui.link_bar.setValue(0)
        self.ui.battery_bar.setValue(0)

        # create signals and slots for updates
        # never access widgets and GUI related things directly from a thread other than the main thread
        self.sig_connection_changed.connect(self.update_ui_state)
        self.sig_update_position.connect(self.update_position_data)
        self.sig_battery_percentage.connect(self.ui.battery_bar.setValue)
        self.sig_wifi_percentage.connect(self.ui.link_bar.setValue)
        self.sig_height.connect(self.ui.label_height.setText)
        self.sig_speed.connect(self.ui.label_speed.setText)
        self.sig_flight_time.connect(self.ui.label_flight_time.setText)
        self.sig_image.connect(self.set_image)
        self.sig_battery_low.connect(self.ui.label_battery_low.setText)
        self.sig_is_flying.connect(self.ui.label_is_flying.setText)
        self.sig_fly_mode.connect(self.ui.label_fly_mode.setText)
        self.sig_fast_mode.connect(self.ui.label_fast_mode.setText)
        self.sig_remaining_time.connect(self.ui.label_remaining_time.setText)
        self.sig_foto_info.connect(self.ui.label_foto.setText)

        #self.sig_mission_state.connect(self.ui.label_mission_state.setText)

        # Connect signals from Tello, Controller und SLAM to GUI
        self.ns_tello = 'tello/'
        self.ns_controller = 'tello_controller/'
        self.topic_slam_pose = '/orb_slam2_mono/pose'
        self.tello_state = self.STATE_DISCONNECTED.getname()
        self.pub_connect = rospy.Publisher(self.ns_tello+'connect', Empty, queue_size=1, latch=True)
        self.pub_disconnect = rospy.Publisher(self.ns_tello+'disconnect', Empty, queue_size=1, latch=True)
        self.pub_take_picture = rospy.Publisher(self.ns_tello+'take_picture', Empty, queue_size=1, latch=True)
        self.pub_mission_command = rospy.Publisher(self.ns_controller+'mission_command', String, queue_size=1, latch=True)
        self.sub_picture_update = rospy.Subscriber(self.ns_tello+'picture_update', String, self.cb_picture_update)
        self.sub_status = rospy.Subscriber(self.ns_tello+'status', TelloStatus, self.cb_status)
        self.sub_connection = rospy.Subscriber(self.ns_tello+'connection_state', String, self.cb_connection)
        self.sub_video = rospy.Subscriber('tello/camera/image_raw', Image, self.cb_video)
        self.sub_odom = rospy.Subscriber(self.ns_tello+'odom', Odometry, self.cb_odom, queue_size=1)
        self.sub_slam_pose = rospy.Subscriber(self.topic_slam_pose, PoseStamped, self.cb_slam_pose, queue_size=1)

        self.sub_mission = rospy.Subscriber(self.ns_controller+'mission_state', String, self.cb_mission_state)

        self.cv_image = None
        print('init done')

    def update_ui_state(self):
        if self.tello_state == self.STATE_QUIT.getname() or self.tello_state == self.STATE_DISCONNECTED.getname():
            self.ui.connect_button.setText("Connect")
            # self.ui.connect_button.setStyleSheet("background-color: None")
            self.ui.link_bar.setValue(0)
            self.ui.battery_bar.setValue(0)
        elif self.tello_state == self.STATE_CONNECTED.getname():
            self.ui.connect_button.setText("Disconnect")
            # self.ui.connect_button.setStyleSheet("background-color: lightgreen")
        #    print('end if..')
        elif self.tello_state == self.STATE_CONNECTING.getname():
            self.ui.connect_button.setText("Connecting...")
            # Set bar values to 0
            self.ui.link_bar.setValue(0)
            self.ui.battery_bar.setValue(0)
            # self.ui.connect_button.setStyleSheet("background-color: None")

    def handle_connect_clicked(self):
        """
        User pressed connect button
        """
        print(self.tello_state)
        if self.tello_state == self.STATE_CONNECTED.getname():
            self.pub_disconnect.publish()
        else:
            self.pub_connect.publish()

    def handle_record_clicked(self):
        # Create a file in ~/Pictures/ to receive image data from the drone.
        print('Recording: ', self.recording)
        if self.recording:
            # close and save video
            self.recording = False
            self.record_time = QTime(00,00,00)
            self.ui.label_record_time.setText('record')
        else:
            self.record_timer.start(1000) # update every second
            # self.stop_event.clear()  # clear event
            self.path = '%s/Videos/tello-%s.avi' % (
                os.getenv('HOME'),
                time.strftime("%Y-%m-%d_%H:%M:%S"))
            self.video_recorder = cv2.VideoWriter(self.path, self.VIDEO_TYPE['avi'], 25, (960, 720))
            self.recording = True

    def handle_foto_clicked(self):
        self.ui.label_foto.setText("Taking picture")
        self.pub_take_picture.publish(Empty())

    def handle_stop_clicked(self):
        self.pub_mission_command.publish('stop')

    def handle_start_clicked(self):
        self.pub_mission_command.publish('start')

    def handle_land_clicked(self):
        self.pub_mission_command.publish('land')

    def handle_target_clicked(self):
        self.pub_mission_command.publish('target')

    def update_record_time(self):
        # self.record_time = QtCore.QTime(00,00,00)
        # print("Update Recording time")
        if self.recording:
            self.record_time = self.record_time.addSecs(1)
            print(self.record_time.toString())
            self.ui.label_record_time.setText(self.record_time.toString())

    def update_position_data(self):
        # try to look up current position
        try:
            (tello_trans, tello_rot) = self.pos_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            # print('trans: ', tello_trans, ' | rot: ', tello_rot)
            """
            euler_angles = euler_from_quaternion(tello_rot)
            self.ui.label_internal_x.setText('%.2f' % tello_trans[0])
            self.ui.label_internal_y.setText('%.2f' % tello_trans[1])
            self.ui.label_internal_z.setText('%.2f' % tello_trans[2])
            self.ui.label_internal_yaw.setText('%.2f' % euler_angles[2])
            """
        except (LookupException, ConnectivityException, ExtrapolationException):
            print('No Position Data available')
            self.ui.label_internal_x.setText('wait')
            self.ui.label_internal_y.setText('wait')
            self.ui.label_internal_z.setText('wait')
            self.ui.label_internal_yaw.setText('wait')
            self.ui.label_slam_x.setText('wait')
            self.ui.label_slam_y.setText('wait')
            self.ui.label_slam_z.setText('wait')
            self.ui.label_slam_yaw.setText('wait')

    def cb_odom(self, msg):
        current_orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        current_angles = euler_from_quaternion(current_orientation_list)
        self.ui.label_internal_x.setText('%.2f' % msg.pose.pose.position.x)
        self.ui.label_internal_y.setText('%.2f' % msg.pose.pose.position.y)
        self.ui.label_internal_z.setText('%.2f' % msg.pose.pose.position.z)
        self.ui.label_internal_yaw.setText('%.2f' % current_angles[2])

    def cb_slam_pose(self, msg):
        current_orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y,
                                    msg.pose.orientation.z, msg.pose.orientation.w]
        current_angles = euler_from_quaternion(current_orientation_list)
        self.ui.label_slam_x.setText('%.2f' % msg.pose.position.x)
        self.ui.label_slam_y.setText('%.2f' % msg.pose.position.y)
        self.ui.label_slam_z.setText('%.2f' % msg.pose.position.z)
        self.ui.label_slam_yaw.setText('%.2f' % current_angles[2])

    def cb_picture_update(self, msg):
        self.ui.label_foto.setText(msg.data)

    def cb_mission_state(self, msg):
        self.ui.label_mission_state.setText(msg.data)

    def cb_status(self, msg):
        self.sig_battery_percentage.emit(msg.battery_percentage)
        self.sig_wifi_percentage.emit(msg.wifi_strength)
        self.sig_height.emit("%.2f" % msg.height_m)
        self.sig_speed.emit("%.2f" % msg.speed_horizontal_mps)
        self.sig_flight_time.emit("%.2f" % msg.flight_time_sec)
        self.sig_battery_low.emit(str(msg.is_battery_lower))
        self.sig_fly_mode.emit(str(msg.fly_mode))
        self.sig_fast_mode.emit(str(msg.cmd_fast_mode))
        self.sig_is_flying.emit(str(msg.is_flying))
        m, s = divmod(-msg.drone_fly_time_left_sec/10., 60)
        # print("Remaining: ",m,"min, ",s,"s")
        self.sig_remaining_time.emit("%d:%d" % (m, s))
        # check position
        self.sig_update_position.emit()

    def cb_connection(self, msg):
        print(self.tello_state)
        self.tello_state = msg.data
        self.sig_connection_changed.emit()

    def cb_video(self, msg):
        """
        callback to record video and display video 
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.recording:
            print('recording.... ')
            self.video_recorder.write(self.cv_image)
        # show Video in GUI
        self.sig_image.emit(self.cv_image)

    def set_image(self, img_bgr):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB) # switch b and r channel
        # Qt Image
        qt_image = QImage(img_rgb, img_rgb.shape[1], img_rgb.shape[0], QImage.Format_RGB888)
        self.ui.label_video.setPixmap(QPixmap(qt_image))  # set pixmap to label


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # wait to terminate Threads
        # self._ros_process.close()
        print(os.path.dirname(os.path.realpath(__file__)))
        self.pub_disconnect.publish(Empty())
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

class KeySuppress(QObject):
    def eventFilter(self, obj, event):
        print('key pressed')
        if event.type() == QEvent.KeyPress and (event.key() == Qt.Key_Left or event.key() == Qt.Key_Right):
            return True # eat alt+tab or alt+shift+tab key
        else:
            # standard event processing
            return QObject.eventFilter(self, obj, event)