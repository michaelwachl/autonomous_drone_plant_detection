#!/usr/bin/env python
import sys
import os
import rospy
from std_msgs.msg import Empty, UInt16, Bool
from environment_sensor.msg import SCD30

from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import pickle

import atexit
import datetime

class SCD30Plot(object):
    def __init__(self):
        self.namespace = 'tello/'

        # Matplotlib
        # plots
        self.fig_path = plt.figure()
        self.fig_temperature = plt.figure()
        self.fig_humidity = plt.figure()
        self.fig_co2 = plt.figure()
        self.ax_path = self.fig_path.add_subplot(111, projection="3d")
        self.ax_temperature = self.fig_temperature.add_subplot(111, projection="3d")
        self.ax_humidity = self.fig_humidity.add_subplot(111, projection="3d")
        self.ax_co2 = self.fig_co2.add_subplot(111, projection="3d")

        # current variables
        self.current_time = rospy.get_rostime()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.temperature = []
        self.co2 = []
        self.humidity = []
        self.x = []
        self.y = []
        self.z = []
        self.x_sensor = []
        self.y_sensor = []
        self.z_sensor = []

        # Subscribe to Sensor and Odom topic
        self.sub_scd30 = rospy.Subscriber('scd30', SCD30, self.cb_scd30)
        self.sub_odom = rospy.Subscriber(self.namespace + 'odom', Odometry, self.cb_odom)
        rospy.loginfo('SCD30 Plot ready')

        # at exit save plot
        atexit.register(self.save_plot)

    def cb_scd30(self, msg):
        self.x_sensor.append(self.current_x)
        self.y_sensor.append(self.current_y)
        self.z_sensor.append(self.current_z)
        # print(msg)
        # print(msg.temperature)
        self.temperature.append(msg.temperature)
        self.co2.append(msg.co2)
        self.humidity.append(msg.humidity)

    def cb_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.z.append(msg.pose.pose.position.z)

        # self.update_path_plot()

    def update_path_plot(self):
        return

    def save_plot(self):
        print('dumping variables')
        path = '%s/Pictures/%s' % (os.getenv('HOME'),
                                   datetime.datetime.now().strftime('sensor-data-path-%Y-%m-%d_%H%M%S'))

        # save save variables as pickle
        pickle.dump([self.x, self.y, self.z, self.x_sensor, self.y_sensor, self.z_sensor,
                     self.temperature, self.humidity, self.co2], file(path + '.pickle', 'w'))
        print('Variables saved')

def main():
    rospy.init_node('scd30_plot_node')
    sensor = SCD30Plot()

    rospy.spin()


if __name__ == '__main__':
    main()
