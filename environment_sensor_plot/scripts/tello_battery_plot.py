#!/usr/bin/env python

import rospy
import os
import matplotlib.pyplot as plt
import pickle
from tello_driver.msg import TelloStatus
import numpy as np

import atexit
import datetime

RECORD_BATTERY = False

class BatteryLogPlot(object):
    def __init__(self):
        # current variables
        self.zero_time = rospy.Time.now()
        print(self.zero_time)
        self.time_axis = []
        self.battery_level = []

        # Subscribe to Sensor and Odom topic
        self.sub_tello_status = rospy.Subscriber('/tello/status', TelloStatus, self.cb_status)
        rospy.loginfo('Battery Plot ready')

        # at exit save plot
        atexit.register(self.save_plot)

    def cb_status(self, msg):
        now = rospy.Time.now()
        self.time_axis.append(now.secs-self.zero_time.secs)
        self.battery_level.append(msg.battery_percentage)
        print('time passed: ', now.secs-self.zero_time.secs, '  battery: ', msg.battery_percentage)


    def save_plot(self):
        print('dumping variables')
        path = '%s/Documents/%s' % (os.getenv('HOME'),
                                   datetime.datetime.now().strftime('battery-hover-tello-sensor-no-ds%Y-%m-%d_%H%M%S'))

        # save save variables as pickle
        pickle.dump([self.time_axis, self.battery_level], file(path + '.pickle', 'w'))
        print('Variables saved')


def plot_battery():
    # load pickles
    # path_tello_sensor_esp_off = '/home/tello18/Documents/battery-hover-2020-08-17_235612.pickle'
    # path_tello_sensor_esp_on_ds_on = '/home/tello18/Documents/battery-hover-2020-08-17_235612.pickle'
    # path_tello_sensor_esp_on_ds_off = '/home/tello18/Documents/battery-hover-no-ds2020-09-06_222136'

    tello_only_1 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-tello-only2020-08-18.pickle'
    tello_only_2 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-tello-only2020-09-08_144210.pickle'
    tello_sensor_1 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-tello-sensor-2020-08-18_165044.pickle'
    tello_sensor_2 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-sensor-off2020-09-07_180356.pickle'
    tello_sensor_no_probs = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-tello-sensor-off-no-props2020-09-08_175258.pickle'
    tello_sensor_ds_1 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-sensor-ds-1-2020-08-20_185543.pickle'
    tello_sensor_ds_2 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-sensor-ds-2-2020-08-21_155335.pickle'
    tello_sensor_no_ds_1 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-no-ds2020-09-06_222136.pickle'
    tello_sensor_no_ds_2 = '/home/tello18/Documents/Tello_flight_time_test/battery-hover-tello-sensor-no-ds2020-09-10_183630.pickle'



    time_tello, battery_level_tello = pickle.load(file(tello_only_1))
    time_tello_2, battery_level_tello_2 = pickle.load(file(tello_only_2))
    time_tello_sensor_no_probs, battery_level_tello_sensor_no_probs = pickle.load(file(tello_sensor_no_probs))
    time_tello_sensor, battery_level_tello_sensor = pickle.load(file(tello_sensor_1))
    time_tello_sensor_2, battery_level_tello_sensor_2 = pickle.load(file(tello_sensor_2))
    time_tello_sensor_ds, battery_level_tello_ds_sensor = pickle.load(file(tello_sensor_ds_1))
    time_tello_sensor_ds_2, battery_level_tello_ds_sensor_2 = pickle.load(file(tello_sensor_ds_2))
    time_tello_sensor_no_ds, battery_level_tello_sensor_no_ds = pickle.load(file(tello_sensor_no_ds_1))
    time_tello_sensor_no_ds_2, battery_level_tello_sensor_no_ds_2 = pickle.load(file(tello_sensor_no_ds_2))

    # time in min
    time_tello_min = [x / 60.0 for x in time_tello]
    time_tello_min_2 = [x / 60.0 for x in time_tello_2]
    time_tello_sensor_no_probs_min = [x / 60.0 for x in time_tello_sensor_no_probs]
    time_tello_sensor_min = [x / 60.0 for x in time_tello_sensor]
    time_tello_sensor_min_2 = [x / 60.0 for x in time_tello_sensor_2]
    time_tello_sensor_ds_min = [x / 60.0 for x in time_tello_sensor_ds]
    time_tello_sensor_ds_min_2 = [x / 60.0 for x in time_tello_sensor_ds_2]
    time_tello_sensor_no_ds_min = [x / 60.0 for x in time_tello_sensor_no_ds]
    time_tello_sensor_no_ds_min_2 = [x / 60.0 for x in time_tello_sensor_no_ds_2]

    # plot
    fig_battery = plt.figure()
    ax_battery = fig_battery.add_subplot(111)
    # ax_battery.scatter(time_tello_min, battery_level_tello, marker='.', c='b', linewidths=0.01, alpha=0.1)
    # , 'o', color='b', label='Tello only')
    # Fit with polyfit
    # p = np.poly1d(np.polyfit(time_tello_min, battery_level_tello, 3))

    # ax_battery.plot(time_tello_min, p(time_tello_min), color='b', linewidth=2.0, label='Tello only')
    ax_battery.plot(time_tello_min, battery_level_tello, color='g', linewidth=1.0,
                    label='Tello only: %.1f min' % time_tello_min[-1])
    ax_battery.plot(time_tello_min_2, battery_level_tello_2, color='g', linewidth=1.0,
                    label='Tello only: %.1f min' % time_tello_min_2[-1])
    ax_battery.plot(time_tello_sensor_no_probs_min, battery_level_tello_sensor_no_probs, color='c', linewidth=1.0,
                    label='Tello no guards with sensor off: %.1f min' % time_tello_sensor_no_probs_min[-1])
    ax_battery.plot(time_tello_sensor_min, battery_level_tello_sensor, color='b', linewidth=1.0,
                    label='Tello with sensor off: %.1f min' % time_tello_sensor_min[-1])
    ax_battery.plot(time_tello_sensor_min_2, battery_level_tello_sensor_2, color='b', linewidth=1.0,
                    label='Tello with sensor off: %.1f min' % time_tello_sensor_min_2[-1])
    ax_battery.plot(time_tello_sensor_ds_min, battery_level_tello_ds_sensor, color='y', linewidth=1.0,
                    label='Tello with sensor and ds: %.1f min' % time_tello_sensor_ds_min[-1])
    ax_battery.plot(time_tello_sensor_ds_min_2, battery_level_tello_ds_sensor_2, color='y', linewidth=1.0,
                    label='Tello with sensor and ds: %.1f min' % time_tello_sensor_ds_min_2[-1])
    ax_battery.plot(time_tello_sensor_no_ds_min, battery_level_tello_sensor_no_ds, color='k', linewidth=1.0,
                    label='Tello with sensor and no ds: %.1f min' % time_tello_sensor_no_ds_min[-1])
    ax_battery.plot(time_tello_sensor_no_ds_min_2, battery_level_tello_sensor_no_ds_2, color='k', linewidth=1.0,
                    label='Tello with sensor and no ds: %.1f min' % time_tello_sensor_no_ds_min_2[-1])

    ax_battery.set_xlabel('time in [min]')
    ax_battery.set_ylabel('battery level in [%]')
    ax_battery.set_title('Flight Time')
    ax_battery.legend()
    plt.xticks(np.arange(0, 12.1, step=1.0))
    plt.yticks(np.arange(0, 100.1, step=10.0))
    plt.grid(linewidth=.5)
    plt.show()


def main():
    rospy.init_node('battery_plot_node')
    
    if RECORD_BATTERY:
        sensor = BatteryLogPlot()
        rospy.spin()                  # keep process alive
    else:
        plot_battery()

if __name__ == '__main__':
    main()