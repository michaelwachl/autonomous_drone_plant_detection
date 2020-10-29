#!/usr/bin/env python

import os
import pickle

import atexit
import datetime


class ControllerLog(object):
    def __init__(self):
        # current variables
        self.time_axis = []
        self.error_x = []
        self.error_y = []
        self.error_z = []
        self.error_yaw = []
        self.target_x = []
        self.target_y = []
        self.target_z = []
        self.target_yaw = []
        self.current_x = []
        self.current_y = []
        self.current_z = []
        self.current_yaw = []
        # at exit save plot
        atexit.register(self.save_plot)

    def save_plot(self):
        print('dumping variables')
        path = '%s/Documents/%s' % (os.getenv('HOME'),
                                   datetime.datetime.now().strftime('controller-tello-%Y-%m-%d_%H%M%S'))

        # save save variables as pickle
        pickle.dump([self.time_axis, self.error_x, self.error_y, self.error_z, self.error_yaw, self.target_x,
                     self.target_y, self.target_z, self.target_yaw, self.current_x, self.current_y, self.current_z,
                     self.current_yaw], file(path + '.pickle', 'w'))
        print('Variables saved')