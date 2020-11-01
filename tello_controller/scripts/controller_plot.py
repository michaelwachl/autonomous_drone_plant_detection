#!/usr/bin/env python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import pickle


"""
----------------------------------------------------------------------------------------------------------
Plot step responses of X, Y, Z and Yaw PID
----------------------------------------------------------------------------------------------------------
"""
pickle_name = "controller-tello-YAW-PID-3-0-01.pickle"
pickle_path = '%s/Documents/%s' % (os.getenv('HOME'), pickle_name)

# load variables
time_axis, error_x, error_y, error_z, error_yaw, target_x, target_y, target_z, target_yaw, current_x, current_y, current_z, current_yaw= pickle.load(file(pickle_path))
#time_axis, error_x, error_y, error_z, error_yaw, target_x, target_y, target_z, target_yaw = pickle.load(file(pickle_path))

# time in min
start_time = time_axis[0]
time_min = [x / 60.0 for x in time_axis]


# plot
# fig_step_resp, (ax_x, ax_y, ax_z, ax_yaw) = plt.subplots(4)
fig_step_resp, ax_yaw = plt.subplots(1)
# ax_battery.scatter(time_tello_min, battery_level_tello, marker='.', c='b', linewidths=0.01, alpha=0.1)
# , 'o', color='b', label='Tello only')
# Fit with polyfit
# p = np.poly1d(np.polyfit(time_tello_min, battery_level_tello, 3))
"""
# ax_battery.plot(time_tello_min, p(time_tello_min), color='b', linewidth=2.0, label='Tello only')
ax_x.plot(time_axis, target_x, color='b', linewidth=1.0, label='Target X')
ax_x.plot(time_axis, current_x, color='g', linewidth=1.0,  label='Current X')
ax_x.plot(time_axis, error_x, color='r', linewidth=1.0,  label='Error X')


ax_y.plot(time_axis, target_y, color='b', linewidth=1.0, label='Target Y')
ax_y.plot(time_axis, current_y, color='g', linewidth=1.0,  label='Current Y')
ax_y.plot(time_axis, error_y, color='r', linewidth=1.0,  label='Error Y')

ax_z.plot(time_axis, target_z, color='b', linewidth=1.0, label='Target Z')
ax_z.plot(time_axis, current_z, color='g', linewidth=1.0,  label='Current Z')
ax_z.plot(time_axis, error_z, color='r', linewidth=1.0,  label='Error Z')
"""
ax_yaw.plot(time_axis, target_yaw, color='b', linewidth=1.0, label='Target Yaw')
ax_yaw.plot(time_axis, current_yaw, color='g', linewidth=1.0,  label='Current Yaw')
ax_yaw.plot(time_axis, error_yaw, color='r', linewidth=1.0,  label='Error Yaw')
"""
ax_x.set_xlabel('Time in [s]')
ax_x.set_ylabel('Target and Current Error in [m]')
ax_x.set_title('X-PID')
ax_x.legend()

ax_y.set_xlabel('Time in [s]')
ax_y.set_ylabel('Target, Current and Error of Y in [m]')
ax_y.set_title('Y-PID')
ax_y.legend()

ax_z.set_xlabel('Time in [s]')
ax_z.set_ylabel('Target, Current and Error of Z in [m]')
ax_z.set_title('Z-PID')
ax_z.legend()
"""
ax_yaw.set_xlabel('Time in [s]')
ax_yaw.set_ylabel('Target, Current and Error of Yaw in [m]')
ax_yaw.set_title('Yaw-PID')
ax_yaw.legend()



#plt.xticks(np.arange(0, 12.1, step=1.0))
#plt.yticks(np.arange(0, 100.1, step=10.0))
# ax_x.grid(linewidth=.5)
# ax_y.grid(linewidth=.5)
# ax_z.grid(linewidth=.5)
ax_yaw.grid(linewidth=.5)
plt.show()


"""
----------------------------------------------------------------------------------------------------------
Plot landing distributions
----------------------------------------------------------------------------------------------------------
"""

fig_land = plt.figure()
ax_land = fig_land.add_subplot(1, 1, 1)
# Move left y-axis and bottim x-axis to centre, passing through (0,0)
ax_land.spines['left'].set_position('center')
ax_land.spines['bottom'].set_position('center')

# Eliminate upper and right axes
ax_land.spines['right'].set_color('none')
ax_land.spines['top'].set_color('none')

# Show ticks in the left and lower axes only
ax_land.xaxis.set_ticks_position('bottom')
ax_land.yaxis.set_ticks_position('left')


# Plot landing distribution
x_landing_rect_empty = [1.0, 0.4, -0.3, -1.0]
y_landing_rect = [0.4, 1.4, 0.3, -0.4]
yaw_landing_rect = []

plt.scatter(x_landing_rect_empty, y_landing_rect)
plt.plot(x_landing_rect_empty, y_landing_rect)
plt.show()
