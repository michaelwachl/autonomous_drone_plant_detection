# Record sensor data of SCD30
This modules is used to recored the sensor data into a pickle file. Is already has the basic implementation for live plots, which are currently not used.

## Usage
Start roscore if not already started
```
roscore
```
start node
```
rosrun environment_sensor_plot scd30_plot.py
```

## Subscripted
* **`/tello/scd30`** ', SCD30
* **`/tello/odom`** ', Odometry

## Output
Pickle file in 'Pictures' folder with path and sensor values of SCD30


# Plot sensor data

In file [scd30_plot_load.py](scripts/scd30_plot_load.py) edit pickle name and run script. (E.g. with Pycharm)


# Plot battery
This script is used to record and plot the battery level

## Usage
Set parameter RECOARD_BATTERY to True or False in order to record or plot the battery level during flight. This is done in script [tello_battery_plot.py](scripts/tello_battery_plot.py).

Start roscore if not already started
```
roscore
```
start node
```
rosrun environment_sensor_plot tello_battery_plot.py
```
