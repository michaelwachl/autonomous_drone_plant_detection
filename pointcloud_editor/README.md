# Point cloud editor
IN DEVELOPMENT!
This package is used to scale and filter point clouds for object detection. This package is still in development. However, the Point cloud scaler already works well.

# Node: pointcloud_scaler_node
Subscribes to point clould of ORBSLAM2

## Usage
```
roscore
```
start node
```
rosrun pointclould_editor pointcloud_scaler_node
```

## Subscribed topics
Subscribes to the point clould of ORBSLAM2

* **`/orb_slam2_mono/map_points`** sensor_msgs::PointCloud2ConstPtr

## Published topics
Published the (true) scale point clould

* **`/pointcloud_scaler_node/scaled_cloud`** sensor_msgs::PointCloud2


# Node: pointcloud_filter_node
Filters the scaled point clould 
## Usage
```
roscore
```
start node
```
rosrun pointclould_editor pointcloud_filter_node
```

# Launch
```
roslaunch pointclould_editor pointcloud_editor.launch
```

## config
`cloud.cfg` parameters to reconfigure in run time  
`cloud_params.yaml` to ajust clould filtering 


# Libaries:
- standard ROS melodic packages
- pcl libaries 
- Eigen3 libaries 




