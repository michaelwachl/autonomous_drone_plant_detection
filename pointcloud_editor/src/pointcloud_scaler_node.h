/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
* Master Thesis
*Author: Michael Wachl
*Year: 2020
*Company: Siemens AG
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
* This class is used to scale a pointcloud
and publish it.
------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include <sstream>
#include <string>
#include <iostream>
#include <time.h>

//pcl specific
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
//tf
#include <tf/transform_listener.h>

//for dyn reconfig
//#include <pointclould_editor/cloudConfig.h>
#include <dynamic_reconfigure/server.h>
//include eigen
#include <Eigen/Dense>

ros::Publisher pub,pub2;
typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace pcl;

class PointcloudScaler
{
public:
   explicit PointcloudScaler(ros::NodeHandle nh)
        : nh_(nh)
    {
	 //subscribe and publish topics
         sub_cloud__ = nh_.subscribe ("/orb_slam2_mono/map_points", 1, &PointcloudScaler::cloud_cb, this);
         pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("scaled_cloud", 1);
    }
private:
    //Visiualization param
    bool CONSOLE_OUTPUT_;
    //filter param
    float WORLD_SCALE_ = 2.0;

    //ros and tf variables
    ros::NodeHandle nh_;
    ros::Publisher pub_cloud_;
    ros::Subscriber sub_cloud__;
    tf::TransformListener tf_listener_;

    //funcitons
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
public:
    //callback function for dyn. reconfigure
    //void paramCallback(pointclould_editor::clouldConfig &config, uint32_t level);
};
 //end of class defintion


