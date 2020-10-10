/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
*Author: Michael Wachl
*Year: 2018
*Course: Leistungskurs C++
*Group: 8
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
* This class is used to detect several objects from a registered pointcloud
and publish them.
------------------------------------------------------------------------------*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include <sstream>
#include <string>
#include <iostream>
#include <time.h>
#include "wachl_3d_object_detection/object.h"
#include "wachl_3d_object_detection/objectArray.h"

//pcl specific
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
//tf
#include <tf/transform_listener.h>

//for dyn reconfig
#include <wachl_3d_object_detection/detectionConfig.h>
#include <dynamic_reconfigure/server.h>

ros::Publisher pub,pub2;
typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace pcl;

class PointCloudFilter
{
public:
   explicit PointCloudFilter(ros::NodeHandle nh)
        : nh_(nh)
    {
	 //subscribe and publish topics
	   sub_cloud_ = nh_.subscribe ("/scaled_cloud", 1, &PointCloudEdit::cloud_cb, this);
       pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
    }
private:
    //Visiualization param
    bool VISIU_, CONSOLE_OUTPUT_;
    //filter param
    float LEAF_SIZE_,Y_LIM_MIN_,Y_LIM_MAX_,Z_LIM_MIN_,Z_LIM_MAX_,CLUSTER_TOLERANCE_,PLANE_RATIO_,ZYLINDER_RATIO_,MAX_WIDTH_,MAX_DEPTH_;
    int CLUSTER_MIN_,CLUSTER_MAX_;
    float ROBO_HEIGHT_MIN_,ROBO_HEIGHT_MAX_,ROBO_WIDTH_MIN_,ROBO_WIDTH_MAX_;
    //ros and tf variables
    ros::NodeHandle nh_;
    ros::Publisher pub_,pub_cloud_,pub_closest_puck_;
    ros::Subscriber sub_cloud_;
    tf::TransformListener tf_listener_;

    //funcitons
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
public:
    //callback function for dyn. reconfigure
    void paramCallback(wachl_3d_object_detection::detectionConfig &config, uint32_t level);
};
 //end of class defintion


