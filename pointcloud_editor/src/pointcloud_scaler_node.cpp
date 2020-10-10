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

//include header
#include "pointcloud_scaler_node.h"



//********************************************************
// Call back to change parameter
//********************************************************
/*
void PointcloudScaler::paramCallback(pointcloud_editor::cloudConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure requested");

SCALE_ = config.WORLD_SALE_;
CONSOLE_OUTPUT_ = config.CONSOLE_OUTPUT_

}
*/

//********************************************************
//callback function to receive pointcloud
//********************************************************
void PointcloudScaler::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("Pointcloud received");
 //#####################################################################################
   //Container for original & scaled pointcloud
  //#####################################################################################
  sensor_msgs::PointCloud2 cloud_transformed;
  sensor_msgs::PointCloud2 cloud_publish;
  cloud_publish.header = cloud_msg->header;
  pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2; //PointCloud2's
  pcl::PCLPointCloud2ConstPtr cloud2Ptr(point_cloud2);
  ROS_INFO("Try to transform to world");
  pcl_ros::transformPointCloud("world", *cloud_msg, cloud_publish, tf_listener_);
  // Scle pointcloud
  Eigen::Matrix4f scale_matrix;
  scale_matrix << WORLD_SCALE_,   0,   			0,  	0,
          0,   			WORLD_SCALE_,    0,  	0,
          0, 				0,		WORLD_SCALE_,0,
          0,          		0,          0,       1;

  ROS_INFO("Try to scale");
  pcl_ros::transformPointCloud(scale_matrix, cloud_publish, cloud_publish);

  // Publish scaled cloud.
  pub_cloud_.publish(cloud_publish);
  ROS_INFO("Published!");
}




//********************************************************
//********************************************************
//Main function
//********************************************************
//********************************************************
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_scaler");
  ros::NodeHandle nh;
  ROS_INFO("NODE scaling runs");
  PointcloudScaler scale(nh);

  //init parameter server
  //dynamic_reconfigure::Server<pointcloud_editor::cloudConfig>::CallbackType f;

  //f = boost::bind(&PointcloudScaler::paramCallback,&cloud, _1, _2);
  //server.setCallback(f);

  while (ros::ok())
     ros::spin ();

   return 0;
}
