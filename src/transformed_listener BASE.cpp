#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
#include <pcl/common/projection_matrix.h>

// declare publisher to use in subscriber callback
ros::Publisher mod_transformed_pub;


//    use this for aid in your journey:
//    http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
//this is the operation that occurs when the subscriber receives the pointcloud from the SICK publisher
void point_transformed_CallBack(const sensor_msgs::PointCloud2& transformed_pointcloud){
  
  //create a new blank pointcloud object for math operations  
  pcl::PointCloud<pcl::PointXYZ>::Ptr recpc (new pcl::PointCloud<pcl::PointXYZ> ());
  
  //convert transformed_pointcloud (PointCloud2 msg) to recpc (PointCloud obj)
  pcl::fromROSMsg(transformed_pointcloud, *recpc);


  // leaving this here as an example of how to print non String data
  /*
  float x = recpc->points[810].x;
  float y = recpc->points[810].y;
  float z = recpc->points[810].z;
  

  std::cerr << "XYZ: " << x << " "
  		       << y << " "
                       << z << std::endl;
  
  //std::cerr << "length of points: " << recpc->points.size() << std::endl;
  */


  // taken from pointclouds.org documentation about planar segmentation
  // this is segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE); // looking for lines in the data
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01); // perhaps tune this to have higher room for error?
  
  //set the pointcloud that will be source for segmenter
  seg.setInputCloud (recpc);
  
  //segment the pointcloud, placing results in inliers and coefficients
  seg.segment (*inliers, *coefficients);
  
  //set up blank pointcloud for copying over the segmented data
  pcl::PointCloud<pcl::PointXYZ> segpc;
  
  //make a copy of the pointcloud recpc, using the inliers indices to determine 
  //which points carry over to new cloud and get visualized
  pcl::copyPointCloud(*recpc, *inliers, segpc);  

  //blank ROS PointCloud2 message to be filled in ros conversion
  sensor_msgs::PointCloud2 lined_pointcloud;  

  //convert segpc PointCloud into a ROS message PointCloud2 for publishing
  pcl::toROSMsg(segpc, lined_pointcloud);

  //publish the segmented pointcloud
  mod_transformed_pub.publish(lined_pointcloud);


}

//This is where we declare the node and its various interactions with other nodes
int main(int argc, char **argv){
  
  //initialize the node using arguments and with the name lidar_listener
  ros::init(argc, argv, "tranformed_listener");
  
  //set up the node's subscriber and specify what topic it is subscribing to (here, "cloud"), the queue of waiting data it can handle (here, 1 for best results), and the operation that will be run when the message is received (here, pointCloudCallBack, the operation we declared at the top of the file) 
  ros::NodeHandle nh;
  ros::Subscriber transformed_sub = nh.subscribe("modified_pointcloud", 1, point_transformed_CallBack);
  
  //setup the node's publisher and specify the topic it will publish, with a queue of 1 for best results
  mod_transformed_pub = nh.advertise<sensor_msgs::PointCloud2>("modified_transformed", 1);
  
  //keep running node to obtain messages
  ros::spin();

  return 0;
}