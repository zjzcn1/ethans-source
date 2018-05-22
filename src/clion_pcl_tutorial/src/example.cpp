#include <ros/ros.h>
 // PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)//cloud_msg[1]  input[2]
{
//
//    // Create a container for the data.
//   sensor_msgs::PointCloud2 output;
//
//    // Do data processing here...
//   output = *input;
//
//   // Publish the data.
//   pub.publish (output);


//   down_sampling[1]
//    // Container for original & filtered data
//    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//    pcl::PCLPointCloud2 cloud_filtered;
//
//    // Convert to PCL data type
//    pcl_conversions::toPCL(*cloud_msg, *cloud);
//
//    // Perform the actual filtering
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloudPtr);
//    sor.setLeafSize (0.1, 0.1, 0.1);
//    sor.filter (cloud_filtered);
//
//    // Convert to ROS data type
//    sensor_msgs::PointCloud2 output;
//    pcl_conversions::fromPCL(cloud_filtered, output);
//
//    // Publish the data
//    pub.publish (output);


////Plane model segmentation[2]
 //Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);
}

int main (int argc, char** argv)
{
   // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);//[1]
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);//segmentation[2]
  // Spin
  ros::spin ();
 }



























