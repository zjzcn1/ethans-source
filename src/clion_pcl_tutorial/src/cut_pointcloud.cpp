//
// Created by ethan on 18-5-9.
//
#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
//filter
#include <pcl/filters/passthrough.h>
using namespace std;
class cut_pointcloud
{
public:
    cut_pointcloud()
    {pub = nh.advertise<sensor_msgs::PointCloud2>("/cuted_pointcloud", 1);
     sub = nh.subscribe("/velodyne_points",3,&cut_pointcloud::call_back,this); }
     void call_back(const sensor_msgs::PointCloud2ConstPtr input)
     {
         cout<<"I RECEIVED INPUT !/n"<<endl;
         pcl::fromROSMsg(*input,*cloud_in);
         pass.setInputCloud (cloud_in);//has to be a pointer
         pass.setFilterFieldName ("z");
         pass.setFilterLimits (0.0, 0.5);
         pass.setFilterLimitsNegative (false);
         pass.filter (*cloud_filtered);
         pcl::toROSMsg(*cloud_filtered,cloud_out);
         cloud_out.header.stamp = ros::Time::now();
         cloud_out.header.frame_id = "horizontal_vlp16_link";
         pub.publish(cloud_out);
         cout<<"I cuted it !/n"<<endl;
     }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZ>};
    sensor_msgs::PointCloud2 cloud_out;
    pcl::PassThrough<pcl::PointXYZ> pass;
};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"cut_pointcloud");
    cut_pointcloud cut_pointcloud1;
    ros::spin();
}