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
using namespace std;
class cut_pointcloud
{
public:
    cut_pointcloud()
    {pub = nh.advertise<sensor_msgs::PointCloud2>("/cuted_pointcloud", 1);
     sub = nh.subscribe("/velodyne_points",3,&cut_pointcloud::call_back,this); }
     void call_back(const sensor_msgs::PointCloud2ConstPtr input)
     {
	 cout<<"I RECEIVED INPUT !"<<endl;
	 pcl::fromROSMsg(*input,*cloud_in);


 	 pcl::toROSMsg(*cloud_final,cloud_out);
         cloud_out.header.stamp = ros::Time::now();
         cloud_out.header.frame_id = " ";
         pub.publish(cloud_out);
         cout<<"I PROCESSED IT !"<<endl;
     }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};//加工后的pcl格式
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"cut_pointcloud");
    cut_pointcloud cut_pointcloud1;
    ros::spin();
}
