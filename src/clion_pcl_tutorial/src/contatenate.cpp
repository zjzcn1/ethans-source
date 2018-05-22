#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


int
main(int argc, char** argv) {
    // *****Initialize ROS
    ros::init (argc, argv, "contatenate");
    ros::NodeHandle nh;

    //*****load two pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda_ptr (new pcl::PointCloud<pcl::PointXYZ>);//创建点云指针，存储点坐标xyz
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudc_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/cloud_processed/cloud1.pcd", *clouda_ptr) == -1)
    {
        PCL_ERROR ("Couldn't read file pcl_2_pcd1.pcd ^.^\n");
        return (-1);
    }
    std::cerr << "size of a : " << clouda_ptr->width*clouda_ptr->height<< std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/cloud_processed/cloud9.pcd", *cloudb_ptr) == -1)
    {
        PCL_ERROR ("Couldn't read file pcl_2_pcd2.pcd ^.^\n");
        return (-1);
    }
    std::cerr << "size of b : " << cloudb_ptr->width*cloudb_ptr->height<< std::endl;

    //*****contatenate
    *cloudc_ptr=*clouda_ptr+*cloudb_ptr;// cloud_a + cloud_b 意思是cloud_c包含了a和b中的点，c的点数 = a的点数+b的点数
    std::cerr << "size of c : " << cloudc_ptr->width*cloudc_ptr->height<< std::endl;

    //*****Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 clouda_ros;
    sensor_msgs::PointCloud2 cloudb_ros;
    sensor_msgs::PointCloud2 cloudc_ros;
    pcl::toROSMsg (*clouda_ptr, clouda_ros);
    pcl::toROSMsg (*cloudb_ptr, cloudb_ros);
    pcl::toROSMsg (*cloudc_ptr, cloudc_ros);
    
    //*****publish these three clouds
    ros::Publisher pubc;
    ros::Publisher pubb;
    ros::Publisher puba;
    pubc = nh.advertise<sensor_msgs::PointCloud2> ("cloudc", 1);
    pubb = nh.advertise<sensor_msgs::PointCloud2> ("cloudb", 1);
    puba = nh.advertise<sensor_msgs::PointCloud2> ("clouda", 1);

    ros::Rate r(3);
    while(ros::ok())
    {
    cloudc_ros.header.stamp = ros::Time::now();
    cloudc_ros.header.frame_id = "sensor_framec";
    cloudb_ros.header.stamp = ros::Time::now();
    cloudb_ros.header.frame_id = "sensor_framec";
    clouda_ros.header.stamp = ros::Time::now();
    clouda_ros.header.frame_id = "sensor_framec";
    pubc.publish(cloudc_ros);
    pubb.publish(cloudb_ros);
    puba.publish(clouda_ros);
    r.sleep();
    }
    return 0;
}

