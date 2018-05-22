#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//ICP
#include <pcl/registration/icp.h>

int
main(int argc, char** argv) {
    // *****Initialize ROS
    ros::init (argc, argv, "ICP");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~my_namespace");
    //*****load two pcd files
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda_ptr (new pcl::PointCloud<pcl::PointXYZ>);//创建点云指针，存储点坐标xyz
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>      Final;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/pcl_2_bag/pcl_2_pcd1.pcd", *clouda_ptr) == -1)
    { PCL_ERROR ("Couldn't read file pcl_2_pcd1.pcd ^.^\n");
        return (-1);}
    else
    {std::cerr << "cloud_a loaded successfully~!"<< std::endl;}
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/pcl_2_bag/pcl_2_pcd2.pcd", *cloudb_ptr) == -1)
    { PCL_ERROR ("Couldn't read file pcl_2_pcd2.pcd ^.^\n");
        return (-1);}
    else
    {std::cerr << "cloud_b loaded successfully~!"<< std::endl;}
    

    //*****icp
    float MaxCorrespondenceDistance;//忽略在此距离之外的点
    float TransformationEpsilon;//这个值一般设为1e-6或者更小
    int EuclideanFitnessEpsilon;//前后两次迭代误差的差值
    int MaximumIterations;//迭代次数，几十上百都可能出现
    pn.param<int>("EuclideanFitnessEpsilon", EuclideanFitnessEpsilon, 1);
    pn.param<int>("MaximumIterations", MaximumIterations, 20);
    pn.param<float>("MaxCorrespondenceDistance", MaxCorrespondenceDistance, 1);
    pn.param<float>("TransformationEpsilon", TransformationEpsilon, 1e-6);
    pn.getParam("EuclideanFitnessEpsilon",EuclideanFitnessEpsilon);
    pn.getParam("MaximumIterations",MaximumIterations);
    pn.getParam("MaxCorrespondenceDistance",MaxCorrespondenceDistance);
    pn.getParam("TransformationEpsilon",TransformationEpsilon);


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(clouda_ptr);
    icp.setInputTarget(cloudb_ptr);
    
    icp.align(Final);//拼接结果输出到Final
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //*****Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 clouda_ros;
    sensor_msgs::PointCloud2 cloudb_ros;
    sensor_msgs::PointCloud2 cloudc_ros;
    pcl::toROSMsg (*clouda_ptr, clouda_ros);
    pcl::toROSMsg (*cloudb_ptr, cloudb_ros);
    pcl::toROSMsg (Final, cloudc_ros);
    
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
    printf("MaximumIterations:  %d\n\n", MaximumIterations);
    r.sleep();
    }
    return 0;
}
