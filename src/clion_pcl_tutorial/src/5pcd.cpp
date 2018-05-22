#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


int
main(int argc, char** argv) {
    // *****Initialize ROS
    ros::init (argc, argv, "five_pcd");
    ros::NodeHandle nh;

    // *****N pcds
    //int N=int(*argv[1])-0x30;
    int N=atoi(argv[1]);  
    // *****load 5 pcd files
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr[6] (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>  cloud[N+1];
    char ch=(char)(int('0'));
    std::cerr << N << std::endl;
    std::string pcd_name("/home/ethan/bags/pcl_2_bag/pcl_2_pcd");
    for(int i=0;i<N;i++)
    {
      ch++;
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> ((pcd_name+ch+".pcd"), cloud[i]) == -1)
      {
        PCL_ERROR ("Couldn't read %d ^.^\n",i);
        return (-1);
      }
      std::cerr << "size of "<<i+1 <<":"<< cloud[i].width*cloud[i].height<< std::endl;

      cloud[N]+=cloud[i];
      std::cerr << "size of contatenated:"<< cloud[N].width*cloud[N].height<< std::endl;
    }
    //*****Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg (cloud[N], cloud_ros);

    //*****publish these three clouds
    ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud", 1);
    ros::Rate r(1);
    while(ros::ok())
    {
      cloud_ros.header.stamp = ros::Time::now();
      cloud_ros.header.frame_id = "sensor_frame";
      pub.publish(cloud_ros);
      r.sleep();
    }

    return 0;
}

