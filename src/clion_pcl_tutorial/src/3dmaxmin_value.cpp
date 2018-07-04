//
// Created by ethan on 18-6-26.
//
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
using namespace std;
int
main(int argc,char **argv)
{

    //自己填充一个点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width  = 5; //设置宽度
    cloud->height = 1; //设置高度
    cloud->points.resize (cloud->width * cloud->height); //变形，无序
    for (size_t i = 0; i < cloud->points.size (); ++i) //设置cloud_a中点的坐标（随机数）
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    //打印出点云
    std::cerr << "Cloud: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i) //打印cloud_a的点坐标信息
    std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y <<" "<< cloud->points[i].z << std::endl;
    //定义存放三轴上各自最小值的容器和存放最大值的容器
//    pcl::PointXYZ min;
//    pcl::PointXYZ max;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D(*cloud,min,max);//pcl库函数，用于求最值

    cout<<"min.z = "<<min[2]<<"\n"<<endl;
    cout<<"max.z = "<<max[2]<<"\n"<<endl;

}




