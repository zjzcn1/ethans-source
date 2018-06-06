//
// Created by ethan on 18-6-4.
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
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
using namespace std;

int
main(int argc,char **argv)
{
    ros::init(argc,argv,"plane_seg2");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    int neighbor=5;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //模型系数对象coefficient和点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //创建分割对象
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    float weight=0.0;
    //创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid2 (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud_out;
    sensor_msgs::PointCloud2 cloud_out2;
    ros::Publisher pub;
    ros::Publisher pub2;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_seg", 1);//only planar
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/plane_seg2", 1);//no planar


    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/pcl_2_bag/cloud9.pcd", *cloud_in) == -1)
    {
        PCL_ERROR ("Couldn't read file pcl_2_pcd1.pcd ^.^\n");
        return (-1);
    }

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_in);
    ros::param::get("neighbor",neighbor);
    ros::param::param<int>("neighbor", neighbor, 5);
    ne.setKSearch (neighbor);
    ne.compute (*cloud_normals);

    //可选设置
    seg.setOptimizeCoefficients (true);
    //必须设置
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);//模型类型
    ros::param::get("weight",weight);
    ros::param::param<float>("weight", weight, 0.1);
    seg.setNormalDistanceWeight (weight);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_in);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers, *coefficients);
    std::cerr << "Plane coefficients: " << *coefficients << std::endl;

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        //return (-1);
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "//打印平面模型参数ax+by+cz+d=0
              <<coefficients->values[1] << " "
              <<coefficients->values[2] << " "
              <<coefficients->values[3] <<std::endl;

    extract.setInputCloud (cloud_in);//从哪里抽取
    extract.setIndices (inliers);//创建抽取出来的点的索引（编号）
    extract.setNegative (false);//将平面保留还是去除
    extract.filter (*cloud_mid);//滤波（抽取）后结果送往cloud_mid
    pcl::toROSMsg(*cloud_mid,cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = "sensor_framec";


    extract.setNegative (true);
    extract.filter (*cloud_mid2);
    pcl::toROSMsg(*cloud_mid2,cloud_out2);
    cloud_out2.header.stamp = ros::Time::now();
    cloud_out2.header.frame_id = "sensor_framec";

    ros::Rate r(3);
    while(ros::ok())
    {
        pub.publish(cloud_out);
        pub2.publish(cloud_out2);
        r.sleep();
    }
}

