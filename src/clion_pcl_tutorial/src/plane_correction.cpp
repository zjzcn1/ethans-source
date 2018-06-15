//
// Created by ethan on 18-6-13.
//
#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
using Eigen::MatrixXd;
using Eigen::Vector3f;


Eigen::Matrix4f CreateRotateMatrix(Vector3f before,Vector3f after)
{
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Vector3f p_rotate =before.cross(after);
    p_rotate.normalize();

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) +p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

    return rotationMatrix;
}

int
main(int argc,char** argv) {

    ros::init(argc, argv, "resterization");
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub2;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/planar_correction", 1);
    pub2= nh.advertise<sensor_msgs::PointCloud2>("/raw_pointcloud", 1);
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    sensor_msgs::PointCloud2 cloud_out2;//转换为ros格式

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ethan/bags/pcl_2_bag/cloud9.pcd", *cloud_in) == -1) {
        PCL_ERROR ("Couldn't read file cloud9.pcd ^.^\n");
        return (-1);
    }

    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType ( pcl::SACMODEL_PLANE );
    plane_seg.setMethodType ( pcl::SAC_RANSAC );
    plane_seg.setDistanceThreshold ( 0.3 );
    plane_seg.setInputCloud ( cloud_in );
    plane_seg.segment (*plane_inliers, *plane_coefficients);


    Vector3f  planar_normal;
    Vector3f  z_axis;
    z_axis<<0,0,1;

    planar_normal<<plane_coefficients->values[0],plane_coefficients->values[1],plane_coefficients->values[2];
    Eigen::Matrix4f rotation=CreateRotateMatrix(planar_normal,z_axis);
    cout<<rotation;
    pcl::transformPointCloud(*cloud_in, *cloud_final, rotation);

//    pcl::visualization::CloudViewer viewer("resterization");//可视化窗口
//    viewer.showCloud(cloud_final);//看一下其中某个栅格
//    while(!viewer.wasStopped())
//    {    }
    pcl::toROSMsg(*cloud_final,cloud_out);
    pcl::toROSMsg(*cloud_in,cloud_out2);
    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = " ";
    cloud_out2.header.stamp = cloud_out.header.stamp;
    cloud_out2.header.frame_id = " ";
    while(ros::ok())
    {
     pub.publish(cloud_out);
     pub2.publish(cloud_out2);
    }


    return 0;
}


