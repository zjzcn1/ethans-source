//
// Created by ethan on 18-6-4.
//

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
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

int
main(int argc,char **argv)
{

    //输入
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    //建立树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    int neighbor=5;
    //法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //区域生长分割器
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    //聚类的索引队列
    std::vector <pcl::PointIndices> inliers ;

    //读入数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/pcl_2_bag/cloud2.pcd", *cloud_in) == -1)
    {
        PCL_ERROR ("Couldn't read file cloud9.pcd ~^.^~\n");
        return (-1);
    }

    // 计算法向量
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_in);
    ros::param::get("neighbor",neighbor);
    ros::param::param<int>("neighbor", neighbor, 5);
    ne.setKSearch (neighbor);
    ne.compute (*cloud_normals);

    //聚类分割
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_in);
    //reg.setIndices (indices);
    reg.setInputNormals (cloud_normals);

    //设置限制条件及先验知识
    reg.setMinClusterSize (5);
    reg.setMaxClusterSize (1000000);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    reg.extract (inliers);


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }
}