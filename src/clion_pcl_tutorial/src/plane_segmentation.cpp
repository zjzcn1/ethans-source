//
// Created by ethan on 18-5-10.
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

#include <pcl/filters/extract_indices.h>

using namespace std;


class plane_seg
{
public:
    plane_seg()
    {pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_seg", 1);//only planar
     pub2 = nh.advertise<sensor_msgs::PointCloud2>("/plane_seg2", 1);//no planar
     sub = nh.subscribe("/clip_boxed_pointcloud",3,&plane_seg::call_back,this); }
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);
        //可选设置
        seg.setOptimizeCoefficients (true);
        //必须设置
        seg.setModelType (pcl::SACMODEL_PLANE);//模型类型
        seg.setMethodType (pcl::SAC_RANSAC);//随机参数估计方法
        ros::param::get("dis",dis);
        ros::param::param<float>("dis", dis, 0.01);
        seg.setDistanceThreshold (dis);//距离阈值
        seg.setInputCloud (cloud_in);//输入点云
        seg.segment (*inliers, *coefficients);
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
        pub.publish(cloud_out);

        extract.setNegative (true);
        extract.filter (*cloud_mid2);
        pcl::toROSMsg(*cloud_mid2,cloud_out2);
        pub2.publish(cloud_out2);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid2 {new pcl::PointCloud<pcl::PointXYZ>};
    sensor_msgs::PointCloud2 cloud_out;
    sensor_msgs::PointCloud2 cloud_out2;
    //模型系数对象coefficient和点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    //创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    //距离阈值
    float dis;
};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"plane_seg");
    plane_seg plane_seg1;
    ros::spin();
}
