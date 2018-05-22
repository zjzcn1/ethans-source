//
// Created by ethan on 18-5-22.
//
#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
using namespace std;


class process_factory
{
public:
    process_factory()
    {   pub = nh.advertise<sensor_msgs::PointCloud2>("/processd_pointcloud", 1);
        sub = nh.subscribe("/velodyne_points",3,&process_factory::call_back,this);
        min_point<<-1.8,-0.8,0.0,1.0;//cube的两个对角点
        max_point<<-3.5,0.8,2.0,1.0;
        pcd_name1="cloud";
        pcd_name2="process";
        a=0;
    }
    std::string num2str(int i)
    {
        std::stringstream ss;
        ss<<i;
        return ss.str();
    }
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        //cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);
        if(a<1000)
        {
            a++;
            if (a%5==0)
            {
                std::string num;
                num=num2str(a);

                if(pcl::io::savePCDFileASCII (pcd_name1+num+".pcd", *cloud_in)>=0)//input pointcloud should be pcl::PointCloud<PointT> only,rather than others
                {std::cerr << "Saved  " << pcd_name1<<num<<".pcd"<< std::endl;}

                nh.getParam("x1",x1);
                nh.getParam("y1",y1);
                nh.getParam("z1",z1);
                nh.getParam("x2",x2);
                nh.getParam("y2",y2);
                nh.getParam("z2",z2);
                nh.getParam("negative",negative);
                min_point<<x1,y1,z1,1.0;//cube的两个对角点
                max_point<<x2,y2,z2,1.0;

                clipper.setMin(min_point);
                clipper.setMax(max_point);
                clipper.setInputCloud(cloud_in);
                clipper.filter(*cloud_clippered);
                clipper.setNegative(negative);//默认为false

                //可选设置
                seg.setOptimizeCoefficients (false);
                //必须设置
                seg.setModelType (pcl::SACMODEL_PLANE);//模型类型
                seg.setMethodType (pcl::SAC_RANSAC);//随机参数估计方法
                ros::param::get("dis",dis);
                ros::param::param<float>("dis", dis, 0.13);
                seg.setDistanceThreshold (dis);//距离阈值
                seg.setInputCloud (cloud_clippered);//输入点云
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
                extract.setInputCloud (cloud_clippered);//从哪里抽取
                extract.setIndices (inliers);//创建抽取出来的点的索引（编号）


                extract.setNegative (true);//将平面保留还是去除
                extract.filter (*cloud_planared);//滤波（抽取）后结果送往cloud_mid
                if(cloud_planared->size() > 0)
                {
                    if(pcl::io::savePCDFileASCII (pcd_name2+num+".pcd", *cloud_planared)>=0)//input pointcloud should be pcl::PointCloud<PointT> only,rather than others
                    {std::cerr << "Saved  " << pcd_name2<<num<<".pcd"<< std::endl;}
                }

                pcl::toROSMsg(*cloud_planared,cloud_out);

                cloud_out.header.stamp = ros::Time::now();
                cloud_out.header.frame_id = "/lidar";
                pub.publish(cloud_out);
                //cout<<"I PROCESSED IT !"<<endl;




            }


        }


    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    //clip
    pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clippered {new pcl::PointCloud<pcl::PointXYZ>};
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    float x1,y1,z1;
    float x2,y2,z2;
    bool negative;
    //planar
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planared {new pcl::PointCloud<pcl::PointXYZ>};
    //模型系数对象coefficient和点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    //创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    //距离阈值
    float dis;
    //pcd name
    std::string pcd_name1;
    std::string pcd_name2;
    int a;
};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"clip_box");
    process_factory process_factory1;
    ros::spin();
}


