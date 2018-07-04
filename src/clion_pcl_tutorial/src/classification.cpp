//
// Created by ethan on 18-6-14.
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
#include <pcl/features/normal_3d.h>
using Eigen::MatrixXd;
using Eigen::Vector3f;

class Grid
{
public:
    bool  road;
    bool  bulge;
    bool  hollow;
    bool  empty;
    float h_mean;
    float h_square;
    float intensity_mean;
    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud {new pcl::PointCloud<pcl::PointXYZI>};
};
//旋转矩阵
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
//ROS节点
    ros::init(argc, argv, "resterization");
    ros::NodeHandle nh;

//读取pcd
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/ethan/bags/pcl_2_bag/cloud9.pcd", *cloud_in) == -1) {
        PCL_ERROR ("Couldn't read file cloud9.pcd ^.^\n");
        return (-1);
    }
//检测地面
    pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
    pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType ( pcl::SACMODEL_PLANE );
    plane_seg.setMethodType ( pcl::SAC_RANSAC );
    plane_seg.setDistanceThreshold ( 0.3 );
    plane_seg.setInputCloud ( cloud_in );
    plane_seg.segment (*plane_inliers, *plane_coefficients);

//地面校准
    Vector3f  planar_normal;
    Vector3f  z_axis;
    z_axis<<0,0,1;
    planar_normal<<plane_coefficients->values[0],plane_coefficients->values[1],plane_coefficients->values[2];
    Eigen::Matrix4f rotation=CreateRotateMatrix(planar_normal,z_axis);
    cout<<rotation<<"\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_correction(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_in, *cloud_correction, rotation);

//棋盘格型栅格化
    int column=25;
    int row=15;
    float grid_size_x=0.1;
    float grid_size_y=0.1;
    int j=0;
    int i=0;
    double threshold;
    int intensity;
    ros::param::get("column",column);
    ros::param::param<int>("column", column, 500);
    ros::param::get("row",row);
    ros::param::param<int>("row", row, 300);
    ros::param::get("intensity",intensity);
    ros::param::param<int>("intensity", intensity, 20);
    ros::param::get("row",row);
    grid_size_x=50.0/column;
    grid_size_y=30.0/row;
    ros::param::get("threshold",threshold);
    ros::param::param<double>("threshold", threshold, 0.00010);
    Grid grid[column][row];
    cout<<"in "<<cloud_correction->size()<<endl;
//栅格化
    for (int m=0;m<cloud_correction->size();m++)//这些点一个一个看
    {
        for( i=-column/2;i<column/2;i++)
        {
            if (((cloud_correction->points[m].x>i*grid_size_x)&&(cloud_correction->points[m].x<i*grid_size_x+grid_size_x)))//如果x在某范围内
            {
                for( j=-row/2;j<row/2;j++)
                {

                    if((cloud_correction->points[m].y>j*grid_size_y)&&(cloud_correction->points[m].y<j*grid_size_y+grid_size_y))//如果y在某范围内
                    {

                        grid[i+column/2][j+row/2].grid_cloud->points.push_back(cloud_in->points[m]);

                    }
                }
            }
        }

    }

//栅格特征求取
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
    int a,b=0;
    for(a=0;a<column;a++)
    {
        for(b=0;b<row;b++)
        {

//高度平均值
            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_mean+=(grid[a][b].grid_cloud->points[i].z)/(grid[a][b].grid_cloud->size());

            }


//高度方差

            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_square+=pow(grid[a][b].grid_cloud->points[i].z-grid[a][b].h_mean,2);

            }
//反射率平均值
            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].intensity_mean+=(grid[a][b].grid_cloud->points[i].intensity)/grid[a][b].grid_cloud->size();

            }
//栅格法向量
            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].intensity_mean+=(grid[a][b].grid_cloud->points[i].intensity)/grid[a][b].grid_cloud->size();

            }
//筛选条件
            if((grid[a][b].h_mean<-0.7)&&(grid[a][b].intensity_mean<intensity))//(grid[a][b].h_square<threshold)&&
            {

                *cloud_final+=*grid[a][b].grid_cloud;
            }


        }
    }

//蛛网型栅格化

//t R=150;
//    int TH=72;
//    float grid_size_r=0.1;
//    float grid_size_th=2*M_PI/TH;
//
//    int j=0;
//    int i=0;
//    double threshold;
//
//    ros::param::get("R",R);
//    ros::param::param<int>("R", R, 150);
//    ros::param::get("TH",TH);
//    ros::param::param<int>("TH", TH, 72);
//
//    grid_size_r=50.0/R;
//    grid_size_th=2*M_PI/TH;
//    ros::param::get("threshold",threshold);
//    ros::param::param<double>("threshold", threshold, 0.00010);
//
//    Grid grid[R][TH];
//    float r=0;
//    float th;
//    for (int m=0;m<cloud_correction->size();m++)//这些点一个一个看
//    {
//        r=pow(cloud_correction->points[m].x,2)+pow(cloud_correction->points[m].y,2);
//        r=pow(r,0.5);
//        th=acos(cloud_correction->points[m].x/r);
//        if(cloud_correction->points[m].y<0)
//        {th=2*M_PI-th;}
//
//        for( i=0;i<R;i++)
//        {
//
//            if (((r>i*grid_size_r)&&(r<(i+1)*grid_size_r)))//如果r在某范围内
//            {
//                for( j=0;j<TH;j++)
//                {
//
//                    if((th>j*grid_size_th)&&(th<(j+1)*grid_size_th))//如果th在某范围内
//                    {
//
//                        grid[i][j].grid_cloud->points.push_back(cloud_correction->points[m]);
//
//                    }
//                }
//            }
//        }
//    }

////栅格特征
//    int a,b=0;
//    for(a=0;a<TH;a++)
//    {
//        for(b=0;b<R;b++)
//        {
//
//
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].h_mean+=(grid[a][b].grid_cloud->points[i].z)/(grid[a][b].grid_cloud->size());
//
//            }
//
//
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].h_square+=pow(grid[a][b].grid_cloud->points[i].z-grid[a][b].h_mean,2);
//
//            }
//
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].intensity_mean+=(grid[a][b].grid_cloud->points[i].intensity)/grid[a][b].grid_cloud->size();
//
//            }
//
//            if((grid[a][b].h_square<threshold)&&(grid[a][b].h_mean<0.2))//&&(grid[a][b].intensity_mean>0)
//            {
//
//                *cloud_final+=*grid[a][b].grid_cloud;
//            }
//
//
//        }
//    }

//删除离群点(小片离群的)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_final);
    sor.setMeanK (5);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud_final);
//删除离群点(大片离群的)
    sor.setInputCloud (cloud_final);
    sor.setMeanK (100);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud_final);

//求栅格的法向量方差(待定)



//可视化点云
    pcl::visualization::CloudViewer viewer("resterization");//可视化窗口
    viewer.showCloud(cloud_final);//看一下其中某个栅格
    while(!viewer.wasStopped())
    {    }



    return 0;
}


