
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Vector3f;

class Grid
{
public:
    bool  road;//是否是道路
    bool  bulge;
    bool  hollow;
    bool  empty;
    float h_mean;//平均高度
    float h_square;//高度方差
    float intensity_mean;//平均反射强度
    float normal_mean_x;//法向量x值的平均
    float normal_mean_y;
    float normal_mean_z;
    float normal_square;//法向量方差

    pcl::PointCloud<pcl::PointXYZI> grid_cloud;
    vector<pcl::Normal > Normals;//用于存放normal
    float h_delta;//栅格内最大高度差
    float h_gradient;//高度梯度
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
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.3);
    plane_seg.setInputCloud(cloud_in);
    plane_seg.segment(*plane_inliers, *plane_coefficients);

//地面校准
    Vector3f planar_normal;
    Vector3f z_axis;
    z_axis << 0, 0, 1;
    planar_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
    Eigen::Matrix4f rotation = CreateRotateMatrix(planar_normal, z_axis);
    cout << rotation << "\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_correction(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_in, *cloud_correction, rotation);
//计算向量信息
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_correction);
    n.setInputCloud(cloud_correction);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //将点云数据与法向信息拼接
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud_correction, *normals, *cloud_with_normals);

//棋盘格型栅格化
    int column = 25;
    int row = 15;
    float grid_size_x = 0.1;
    float grid_size_y = 0.1;
    int j = 0;
    int i = 0;
    double threshold;
    int intensity;
    double normal_square;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    ros::param::get("column", column);
    ros::param::param<int>("column", column, 500);
    ros::param::get("row", row);
    ros::param::param<int>("row", row, 300);
    ros::param::get("intensity", intensity);
    ros::param::param<int>("intensity", intensity, 20);
    ros::param::get("row", row);
    ros::param::get("normal_square", normal_square);
    ros::param::param<double >("normal_square", normal_square, 0.050);
    grid_size_x = 50.0 / column;
    grid_size_y = 30.0 / row;
    ros::param::get("threshold", threshold);
    ros::param::param<double>("threshold", threshold, 0.00010);
//    Grid grid[column][row];
    vector<vector<Grid> > grid;
    grid.resize(column);
    for (int i=0;i<column;i++)//错把i++写到中间了
    {
        grid[i].resize(row);
    }

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

                        grid[i+column/2][j+row/2].grid_cloud.points.push_back(cloud_correction->points[m]);
                        grid[i+column/2][j+row/2].Normals.push_back(normals->points[m]);

                    }
                }
            }
        }

    }

//栅格特征求取
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
    int a,b=0;
//    for(a=0;a<column;a++)
//    {
//        for(b=0;b<row;b++)
//        {
//
////高度平均值
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].h_mean+=(grid[a][b].grid_cloud->points[i].z)/(grid[a][b].grid_cloud->size());
//            }
////            cout<<"h_mean"<<grid[a][b].h_mean<<"\n";
////高度方差
//
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].h_square+=pow(grid[a][b].grid_cloud->points[i].z-grid[a][b].h_mean,2);
//
//            }
////反射率平均值
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].intensity_mean+=(grid[a][b].grid_cloud->points[i].intensity)/grid[a][b].grid_cloud->size();
//
//            }
////栅格法向量平均值
//            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].normal_mean_x+=(grid[a][b].grid_cloud->points[i].normal_x);
//                grid[a][b].normal_mean_y+=(grid[a][b].grid_cloud->points[i].normal_y);
//                grid[a][b].normal_mean_z+=(grid[a][b].grid_cloud->points[i].normal_z);
//                grid[a][b].normal_mean_x=grid[a][b].normal_mean_x/grid[a][b].grid_cloud->size();
//                grid[a][b].normal_mean_y=grid[a][b].normal_mean_y/grid[a][b].grid_cloud->size();
//                grid[a][b].normal_mean_z=grid[a][b].normal_mean_z/grid[a][b].grid_cloud->size();
//            }
////栅格法向量平均值
//            for(int i=0;i<grid[a][b].grid_cloud->size();i++)
//            {
//                grid[a][b].normal_square+=pow((grid[a][b].grid_cloud->points[i].normal_x-grid[a][b].normal_mean_x),2);
//                grid[a][b].normal_square+=pow((grid[a][b].grid_cloud->points[i].normal_y-grid[a][b].normal_mean_y),2);
//                grid[a][b].normal_square+=pow((grid[a][b].grid_cloud->points[i].normal_z-grid[a][b].normal_mean_z),2);
//                grid[a][b].normal_square=grid[a][b].normal_square/grid[a][b].grid_cloud->size();
//            }
////筛选条件
//            if((grid[a][b].h_mean<-0.7)&&(grid[a][b].intensity_mean<intensity))//(grid[a][b].h_square<threshold)&&
//            {
//
//                *cloud_final+=*grid[a][b].grid_cloud;
//            }
//
//
//        }
//    }
    for(a=0;a<column;a++)
    {
        for(b=0;b<row;b++)
        {
            if(grid[a][b].grid_cloud.size()>0)
            {
                cout<<"size "<<grid[a][b].grid_cloud.size()<<"\n";
//高度平均值
                for (int i=0;i<grid[a][b].grid_cloud.size();i++)
                {
                    grid[a][b].h_mean+=(grid[a][b].grid_cloud.points[i].z)/(grid[a][b].grid_cloud.size());

                }
//高度方差
                for (int i=0;i<grid[a][b].grid_cloud.size();i++)
                {
                    grid[a][b].h_square+=pow(grid[a][b].grid_cloud.points[i].z-grid[a][b].h_mean,2);

                }
//反射率平均值
                for (int i=0;i<grid[a][b].grid_cloud.size();i++)
                {
                    grid[a][b].intensity_mean+=(grid[a][b].grid_cloud.points[i].intensity)/grid[a][b].grid_cloud.size();

                }
//栅格法向量平均值
                for (int i=0;i<grid[a][b].grid_cloud.size();i++)
                {
                    grid[a][b].normal_mean_x+=(grid[a][b].Normals[i].normal_x);
                    grid[a][b].normal_mean_y+=(grid[a][b].Normals[i].normal_y);
                    grid[a][b].normal_mean_z+=(grid[a][b].Normals[i].normal_z);
                    grid[a][b].normal_mean_x=grid[a][b].normal_mean_x/grid[a][b].grid_cloud.size();
                    grid[a][b].normal_mean_y=grid[a][b].normal_mean_y/grid[a][b].grid_cloud.size();
                    grid[a][b].normal_mean_z=grid[a][b].normal_mean_z/grid[a][b].grid_cloud.size();
                }
//栅格法向量平均值
                for(int i=0;i<grid[a][b].grid_cloud.size();i++)
                {
                    grid[a][b].normal_square+=pow((grid[a][b].Normals[i].normal_x-grid[a][b].normal_mean_x),2);
                    grid[a][b].normal_square+=pow((grid[a][b].Normals[i].normal_y-grid[a][b].normal_mean_y),2);
                    grid[a][b].normal_square+=pow((grid[a][b].Normals[i].normal_z-grid[a][b].normal_mean_z),2);
                    grid[a][b].normal_square= grid[a][b].normal_square/grid[a][b].grid_cloud.size();
                }
//最大高度差
                pcl::getMinMax3D(grid[a][b].grid_cloud,min,max);
                grid[a][b].h_delta=max[2]-min[2];
//最大梯度
                if((a+1<=column)&&(b+1<=row)&&(a>=1)&&(b>=1))
                {
                    for(int i=-1;i<1;i++)
                    {
                        for(int j=-1;j<1;j++)
                        {
                            if(grid[a][b].h_mean-grid[a+i][b+j].h_mean>grid[a][b].h_gradient)
                            {
                                grid[a][b].h_gradient=grid[a][b].h_mean-grid[a+i][b+j].h_mean;
                            }
                        }

                    }
                } else{grid[a][b].h_gradient=0;}

//筛选条件
                if((grid[a][b].h_gradient<0.02))//(grid[a][b].h_mean<-0.3)&&(grid[a][b].intensity_mean<intensity)&&(grid[a][b].normal_square<normal_square)&&(grid[a][b].intensity_mean<intensity)&&(grid[a][b].h_square<threshold)
                {
                    *cloud_final+=grid[a][b].grid_cloud;
                }
            }
        }
    }



    for(int i=0;i<column;i++)
    {
        for(int j=0;j<row;j++)
        {
            cout<<grid[i][j].normal_square<<"\n";
        }
    }


////删除离群点(小片离群的)
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//    sor.setInputCloud (cloud_final);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (0.1);
//    sor.filter (*cloud_final);
////删除离群点(大片离群的)
//    sor.setInputCloud (cloud_final);
//    sor.setMeanK (500);
//    sor.setStddevMulThresh (0.3);
//    sor.filter (*cloud_final);

//可视化 普通
    pcl::visualization::CloudViewer viewer("resterization");//可视化窗口
    viewer.showCloud(cloud_final);//看一下其中某个栅格
    while(!viewer.wasStopped())
    {    }
//可视化 向量点云

    return 0;
}




