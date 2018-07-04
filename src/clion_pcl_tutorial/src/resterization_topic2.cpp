//
// Created by ethan on 18-6-21.
//
//
// Created by ethan on 18-6-6.
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
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>

using Eigen::MatrixXd;
using Eigen::Vector3f;
using namespace std;
class Grid//栅格单元
{
public:
    bool  road;
    float h_mean;
    float h_square;
    float intensity_mean;
    pcl::PointCloud<pcl::PointXYZI> grid_cloud ;
};

class Grid_cloud//栅格化后的点云
{
public:
    int column_num;
    int row_num;
    float column_size;
    float row_size;
    float column_len;
    float row_len;
    //Grid grid[6][6];
    vector<vector<Grid> > grid;//动态二维数组
};
inline float abs(float &a)//求绝对值
{
    if (a<0)
    {a=-a;}
    return a;
}
//旋转矩阵，用于校准地面水平度
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
//地面校准
pcl::PointCloud<pcl::PointXYZI> plane_correction(pcl::PointCloud<pcl::PointXYZI> input)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_correction;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    *input_ptr=input;
    if(input_ptr->size()==0)
    {
        cout<<"cloud is empty\n";
        cloud_correction=*input_ptr;
    }
    else
    {
        pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
        pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
        pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
        plane_seg.setOptimizeCoefficients (true);
        plane_seg.setModelType ( pcl::SACMODEL_PLANE );
        plane_seg.setMethodType ( pcl::SAC_RANSAC );
        plane_seg.setDistanceThreshold ( 0.3 );
        plane_seg.setInputCloud ( input_ptr );
        plane_seg.segment (*plane_inliers, *plane_coefficients);

        //地面校准
        Vector3f  planar_normal;
        Vector3f  z_axis;
        z_axis<<0,0,1;
        planar_normal<<plane_coefficients->values[0],plane_coefficients->values[1],plane_coefficients->values[2];
        Eigen::Matrix4f rotation=CreateRotateMatrix(planar_normal,z_axis);


//        rotation(3,2)=1;????????????????
        cout<<rotation<<"\n";
        pcl::transformPointCloud(input, cloud_correction, rotation);

    }
    return cloud_correction;
}

//栅格化点云
Grid_cloud grid_cloud(pcl::PointCloud<pcl::PointXYZI> icloud)//如何返回数组
{
    int column=100;
    int row=100;
    float grid_size_x=0.1;
    float grid_size_y=0.1;

    int j=0;
    int i=0;


    ros::param::get("column",column);
    ros::param::param<int>("column", column, 500);
    ros::param::get("row",row);
    ros::param::param<int>("row", row, 300);

    grid_size_x=50.0/column;
    grid_size_y=30.0/row;

    Grid_cloud grid_cloud;
    grid_cloud.column_len=50.0;
    grid_cloud.row_len=30.0;
    grid_cloud.column_size=grid_size_x;
    grid_cloud.row_size=grid_size_y;
    grid_cloud.column_num=column;
    grid_cloud.row_num=row;

//创建栅格
    grid_cloud.grid.resize(column);//先创建column，再逐column创建row
    for(int i=0;i<column;i++)
    {grid_cloud.grid[i].resize(row);}

//往栅格里填充点
    for (int m=0;m<icloud.size();m++)//遍历这些点
    {
        for( i=-column/2;i<column/2;i++)
        {
            if (((icloud.points[m].x>i*grid_size_x)&&(icloud.points[m].x<i*grid_size_x+grid_size_x)))//如果x在某范围内
            {
                for( j=-row/2;j<row/2;j++)
                {

                    if((icloud.points[m].y>j*grid_size_y)&&(icloud.points[m].y<j*grid_size_y+grid_size_y))//如果y在某范围内
                    {

                        grid_cloud.grid[i+column/2][j+row/2].grid_cloud.points.push_back(icloud.points[m]);

                    }
                }
            }
        }
    }


    return grid_cloud;

}
//按条件筛选栅格
vector<pcl::PointCloud<pcl::PointXYZI> > select_grids( Grid_cloud& grided_cloud)
{
    int a,b=0;
    double threshold;
    int intensity;
    ros::param::get("h_threshold",threshold);
    ros::param::param<double>("h_threshold", threshold, 0.00010);
    ros::param::get("intensity",intensity);
    ros::param::param<int>("intensity", intensity, 18);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_selected(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZI>);
    for(a=0;a<grided_cloud.column_num;a++)
    {
        for(b=0;b<grided_cloud.row_num;b++)
        {
//高度平均值
            for (int i=0;i<grided_cloud.grid[a][b].grid_cloud.size();i++)
            {
                grided_cloud.grid[a][b].h_mean+=(grided_cloud.grid[a][b].grid_cloud.points[i].z)/(grided_cloud.grid[a][b].grid_cloud.size());

            }

//高度方差
            for (int i=0;i<grided_cloud.grid[a][b].grid_cloud.size();i++)
            {
                grided_cloud.grid[a][b].h_square+=pow(grided_cloud.grid[a][b].grid_cloud.points[i].z-grided_cloud.grid[a][b].h_mean,2);

            }

//反射率平均值
            for (int i=0;i<grided_cloud.grid[a][b].grid_cloud.size();i++)
            {
                grided_cloud.grid[a][b].intensity_mean+=(grided_cloud.grid[a][b].grid_cloud.points[i].intensity)/grided_cloud.grid[a][b].grid_cloud.size();

            }
//筛选
            if((grided_cloud.grid[a][b].intensity_mean<intensity)&&(grided_cloud.grid[a][b].h_mean<-0.3)&&(grided_cloud.grid[a][b].h_square<threshold))//(a%2==0)&&(b%2==0)
            {

                *cloud_selected+=grided_cloud.grid[a][b].grid_cloud;

            }
            else
            {
                *cloud_rest+=grided_cloud.grid[a][b].grid_cloud;
            }
            cout<< grided_cloud.grid[a][b].intensity_mean<<"\n";
        }
    }
    vector<pcl::PointCloud<pcl::PointXYZI> > selected;
    selected.push_back(*cloud_selected);
    selected.push_back(*cloud_rest);
    return selected;

}
//去除离群点
pcl::PointCloud<pcl::PointXYZI> rm_lonely( pcl::PointCloud<pcl::PointXYZI> input)
{
    //删除离群点(小片离群的)
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    *input_ptr=input;

    if(input_ptr->size()==0)
    {
        cout<<"rm_lonely empty cloud\n";

    }
    else
    {
        sor.setInputCloud (input_ptr);
        sor.setMeanK (5);
        sor.setStddevMulThresh (0.5);
        sor.filter (input);

        //删除离群点(大片离群的)
        *input_ptr=input;
        sor.setInputCloud (input_ptr);
        sor.setMeanK (100);
        sor.setStddevMulThresh (0.5);
        sor.filter (input);
    }
    return input;
}

class Resterization
{
public:
    Resterization()
    {
        pub = nh.advertise<sensor_msgs::PointCloud2>("/drivable_area", 1);
        pub2 = nh.advertise<sensor_msgs::PointCloud2>("/rest_pointcloud", 1);
        sub = nh.subscribe("/icped_pointcloud",3,&Resterization::call_back,this);
    }
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);
        //水平面校准
        cloud_horizontal=plane_correction(*cloud_in);
        //栅格化
        grid_cloud1=grid_cloud(cloud_horizontal);
        //筛选栅格
        cloud_final_raw=select_grids(grid_cloud1)[0];
        cloud_rest=select_grids(grid_cloud1)[1];
        //去除离群点（后面应该去除离群栅格）
        cloud_final=rm_lonely(cloud_final_raw);
        //静态障碍物检测

        //动态障碍物检测

        //障碍物聚类

        //连通可行驶域


        pcl::toROSMsg(cloud_final,cloud_out);
        cloud_out.header.stamp = ros::Time::now();
        cloud_out.header.frame_id = " ";
        pcl::toROSMsg(cloud_rest,cloud_out_rest);
        cloud_out_rest.header.stamp = cloud_out.header.stamp;
        cloud_out_rest.header.frame_id = " ";

        pub.publish(cloud_out);
        pub2.publish(cloud_out_rest);

    }
private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Publisher  pub2;
    ros::Subscriber sub;
    Grid_cloud grid_cloud1;
    pcl::PointCloud<pcl::PointXYZI> cloud_horizontal ;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in  {new pcl::PointCloud<pcl::PointXYZI>};
    pcl::PointCloud<pcl::PointXYZI> cloud_final_raw ;
    pcl::PointCloud<pcl::PointXYZI> cloud_final;
    pcl::PointCloud<pcl::PointXYZI> cloud_rest;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final_ptr {new pcl::PointCloud<pcl::PointXYZI>} ;
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    sensor_msgs::PointCloud2 cloud_out_rest;
};


int
main(int argc,char** argv)
{
    ros::init(argc,argv,"road_detection");
    Resterization resterization;
    ros::spin();
}



























