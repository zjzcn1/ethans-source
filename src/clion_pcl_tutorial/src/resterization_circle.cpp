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
#define _USE_MATH_DEFINES
class Grid
{
public:
    bool  road;
    float h_mean;
    float h_square;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud {new pcl::PointCloud<pcl::PointXYZ>};
};
inline float abs(float &a)
{
    if (a<0)
    {a=-a;}
    return a;
}
int
main(int argc,char** argv)
{

    ros::init(argc,argv,"resterization");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/resterization_pointcloud", 1);
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in  (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/ethan/bags/pcl_2_bag/cloud9.pcd", *cloud_in) == -1)
    {
        PCL_ERROR ("Couldn't read file cloud9.pcd ^.^\n");
        return (-1);
    }

    int R=150;
    int TH=72;
    float grid_size_r=0.1;
    float grid_size_th=2*M_PI/TH;

    int j=0;
    int i=0;
    double threshold;

    ros::param::get("R",R);
    ros::param::param<int>("R", R, 150);
    ros::param::get("TH",TH);
    ros::param::param<int>("TH", TH, 72);

    grid_size_r=50.0/R;
    grid_size_th=2*M_PI/TH;
    ros::param::get("threshold",threshold);
    ros::param::param<double>("threshold", threshold, 0.00010);

    Grid grid[R][TH];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
    float r=0;
    float th;
    for (int m=0;m<cloud_in->size();m++)//这些点一个一个看
    {
        r=pow(cloud_in->points[m].x,2)+pow(cloud_in->points[m].y,2);
        r=pow(r,0.5);
        th=acos(cloud_in->points[m].x/r);
        if(cloud_in->points[m].y<0)
        {th=2*M_PI-th;}

        for( i=0;i<R;i++)
        {

            if (((r>i*grid_size_r)&&(r<(i+1)*grid_size_r)))//如果r在某范围内
            {
                for( j=0;j<TH;j++)
                {

                    if((th>j*grid_size_th)&&(th<(j+1)*grid_size_th))//如果th在某范围内
                    {

                        grid[i][j].grid_cloud->points.push_back(cloud_in->points[m]);

                    }
                }
            }
        }
    }


    int a,b=0;
    for(a=0;a<TH;a++)
    {
        for(b=0;b<R;b++)
        {


            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_mean+=(grid[a][b].grid_cloud->points[i].z)/(grid[a][b].grid_cloud->size());

            }


            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_square+=pow(grid[a][b].grid_cloud->points[i].z-grid[a][b].h_mean,2);

            }

            if((grid[a][b].h_square<threshold)&&(grid[a][b].h_mean<0.2))
            {

                *cloud_final+=*grid[a][b].grid_cloud;
            }


        }
    }


    pcl::toROSMsg(*cloud_final,cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = " ";
    pub.publish(cloud_out);


    pcl::visualization::CloudViewer viewer("resterization");//可视化窗口
    viewer.showCloud(cloud_final);//看一下其中某个栅格
    while(!viewer.wasStopped())
    {    }
}
