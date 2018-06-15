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

    int column=100;
    int row=100;
    float grid_size_x=0.1;
    float grid_size_y=0.1;

    int j=0;
    int i=0;
    double threshold;

//while(ros::ok())
//{

    ros::param::get("column",column);
    ros::param::param<int>("column", column, 500);
    ros::param::get("row",row);
    ros::param::param<int>("row", row, 200);

    grid_size_x=50.0/column;
    grid_size_y=30.0/row;
    ros::param::get("threshold",threshold);
    ros::param::param<double>("threshold", threshold, 0.00010);

    Grid grid[column][row];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
    cout<<"in "<<cloud_in->size()<<endl;
    for (int m=0;m<cloud_in->size();m++)//这些点一个一个看
    {
        for( i=-column/2;i<column/2;i++)
        {
            if (((cloud_in->points[m].x>i*grid_size_x)&&(cloud_in->points[m].x<i*grid_size_x+grid_size_x)))//如果x在某范围内
            {
                for( j=-row/2;j<row/2;j++)
                {

                    if((cloud_in->points[m].y>j*grid_size_y)&&(cloud_in->points[m].y<j*grid_size_y+grid_size_y))//如果y在某范围内
                    {

                        grid[i+column/2][j+row/2].grid_cloud->points.push_back(cloud_in->points[m]);

                    }
                }
            }
        }
    }


    int a,b=0;
    for(a=0;a<column;a++)
    {
        for(b=0;b<row;b++)
        {


            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_mean+=(grid[a][b].grid_cloud->points[i].z)/(grid[a][b].grid_cloud->size());

            }


            for (int i=0;i<grid[a][b].grid_cloud->size();i++)
            {
                grid[a][b].h_square+=pow(grid[a][b].grid_cloud->points[i].z-grid[a][b].h_mean,2);

            }


            if((grid[a][b].h_square<threshold)&&(grid[a][b].h_mean<-0.9))
            {

                *cloud_final+=*grid[a][b].grid_cloud;
            }


        }
    }
    //删除离群点
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud_final);
//    sor.setMeanK (100);
//    sor.setStddevMulThresh (0.03);
//    sor.filter (*cloud_final);


    pcl::toROSMsg(*cloud_final,cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = " ";
    pub.publish(cloud_out);

//}
    pcl::visualization::CloudViewer viewer("resterization");//可视化窗口
    viewer.showCloud(cloud_final);//看一下其中某个栅格
    while(!viewer.wasStopped())
    {    }
    return 0;
}
