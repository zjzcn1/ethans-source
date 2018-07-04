//
// Created by ethan on 18-6-21.
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
#include <nav_msgs/GridCells.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

using Eigen::MatrixXd;
using Eigen::Vector3f;
using namespace std;


class Grid//栅格单元
{
public:
    Grid(bool r=1,bool area=0,float h_m=0.0,float h_s=0.0,float h_delta=0.0,float i_mean=0.0,int i_=0,int j_=0);
    bool  road;
    bool  area;
    float h_mean;
    float h_square;
    float h_delta;
    float intensity_mean;
    pcl::PointCloud<pcl::PointXYZI> grid_cloud ;
    int i;
    int j;
};
Grid::Grid(bool r,bool area,float h_m, float h_s, float h_delta_,float i_mean, int i_, int j_)
{
road= r;area=area;h_mean=h_m;h_s=h_square;h_delta=h_delta_;intensity_mean=i_mean;i=i_;j=j_;
}

typedef struct Index_of_grid//栅格的下标
{
    int i=0;
    int j=0;
}Index;

class Grid_cloud//栅格化后的点云
{
public:
    Grid_cloud(int column_num_=0,int row_num_=0,float column_size_=0,float row_size_=0);
    int column_num;
    int row_num;
    float column_size;
    float row_size;
    float column_len;
    float row_len;
    vector<vector<Grid> > grid;//动态二维数组,栅格化后的点云
    int area_num;
};
Grid_cloud::Grid_cloud(int column_num_, int row_num_, float column_size_, float row_size_)
{
    column_num=column_num_;row_num=row_num_;column_size=column_size_;row_size=row_size_;
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

////////////////////////////////
//地面校准
void plane_correction(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{

    if(input->size()==0)
    {
        cout<<"cloud is empty\n";
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
        plane_seg.setInputCloud ( input );
        plane_seg.segment (*plane_inliers, *plane_coefficients);

        //地面校准
        Vector3f  planar_normal;
        Vector3f  z_axis;
        z_axis<<0,0,1;
        planar_normal<<plane_coefficients->values[0],plane_coefficients->values[1],plane_coefficients->values[2];
        Eigen::Matrix4f rotation=CreateRotateMatrix(planar_normal,z_axis);

        pcl::transformPointCloud(*input, *input, rotation);

    }

}
//选取区域
void pass_through(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{

    float x_back=0.0;
    float x_front=0.0;
    float y_left=0.0;
    float y_right=0.0;
    //直通滤波
    pcl::PassThrough<pcl::PointXYZI> pass;

    pass.setInputCloud (input);//has to be a pointer
    pass.setFilterFieldName ("x");
    ros::param::get("x_back",x_back);
    ros::param::param<float>("x_back", x_back, -5.0);
    ros::param::get("x_front",x_front);
    ros::param::param<float>("x_front", x_front, 15.0);
    pass.setFilterLimits (x_back, x_front);
    pass.setFilterLimitsNegative (false);
    pass.filter(*input);

    pass.setInputCloud (input);
    pass.setFilterFieldName ("y");
    ros::param::get("y_left",y_left);
    ros::param::param<float>("y_left", y_left, -7.0);
    ros::param::get("y_right",y_right);
    ros::param::param<float>("y_right", y_right, 7.0);
    pass.setFilterLimits (y_left, y_right);
    pass.setFilterLimitsNegative (false);
    pass.filter(*input);

}
//栅格化点云
Grid_cloud grid_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr icloud)//不能返回引用,因为返回的东西是一个局部对象
{
    int column=100;
    int row=100;
    float grid_size_x=0.1;
    float grid_size_y=0.1;
    int j=0;
    int i=0;
    float x_back=0;
    float x_front=0;
    float y_left=0;
    float y_right=0;
    ros::param::get("x_back",x_back);
    ros::param::param<float>("x_back", x_back, -5.0);
    ros::param::get("x_front",x_front);
    ros::param::param<float>("x_front", x_front, 15.0);
    ros::param::get("y_left",y_left);
    ros::param::param<float>("y_left", y_left, -7.0);
    ros::param::get("y_right",y_right);
    ros::param::param<float>("y_right", y_right, 7.0);
    ros::param::get("column",column);
    ros::param::param<int>("column", column, 200);
    ros::param::get("row",row);
    ros::param::param<int>("row", row, 140);

    grid_size_x=(x_front-x_back)/column;//注意 点零
    grid_size_y=(y_right-y_left)/row;

    Grid_cloud grid_cloud;
    grid_cloud.column_len=x_front-x_back;
    grid_cloud.row_len=y_right-y_left;
    grid_cloud.column_size=grid_size_x;
    grid_cloud.row_size=grid_size_y;
    grid_cloud.column_num=column;
    grid_cloud.row_num=row;

//创建栅格
    grid_cloud.grid.resize(column);//先创建column，再逐column创建row
    for(int i=0;i<column;i++)
    {grid_cloud.grid[i].resize(row);}

//往栅格里填充点
    for (int m=0;m<icloud->size();m++)//遍历这些点
    {
        for( i=0;i<column;i++)
        {
            if (((icloud->points[m].x>(i*grid_size_x)+x_back)&&(icloud->points[m].x<(i+1)*grid_size_x+x_back)))//如果x在某范围内
            {
                for( j=0;j<row;j++)
                {
                    if((icloud->points[m].y>j*grid_size_y+y_left)&&(icloud->points[m].y<(j+1)*grid_size_y+y_left))//如果y在某范围内
                    {
                      grid_cloud.grid[i][j].grid_cloud.points.push_back(icloud->points[m]);
                    }
                }
            }
        }
    }
    return grid_cloud;
}
//判断栅格属性
void grids_attributes(Grid_cloud& grided_cloud)
{
    int a,b=0;
    double threshold;
    int intensity;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
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
            grided_cloud.grid[a][b].i=a;
            grided_cloud.grid[a][b].j=b;

            if(grided_cloud.grid[a][b].grid_cloud.size()==0)//如果是空栅格,那么直接认为是道路
            {
                grided_cloud.grid[a][b].road=0;
            }
            else
            {
//高度平均值
                for (int i = 0; i < grided_cloud.grid[a][b].grid_cloud.size(); i++)
                {
                    grided_cloud.grid[a][b].h_mean += (grided_cloud.grid[a][b].grid_cloud.points[i].z) /
                                                      (grided_cloud.grid[a][b].grid_cloud.size());

                }
//高度方差
                for (int i = 0; i < grided_cloud.grid[a][b].grid_cloud.size(); i++)
                {
                    grided_cloud.grid[a][b].h_square += pow(
                            grided_cloud.grid[a][b].grid_cloud.points[i].z - grided_cloud.grid[a][b].h_mean, 2);
                }
//最大高度差
                pcl::getMinMax3D(grided_cloud.grid[a][b].grid_cloud,min,max);
                grided_cloud.grid[a][b].h_delta=max[2]-min[2];

//反射率平均值
                for (int i = 0; i < grided_cloud.grid[a][b].grid_cloud.size(); i++)
                {
                    grided_cloud.grid[a][b].intensity_mean += (grided_cloud.grid[a][b].grid_cloud.points[i].intensity) /
                                                              grided_cloud.grid[a][b].grid_cloud.size();
                }
//贴标道路判别标签
                if ((grided_cloud.grid[a][b].intensity_mean < intensity) && (grided_cloud.grid[a][b].h_delta < 0.03) &&
                    (grided_cloud.grid[a][b].h_square < threshold))//(a%2==0)&&(b%2==0)grided_cloud.grid[a][b].h_mean
                {
                    grided_cloud.grid[a][b].road = 1;
                }
                else
                {
                    grided_cloud.grid[a][b].road = 0;
                }

            }
            grided_cloud.grid[a][b].area = false;
        }
    }


}
//查看单个edge元素,查找它可走的邻居栅格,并将这种栅格标记为edge新成员和连通区域成员
int Find_neighbours(Grid grid_ij,Grid_cloud& grided_cloud,vector<Index>& edge)
{
    int8_t new_edge_single=0;//邻居的个数

      for(int i=grid_ij.i-1;i<=grid_ij.i+1;i++)//看本栅格周围的八个
      {
        for(int j=grid_ij.j-1;j<=grid_ij.j+1;j++)
        {
            if((i+j)==(grid_ij.i+grid_ij.j+2)||(i+j)==(grid_ij.i+grid_ij.j-2)||(i+j)==(grid_ij.i+grid_ij.j))//不看自己和对角线处的
            {

            }
            else if((i>=0)&&(j>=0)&&(i<grided_cloud.column_num)&&(j<grided_cloud.row_num))//index应该在容器大小范围内
            {
                if((grided_cloud.grid[i][j].road==1)&&(grided_cloud.grid[i][j].area==false))//可以走,并且还没加入到可行驶区域里
                {
                    grided_cloud.grid[i][j].area=true;//标记为连通区域成员

                    Index index_new;//将该栅格的下标存起来,记为新成员
                    index_new.i=grided_cloud.grid[i][j].i;
                    index_new.j=grided_cloud.grid[i][j].j;
                    edge.push_back(index_new);

                    new_edge_single++;//新邻居个数

                }
            }
        }
      }
      edge.erase(edge.begin());//遍历过的都扔掉

    return new_edge_single;//返回查找结果
}

//区域增长
void Region_grow(Grid_cloud& grided_cloud)
{
    vector<Index> edge;
    Index growth_init;//种子栅格(从这里开始查找)
    growth_init.i=grided_cloud.column_num/2;
    growth_init.j=grided_cloud.row_num/2;
    edge.push_back(growth_init);

    int new_edge_single=0;
    int edge_size_last=1;//上次遍历新栅格之后,edge的大小,=1代表种子
    int num=0;//区域大小
    while(edge_size_last>0)//如果上次遍历新成员又找到了新成员,就继续看最新的成员
    {
        for(int i=0;i<edge_size_last;i++)//由于for循环里的函数改变了edge.size(),所以不能将edge.size()放在for条件里
        {
            new_edge_single=Find_neighbours(grided_cloud.grid[edge[0].i][edge[0].j],grided_cloud,edge);//遍历新成员时,因为看一个扔一个,所以一直都是看的[0][0]
            num+=new_edge_single;//区域大小
        }
       //一轮查找结束后,更新下次遍历的个数
        edge_size_last=edge.size();
    }
    grided_cloud.area_num=num;
    cout<<"area_num1 = "<<num<<"\n";
}

//集合联通区域
//pcl::PointCloud<pcl::PointXYZI> Drivable_area( Grid_cloud& grided_cloud)
nav_msgs::GridCells Drivable_area( Grid_cloud& grided_cloud)
{
    nav_msgs::GridCells cells;//连通区域对应的栅格
    cells.header.frame_id="horizontal_vlp16_link";
    cells.cell_height=grided_cloud.column_size*3;
    cells.cell_width=grided_cloud.row_size*3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_area(new pcl::PointCloud<pcl::PointXYZI>);

    int a=0,b=0;
    for(a=0;a<grided_cloud.column_num;a++)
    {
        for(b=0;b<grided_cloud.row_num;b++)
        {
            if(grided_cloud.grid[a][b].road==true)
            {
//              *cloud_area+= grided_cloud.grid[a][b].grid_cloud;
                geometry_msgs::Point cell;
                cell.x=((a+0.5)-grided_cloud.column_num/2)*grided_cloud.column_size;
                cell.y=((b+0.5)-grided_cloud.row_num/2)*grided_cloud.row_size;
                cell.z=0;
                cells.cells.push_back(cell);
            }
            else
            {

            }
        }
    }


    return cells;
//    return *cloud_area;
}
/////////////////////////////////////////////////////////////////////////////////////
class Resterization
{
public:
    Resterization()
    {
//        pub = nh.advertise<sensor_msgs::PointCloud2>("/drivable_area1", 1);
        pub = nh.advertise<nav_msgs::GridCells>("/drivable_area1", 1);
        pub2= nh.advertise<sensor_msgs::PointCloud2>("/origin", 1);
        sub = nh.subscribe("/icped_pointcloud",3,&Resterization::call_back,this);
    }
    void call_back(const sensor_msgs::PointCloud2::Ptr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);
        //水平面校准
        plane_correction(cloud_in);
        //剪切区域
        pass_through(cloud_in);
        //栅格化
        grid_cloud1=grid_cloud(cloud_in);
        //判定栅格属性
        grids_attributes(grid_cloud1);

        //静态障碍物检测

        //动态障碍物检测

        //障碍物聚类

        //连通可行驶域//去除离群栅格
        Region_grow(grid_cloud1);
        cloud_final=Drivable_area(grid_cloud1);

//        pcl::toROSMsg(*cloud_in,cloud_out);
        input->header.stamp = ros::Time::now();
        input->header.frame_id = "horizontal_vlp16_link";


        pub.publish(cloud_final);
        pub2.publish(*input);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Publisher  pub2;
    ros::Subscriber sub;
    Grid_cloud grid_cloud1;//栅格化后的点云

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in  {new pcl::PointCloud<pcl::PointXYZI>};
    pcl::PointCloud<pcl::PointXYZI> cloud_final_raw;
//    pcl::PointCloud<pcl::PointXYZI> cloud_final;
    nav_msgs::GridCells cloud_final;


    pcl::PointCloud<pcl::PointXYZI> cloud_rest;
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    sensor_msgs::PointCloud2 cloud_out_rest;

    //test:

};


int
main(int argc,char** argv)
{
    ros::init(argc,argv,"road_detection");
    Resterization resterization;
    ros::spin();
}





