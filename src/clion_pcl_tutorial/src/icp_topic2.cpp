//
// Created by ethan on 18-5-28.
//
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


void pairAlign2 (const PointCloud::Ptr cloud_planar, const PointCloud::Ptr cloud_last, PointCloud::Ptr cloud_temp, Eigen::Matrix4f &pairTransform, bool downsample = false)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    int MaximumIterations;
    double TransformationEpsilon;
    float EuclideanFitnessEpsilon;
    float MaxCorrespondenceDistance;
    icp.setInputSource(cloud_planar);
    icp.setInputTarget(cloud_last);
    ros::param::get("MaximumIterations",MaximumIterations);
    ros::param::param<int>("MaximumIterations", MaximumIterations, 500);
    ros::param::get("TransformationEpsilon",TransformationEpsilon);
    ros::param::param<double >("TransformationEpsilon", TransformationEpsilon, 1e-10);
    ros::param::get("EuclideanFitnessEpsilon",EuclideanFitnessEpsilon);
    ros::param::param<float>("EuclideanFitnessEpsilon", EuclideanFitnessEpsilon, 0.01);
    ros::param::get("MaxCorrespondenceDistance",MaxCorrespondenceDistance);
    ros::param::param<float>("MaxCorrespondenceDistance", MaxCorrespondenceDistance, 1.0);
    icp.setMaximumIterations(MaximumIterations);
    icp.setTransformationEpsilon(TransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
    icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
    icp.align(*cloud_temp);
    pairTransform= icp.getFinalTransformation();
}



class icp_topic
{
public:
    icp_topic()
    {pub = nh.advertise<sensor_msgs::PointCloud2>("/icped_pointcloud", 1);
        sub = nh.subscribe("/velodyne_points",3,&icp_topic::call_back,this);
        min_point<<-1.8,-0.8,0.0,1.0;//cube的两个对角点
        max_point<<-3.5,0.8,2.0,1.0;
        i=0;
    }

    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {

        if (i<5)
        {
            i++;
            cout << "jumping "<<i<<" frame" << endl;
        }
        else
        {
            cout << "I RECEIVED INPUT !" << endl;
            pcl::fromROSMsg(*input, *cloud_in);
//clip_box
            ros::param::param<float>("x1", x1, -3.3);
            ros::param::param<float>("y1", y1, -1.0);
            ros::param::param<float>("z1", z1, -5.0);
            ros::param::param<float>("x2", x2, -1.8);
            ros::param::param<float>("y2", y2, 1.5);
            ros::param::param<float>("z2", z2, 5);
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
            clipper.filter(*cloud_clipper);
            clipper.setNegative(negative);//默认为false

//planar

            ros::param::get("dis1",dis1);
            ros::param::get("dis2",dis2);
            ros::param::param<float>("dis1", dis1, -1.5);
            ros::param::param<float>("dis2", dis2, -0.5);
            ros::param::get("negative2",negative2);
            pass.setInputCloud (cloud_clipper);//has to be a pointer
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (dis1, dis2);
            pass.setFilterLimitsNegative (negative2);
            pass.filter (*cloud_planar);
//ICP
            if (cloud_last->size()==0)
            {*cloud_last=*cloud_planar;}
            else
             {
                pairAlign2(cloud_planar,cloud_last,cloud_temp,pairTransform,false);
                GlobalTransform = GlobalTransform*pairTransform;
                pcl::transformPointCloud(*cloud_in, *cloud_final_raw, GlobalTransform);
                *cloud_final += *cloud_final_raw;
                *cloud_last = *cloud_planar;//!!!
             }


            pcl::toROSMsg(*cloud_final, cloud_out);
            cloud_out.header.stamp = ros::Time::now();
            cloud_out.header.frame_id = "/lidar";
            pub.publish(cloud_out);
            cout << "I PROCESSED IT !" << endl;
            i=0;
        }
    }
private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp {new pcl::PointCloud<pcl::PointXYZ>};//每两个相加的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final_raw {new pcl::PointCloud<pcl::PointXYZ>};//某帧经过全局转换后的存储地
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};//icp后的pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last {new pcl::PointCloud<pcl::PointXYZ>};//上一次订阅到的帧
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planar {new pcl::PointCloud<pcl::PointXYZ>};//上一次订阅到的帧
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clipper {new pcl::PointCloud<pcl::PointXYZ>};//上一次订阅到的帧
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    Eigen::Matrix4f pairTransform,GlobalTransform = Eigen::Matrix4f::Identity ();
    int i;

    //clip
    pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    float x1,y1,z1;
    float x2,y2,z2;
    bool negative;
    bool negative2;
    //planar
    pcl::PassThrough<pcl::PointXYZ> pass;
    float dis1;//距离阈值
    float dis2;//距离阈值
    //icp

};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "icp_topic2");

    icp_topic icp_topic1;

    ros::spin ();
}


