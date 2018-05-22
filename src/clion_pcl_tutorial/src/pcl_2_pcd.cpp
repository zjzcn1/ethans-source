#include <ros/ros.h>
#include <pcl/point_cloud.h>
//msgs type and conversion
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcd io
#include <pcl/io/pcd_io.h>
//point types
#include <pcl/point_types.h>
#include <sstream>
int i=0;
int name_num=0;
std::string num2str(int i)
{
    std::stringstream ss;
    ss<<i;
    return ss.str();
}
void 
call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
//Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);//cloud is the output
    
//save to PCD for 30 times    
    std::string pcd_name("cloud");
    if(i<60)
    {
        i++;
        std::string num;
        num=num2str(i);
        pcd_name+=num;
        if(pcl::io::savePCDFileASCII (pcd_name+".pcd", cloud)>=0)//input pointcloud should be pcl::PointCloud<PointT> only,rather than others
        {std::cerr << "Saved  " << pcd_name<<".pcd"<< std::endl;}

    }
}

int
main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "pcl_2_pcd");
  ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("plane_seg2", 1, call_back);

// Spin
  ros::spin ();
}








