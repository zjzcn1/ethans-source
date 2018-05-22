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


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;//using可以在派生类中使用基类的protected成员
public:
    MyPointRepresentation ()
    {
      nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray (const PointNormalT &p, float * out) const//const修饰类的成员函数，则该成员函数不能修改类中任何非const成员函数
    {
      // < x, y, z, curvature >
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;//这个函数是干嘛用的？
    }
};

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

    PointCloud::Ptr src (new PointCloud);//倒贴的
    PointCloud::Ptr tgt (new PointCloud);//被贴的
    pcl::VoxelGrid<PointT> grid;//下采样
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);//倒贴的变成带法线的
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;//法线估算对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());//kdtree对象，Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到最近邻点
    norm_est.setSearchMethod (tree);//使用kdtree的方法搜索估计法线
    norm_est.setKSearch (30);//设置最近邻的数量，或者用setRadiusSearch，对每个点设置一个半径，用来确定法线和曲率
    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);//计算得到带每个点的法向量
    pcl::copyPointCloud (*src, *points_with_normals_src);//将点云复制过去，最后得到的点云，每个点都包含坐标和法向量
    norm_est.setInputCloud (tgt);//对被倒贴的也这样操作,弄成带法线的
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


    MyPointRepresentation point_representation;

    float alpha[4] = {1.0, 1.0, 1.0, 1.0};//这个是干嘛的？加权曲率维度，以和坐标xyz保持平衡，没看懂
    point_representation.setRescaleValues (alpha);


    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//创建非线性ICP对象
    reg.setTransformationEpsilon (1e-6);//这个值一般设为1e-6或者更小。意义是什么？
    reg.setMaxCorrespondenceDistance (1.0);//设置对应点之间的最大距离（0.1m）,在配准过程中，忽略大于该阈值的点
    reg.setMaximumIterations (200);//迭代次数，几十上百都可能出现

    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));//括号内类比于：int a？？？ make_shared<T>:C创建一个shared_ptr共享指针
    reg.setInputSource (points_with_normals_src);//非线性icp主角：带法线的点云
    reg.setInputTarget (points_with_normals_tgt);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;//声明3个变量，其中Ti=Eigen::Matrix4f::Identity ()创建一个单位矩阵
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;//存放icp结果

    for (int i = 0; i < 30; ++i)//这个for循环是和距离阈值有关的，不断的减小距离阈值30次，相当于不断地离目标点云越来越近
    {
       // PCL_INFO ("Iteration Nr. %d.\n", i);

        points_with_normals_src = reg_result;//将上次的拼接结果作为本次拼接的倒贴者。因为在for循环里，不是每次都定义，所以直接用reg_result这个变量

        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);//拼接

        Ti = reg.getFinalTransformation()*Ti;//将本次最终的转换矩阵累积到之前的转换矩阵

        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())//如果上次和本次的转换矩阵差值的元素和小于我们设置的值（就是相比上一次动不了多少了）
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.01);//就将距离阈值设的小一些，把一些远的点去掉，再继续（for循环中）匹配
        prev = reg.getLastIncrementalTransformation ();//更新上一次的变换


    }

    targetToSource = Ti.inverse();//使倒贴者和被倒贴者互换的矩阵

    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);//被倒贴变倒贴了

    *output += *cloud_src;//贴过去以后和人加一块，变成一家人。拼接完成。
    final_transform = targetToSource;
}

class icp_topic
{
public:
    icp_topic()
    {pub = nh.advertise<sensor_msgs::PointCloud2>("/icped_pointcloud", 1);
     sub = nh.subscribe("/plane_seg2",3,&icp_topic::call_back,this); }
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        pcl::fromROSMsg(*input,*cloud_in);

        pairAlign(cloud_in,cloud_last,cloud_temp,pairTransform, false);
        GlobalTransform=pairTransform*GlobalTransform;
        pcl::transformPointCloud (*cloud_in, *cloud_final_raw, GlobalTransform);
        *cloud_final+=*cloud_final_raw;
        cloud_last=cloud_in;

        pcl::toROSMsg(*cloud_final,cloud_out);
        cloud_out.header.stamp = ros::Time::now();
        cloud_out.header.frame_id = "/lidar";
        pub.publish(cloud_out);
        cout<<cloud_out.width<<endl;
        cout<<"I PROCESSED IT !"<<endl;
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp {new pcl::PointCloud<pcl::PointXYZ>};//每两个相加的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final_raw {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};//加工后的pcl格式

    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last{new pcl::PointCloud<pcl::PointXYZ>};
    Eigen::Matrix4f pairTransform,GlobalTransform = Eigen::Matrix4f::Identity ();

};

int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "icp_topic");

  icp_topic icp_topic1;

  
  ros::spin ();
}

