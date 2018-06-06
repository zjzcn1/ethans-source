//
// Created by ethan on 18-5-15.
//
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>


using namespace std;
using Eigen::MatrixXd;
MatrixXd p_last(2,2);

class kalman_filter
{
public:
    kalman_filter(float q1_,float q2_,float q3_,float q4_,float r1)
    {
        s=0.0;
        x_raw=MatrixXd::Constant(2,1,0);
        x=MatrixXd::Constant(2,1,0);
        t=0.0;
        last=ros::Time::now().toSec();
        a<<0.5;

        p_raw= MatrixXd::Constant(2,2,0);
        p_last=MatrixXd::Constant(2,2,0);
        p = MatrixXd::Constant(2,2,0);
        K = MatrixXd::Constant(2,1,0);
        Z = MatrixXd::Constant(1,1,0);
        Q<<q1_,q2_,q3_,q4_;
        R<<r1;

        H<<1.0,0.0;
        pub = nh.advertise<std_msgs::Float32>("/s_raw", 1);
        pub2 = nh2.advertise<std_msgs::Float32>("/s_real", 1);
        pub3= nh3.advertise<std_msgs::Float32>("/s_filtered", 1);
        sub = nh.subscribe("/gauss_noise",3,&kalman_filter::call_back,this);
    };

    void call_back(const std_msgs::Float32::ConstPtr measure)
    {
        ros::param::get("Q_noise1",q1);
        ros::param::get("Q_noise2",q2);
        ros::param::get("Q_noise3",q3);
        ros::param::get("Q_noise4",q4);
        ros::param::get("R_noise" ,r);
        Q<<q1,q2,q3,q4;
        R<<r;
        delta_t=ros::Time::now().toSec()-last;
        A<<1.0,delta_t,0.0,1.0;
        B<<0.5*delta_t*delta_t,delta_t;
        t+=delta_t;
        s=0.5*a(0,0)*t*t;//真实位移
        Z(0,0)=s+measure->data;//测量位移，真实加噪声

        //预测
        x_raw = A*x+B*a;
        p_raw=A*p_last*A.transpose()+Q;
        //测量
        K=p_raw*H.transpose()*(H*p_raw*H.transpose()+R).inverse();
        x=x_raw+K*(Z-H*x_raw);
        p=(MatrixXd::Identity(2,2)-K*H)*p_raw;
        p_last=p;


        //pub
        raw.data= Z(0,0);
        Real.data=s;
        filtered.data=x(0,0);
        static int i;
        if (i<2000)
        {
            i++;
            pub.publish(raw);
            pub2.publish(Real);
            pub3.publish(filtered);
        }

        last=ros::Time::now().toSec();

    }


private:
    MatrixXd x_raw{2,1};//预测值n行1列
    MatrixXd x{2,1};//状态向量n行1列
    MatrixXd A{2,2};//n*n,状态转移矩阵
    MatrixXd B{2,1};//控制转移矩阵，n行1列
    MatrixXd a{1,1};//控制量
    MatrixXd p{2,2};//状态的协方差矩阵n*n
    MatrixXd p_raw{2,2};//协方差矩阵的预测n*n
    MatrixXd Q{2,2};//n*n,预测值噪声，是！叠！加！在！协方差矩阵！上！的！
    MatrixXd K{2,1};//n*m,卡尔曼增益，[n*m]*[m*1]=[n*1](x的维度)
    MatrixXd R{1,1};//m*m我们先试试观测值只有一个
    MatrixXd Z{1,1};//观测值
    MatrixXd H{1,2};//m*n,状态与观测关系
    float s;//位移
    float delta_t;//时间间隔
    float t;//时间
    float q1,q2,q3,q4;
    float r;


    ros::NodeHandle nh;
    ros::NodeHandle nh2;
    ros::NodeHandle nh3;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Subscriber sub;
    std_msgs::Float32 raw;
    std_msgs::Float32 Real;
    std_msgs::Float32 filtered;
    double last;
};

//
int main(int argc,char** argv) {
    cout<<"Lets start kalmanfilter!\n"<<endl;
    ros::init(argc,argv,"kalmanfilter");
    kalman_filter kalman_filter1(0.01,0.0,0.0,0.01,1000);

    ros::spin();
}

