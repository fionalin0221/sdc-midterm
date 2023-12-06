#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

class A{
public:
    int x_;
    int y_;
    A(){

    }
    A(int x, int y, int z){
        
    }
    A(int x, int y){ //建構子，用來初始化class的數值
        x_ = x;
        y_ = y;
    }

    void print(){ //成員函式
        ROS_INFO("x:%d, y:%d", x_, y_);
    }

};

class B{
public:
    int x;
    int y;
    B(){
        ROS_INFO("Initialized");
    }
    ~B(){ //解構子：程式跑完會歸還所有記憶體，當此class的變數被刪除時，就會跑解構子
        ROS_INFO("Delete %d", x);
    }
};
void transform(A a, B* b){
    
};

void test(){
    B b;
    b.x = 2;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    // radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
 
    A a; 
    B b;
    b.x = 0;
    // A a(2,3);
    // transform(a, &b);
 
    // a.print();
    test();
    
    Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f B = Eigen::Matrix3f::Identity();
    Eigen::MatrixXf temp_pose(3,1);
    Eigen::MatrixXf u(3,1);
    temp_pose(0,0) = 1;
    temp_pose(1,0) = 1;
    temp_pose(2,0) = 0;
     
    u(0,0) = 2;
    u(1,0) = 1;
    u(2,0) = 1;

    temp_pose = A * temp_pose + B * u;
    ROS_INFO("%f,%f,%f",temp_pose(0,0),temp_pose(1,0),temp_pose(2,0));

    

    return 0;
}