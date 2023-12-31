#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h> 
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
using namespace ros;

const float range_resolution = 0.175;
Publisher radar_pub;

bool intensity_compare(pcl::PointXYZI a, pcl::PointXYZI b) 
{
    return a.intensity > b.intensity; 
}

pcl::PointCloud<pcl::PointXYZI>::Ptr create_radar_pc(Mat img)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZI>);
    
    /*TODO : Transform Polar Image to Cartisien Pointcloud*/
    double PI = 3.1415926;
    int row = img.rows;  // 2586
    int col = img.cols;  // 400
    // ROS_INFO("%d",row);
    // ROS_INFO("%d",col);
    for (int i = 4; i < row; i++) {
        for (int j = 0; j < col; j++) {
            // calculate (x,y) and define intensity
            pcl::PointXYZI point;
            point.x = (i - 4) * range_resolution * cos(-j * 2 * PI / col);
            point.y = (i - 4) * range_resolution * sin(-j * 2 * PI / col);
            point.z = 0;
            point.intensity = img.at<uchar>(i - 4, j);
            // ROS_INFO("x is [%f]",point.x);
            // ROS_INFO("y is [%f]",point.y);
            // ROS_INFO("z is [%f]",point.z);
            // ROS_INFO("i is [%f]",point.intensity);
            if(point.intensity>90){
                new_pc->points.push_back(point);
            // temp_pc->points.push_back(point);  // put the info of point to temp_pc
            }
        }
    }
    // pcl::MedianFilter<pcl::PointXYZI> median_filter;
    // median_filter.setInputCloud(temp_pc);
    // median_filter.setWindowSize(3);
    // median_filter.filter(*new_pc);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(temp_pc);
    // sor.setMeanK(10);
    // sor.setStddevMulThresh(100.0);
    // sor.filter(*new_pc);

    return new_pc;
}

void radarCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc_ptr = create_radar_pc(img);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*radar_pc_ptr, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "navtech"; //base_link
    radar_pub.publish(pc_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_polar_to_pointcloud");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
    image_transport::Subscriber sub = it.subscribe("/Navtech/Polar", 1, radarCallback);
    
    ros::spin();
    return 0;
}