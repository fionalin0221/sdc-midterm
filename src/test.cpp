#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);

    return 0;
}