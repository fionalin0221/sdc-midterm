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

using namespace std;
class Localizer
{
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_local;

    ros::Subscriber radar_pc_sub;
    ros::Subscriber map_sub;
    ros::Subscriber gps_sub;

    ros::Publisher radar_pc_pub;
    ros::Publisher radar_pose_pub;
    ros::Publisher path_pub;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    nav_msgs::Path path;
    tf::TransformBroadcaster br;
    
    std::string save_path;
    std::ofstream file;

    float pose_x;
    float pose_y;
    float pose_yaw;
    float gps_x;
    float gps_y;
    float gps_yaw;

    int seq = 0;
    int max_iter;
    float epsilon1;
    float epsilon2;
    float correspond;

    Eigen::Matrix4f init_guess;
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 4> C;
    Eigen::Matrix<double, 4, 4> S;
    Eigen::Matrix<double, 4, 4> R;
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 4, 1> temp_pose;
    Eigen::Matrix<double, 4, 1> z;
    Eigen::Matrix<double, 4, 4> K;
    Eigen::Matrix<double, 4, 4> I;
    // float pre_EKF_x = 0;
    // float pre_EKF_y = 0;
    float pre_ICP_x = 0.0;
    float pre_ICP_y = 0.0;
    float pre_ICP_yaw = 0;
    // float temp_ICP_x = 0;
    // float temp_ICP_y = 0;
    float vx = 0.0;
    float vy = 0.0;

    bool map_ready = false;
    bool gps_ready = false;
    bool initialized = false;

    double position_gain = 0.01;
    double velocity_gain = 0.05;

public:
    Localizer(ros::NodeHandle nh) : map_pc(new pcl::PointCloud<pcl::PointXYZI>)
    {
        map_ready = false;
        gps_ready = false;
        
        _nh = nh;
        _nh.param<string>("/save_path", save_path, "/Default/path");
        // ROS_INFO("%f",position_gain);

        init_guess.setIdentity();
        file.open(save_path);
        file << "id,x,y,yaw\n";

        radar_pc_sub = _nh.subscribe("/radar_pc", 400, &Localizer::radar_pc_callback, this);
        map_sub = _nh.subscribe("/map_pc", 1, &Localizer::map_callback, this);
        gps_sub = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);

        radar_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/tranformed_radar_pc", 1);
        radar_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/tranformed_radar_pose", 1);
        path_pub = _nh.advertise<nav_msgs::Path>("/localization_path", 1);

        A << 1,0,1,0,
             0,1,0,1,
             0,0,1,0,
             0,0,0,1;

        C << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;

        S << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;

        R << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;

        Q << position_gain,0,0,0,
             0,position_gain,0,0,
             0,0,velocity_gain,0,
             0,0,0,velocity_gain;

        I << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;
    }

    ~Localizer()
    {
        ROS_WARN("Exit Localization");
        file.close();
    }

    void gps_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_WARN("Got GPS data");
        gps_x = msg->pose.position.x;
        gps_y = msg->pose.position.y;
        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double r, p, yaw;
        m.getRPY(r, p, yaw);
        gps_yaw = yaw;
        if(!gps_ready)
        {
            pose_x = gps_x;
            pose_y = gps_y;
            pose_yaw = gps_yaw;
            gps_ready = true;
        }
    }

    void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ROS_WARN("Got Map Pointcloud");
        pcl::fromROSMsg(*msg, *map_pc);
        map_ready = true;
    }

    void radar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) //msg is get from radar.cpp
    {
        ROS_WARN("Got Radar Pointcloud");
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *radar_pc); //get the number of msg and put into radar_pc
        ROS_INFO("point size: %d", radar_pc->width);

        while(!(map_ready && gps_ready))
        { 
            ROS_WARN("Wait for map and gps ready");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if(!initialized)
        {
            /*TODO : Initialize initial guess*/
        }

        /*TODO : Implenment any scan matching base on initial guess, ICP, NDT, etc. */
        /*TODO : Assign the result to pose_x, pose_y, pose_yaw */
        /*TODO : Use result as next time initial guess */
        

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance (2);
        icp.setMaximumIterations (500);
        icp.setTransformationEpsilon (1e-8);
        icp.setEuclideanFitnessEpsilon (1e-8);
        icp.setInputSource (radar_pc);
        icp.setInputTarget (map_pc);
        Eigen::Matrix4f gauss = Eigen::Matrix4f::Identity();
        gauss(0,3) = pose_x;
        gauss(1,3) = pose_y;
        gauss.block<2, 2>(0,0) << cos(pose_yaw), -sin(pose_yaw), sin(pose_yaw),cos(pose_yaw);
        // gauss(0,0) = cos(pose_yaw);
        // gauss(0,1) = -sin(pose_yaw);
        // gauss(1,0) = sin(pose_yaw);
        // gauss(1,1) = cos(pose_yaw);
        icp.align (*output_pc, gauss);
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        // printf ("    | %6.3f %6.3f %6.3f | \n", transformation (0, 0), transformation (0, 1), transformation (0, 2));
        // printf ("R = | %6.3f %6.3f %6.3f | \n", transformation (1, 0), transformation (1, 1), transformation (1, 2));
        // printf ("    | %6.3f %6.3f %6.3f | \n", transformation (2, 0), transformation (2, 1), transformation (2, 2));
        // printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", transformation (0, 3), transformation (1, 3), transformation (2, 3));       
        
        pose_x = transformation(0,3);
        pose_y = transformation(1,3);
        // temp_ICP_x = transformation(0,3);
        // temp_ICP_y = transformation(1,3);
        Eigen::Matrix3f Rotation;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                Rotation(i,j) = transformation(i,j);
            }
        }
        Eigen::Vector3f euler_angles = Rotation.eulerAngles(0, 1, 2); 
        pose_yaw = euler_angles[2];

        ROS_INFO("original:%f,%f,%f",pose_x,pose_y,pose_yaw);
        
        if(seq == 0){
            temp_pose(0,0) = pose_x;
            temp_pose(1,0) = pose_y;
            temp_pose(2,0) = 0;
            temp_pose(3,0) = 0;
            pre_ICP_x = pose_x;
            pre_ICP_y = pose_y;
            ROS_INFO("no pass kalman filter:%f,%f,%f",pose_x,pose_y,pose_yaw);
        }else{
            z(0,0) = pose_x;
            z(1,0) = pose_y;
            z(2,0) = pose_x - pre_ICP_x;
            z(3,0) = pose_y - pre_ICP_y;
            //low pass filter
            // float filter_pose = 0.8;
            // float filter_yaw = 0.1;
            // if(seq >= 95 && seq <= 105){
            //     pose_x = pose_x * filter_pose + pre_ICP_x * (1-filter_pose);
            //     pose_y = pose_y * filter_pose + pre_ICP_y * (1-filter_pose);
            //     pose_yaw = pose_yaw * filter_yaw + pre_ICP_yaw * (1-filter_yaw);
            // }
            // pre_ICP_x = pose_x;
            // pre_ICP_y = pose_y;
            // pre_ICP_yaw = pose_yaw;
            if(seq >= 15 && seq < 25){
                position_gain *= 2.5;
                velocity_gain *= 2.5;
                Q << position_gain,0,0,0,
                     0,position_gain,0,0,
                     0,0,velocity_gain,0,
                     0,0,0,velocity_gain;
                ROS_INFO("position_gain:%f",Q(0,0));
                ROS_INFO("velocity_gain:%f",Q(2,2));
            }
            if(seq >= 50 && seq < 60){
                position_gain /= 2.5;
                velocity_gain /= 2.5;
                Q << position_gain,0,0,0,
                     0,position_gain,0,0,
                     0,0,velocity_gain,0,
                     0,0,0,velocity_gain;
                ROS_INFO("position_gain:%f",Q(0,0));
                ROS_INFO("velocity_gain:%f",Q(2,2));
            }

    
            //predict
            temp_pose = A * temp_pose;
            S = A * S * A.transpose() + R ;
            //update
            K = S * C.transpose() * (( C * S * C.transpose() + Q).inverse());
            temp_pose = temp_pose + K * ( z - C * temp_pose);
            S = (I - K * C) * S;
            pose_x = temp_pose(0,0);
            pose_y = temp_pose(1,0);
            vx = temp_pose(2,0);
            vy = temp_pose(3,0);
            ROS_INFO("after kalman filter of copy:%f,%f,%f",pose_x,pose_y,pose_yaw);
            ROS_INFO("velocity:%f,%f",vx,vy);
        }

 



        tf_brocaster(pose_x, pose_y, pose_yaw);
        radar_pose_publisher(pose_x, pose_y, pose_yaw);

        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(*radar_pc, radar_pc_msg);
        radar_pc_msg.header.stamp = ros::Time::now();
        radar_pc_msg.header.frame_id = "base_link";
        radar_pc_pub.publish(radar_pc_msg);
        ROS_INFO("Publish transformed pc");
        ROS_INFO("[seq %d] x:%.3f, y:%.3f, yaw:%.3f\n", seq, pose_x, pose_y, pose_yaw);

        file << seq << ",";
        file << pose_x << ",";
        file << pose_y << ",";
        file << pose_yaw << "\n";

        seq++;
    }

    void radar_pose_publisher(float x, float y, float yaw)
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, yaw);
        myQuaternion.normalize();

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();
        radar_pose_pub.publish(pose);
        
        path.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    void tf_brocaster(float x, float y, float yaw)
    {  
        ROS_INFO("Update map to baselink");
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

    void set_init_guess(float x, float y, float yaw)
    {
        init_guess(0, 0) = cos(yaw);
        init_guess(0, 1) = -sin(yaw);
        init_guess(0, 3) = x;

        init_guess(1, 0) = sin(yaw);
        init_guess(1, 1) = -sin(yaw);
        init_guess(1, 3) = y;
    }
};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "localizer");
    ros::NodeHandle nh;
    Localizer Localizer(nh);

    ros::spin();
    return 0;
}