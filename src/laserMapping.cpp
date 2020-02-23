#include <cmath>
#include <vector>
#include <queue>
#include <mutex>
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidarFactor.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <thread>

ros::Publisher mapCloudPub;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudFullBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudCornerBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudSurBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> laserOdomBuf;
std::mutex mBuf;
double MAXDISTANCE = 1;

void laserCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mBuf.lock();
    laserCloudFullBuf.push(msg);
    mBuf.unlock();
}

void laserCloudCornerHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mBuf.lock();
    laserCloudCornerBuf.push(msg);
    mBuf.unlock();
}

void laserCloudSurHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mBuf.lock();
    laserCloudSurBuf.push(msg);
    mBuf.unlock();
}

// void laserOdomHandler(const geometry_msgs::PoseStampedConstPtr &msg)
// {
//     mBuf.lock();
//     laserOdomBuf.push(msg);
//     mBuf.unlock();
// }

bool checkOK(std_msgs::Header &header_)
{
    if(!laserCloudFullBuf.empty()&&!laserCloudCornerBuf.empty()&&!laserCloudSurBuf.empty())
    {
        double timeLaserCloudFull_ = laserCloudFullBuf.front()->header.stamp.toSec();
        double timeLaserCorner_ = laserCloudCornerBuf.front()->header.stamp.toSec();
        double timeLaserSurf_ = laserCloudSurBuf.front()->header.stamp.toSec();

        if(timeLaserCloudFull_ == timeLaserCorner_ &&
           timeLaserCloudFull_ == timeLaserSurf_)
        {
            header_.stamp = laserCloudFullBuf.front()->header.stamp;
            return true;
        }
        else
        {
            std::cout<<"*************Error in the timestamps in mapping*************"<<std::endl;
            return false;
        }
    }
    else
    {
        return false;
    }
}


void getData(pcl::PointCloud<pcl::PointXYZI> &newCornerPointCloud_, pcl::PointCloud<pcl::PointXYZI> &newSurPointCloud_, 
             Eigen::Vector3d &tNowLaserOdom_, Eigen::Quaterniond &qNowLaserOdom_)
{
    mBuf.lock();

    newCornerPointCloud_.clear();
    pcl::fromROSMsg(*laserCloudCornerBuf.front(), newCornerPointCloud_);
    newSurPointCloud_.clear();
    pcl::fromROSMsg(*laserCloudSurBuf.front(), newSurPointCloud_);
    newCornerPointCloudBake_.clear();
    pcl::fromROSMsg(*laserCloudCornerBuf.front(), newCornerPointCloudBake_);
    newSurPointCloudBake_.clear();
    pcl::fromROSMsg(*laserCloudSurBuf.front(), newSurPointCloudBake_);
    laserCloudFullBuf.pop();
    laserCloudCornerBuf.pop();
    laserCloudSurBuf.pop();

    mBuf.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mappping");
    ros::NodeHandle nh;

    ros::Subscriber laserCloudFullSub = nh.subscribe("/velodyne_cloud_3", 100, laserCloudFullHandler);
    ros::Subscriber laserCloudCornerSub = nh.subscribe("/laser_cloud_less_sharp_formap", 100, laserCloudCornerHandler);
    ros::Subscriber laserCloudSurSub = nh.subscribe("/laser_cloud_less_flat_formap", 100, laserCloudSurHandler);
    
    
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_map_path", 100);
    nav_msgs::Path laserPath;

    //这里应该是最新收到的cloud，要改一下

    // laserOdomSub = nh.subscribe("/laser_odom_to_init", 100, laserOdomHandler);

    mapCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 100);

    while (ros::ok())
    {
        ros::spinOnce();
        if(checkOK(mapHeader))
        {
            getData(newCornerPointCloud, newSurPointCloud, tNowLaserOdom, qNowLaserOdom);
        
        
        
        }
    }


}