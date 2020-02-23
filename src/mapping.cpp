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
#include "wxx_loam/tic_toc.h"
#include <thread>

ros::Subscriber laserCloudFullSub;
ros::Subscriber laserCloudCornerSub;
ros::Subscriber laserCloudSurSub;
ros::Subscriber laserOdomSub;
ros::Publisher mapCloudPub;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudFullBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudCornerBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudSurBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> laserOdomBuf;
std::mutex mBuf;
double MAXDISTANCE = 1;

pcl::PointCloud<pcl::PointXYZI> newCornerPointCloudBakeThread;
pcl::PointCloud<pcl::PointXYZI> newSurPointCloudBakeThread;
pcl::PointCloud<pcl::PointXYZI> cornerPointsFromMapThread;
pcl::PointCloud<pcl::PointXYZI> surPointsFromMapThread;
pcl::PointCloud<pcl::PointXYZI> cornerPointsFromMapClose;
pcl::PointCloud<pcl::PointXYZI> surPointsFromMapClose;

Eigen::Quaterniond qNowThread;
Eigen::Vector3d tNowThread;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMapThread;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeSurFromMapThread;
bool threadOK = true;

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
             Eigen::Vector3d &tNowLaserOdom_, Eigen::Quaterniond &qNowLaserOdom_,
             pcl::PointCloud<pcl::PointXYZI> &newCornerPointCloudBake_, pcl::PointCloud<pcl::PointXYZI> &newSurPointCloudBake_)
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

bool findCornerPointsCorrLine(pcl::PointXYZI pointSearch_, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtreeCornerFromMap_, 
                              pcl::PointCloud<pcl::PointXYZI> &cornerPointsFromMap_, 
                              std::vector<Eigen::Vector3d> &pointFind_, double maxDis_)
{
    std::vector<int> pointFindIndex;
    std::vector<float> pointFindDistance;
    int len_ = 5;
    kdtreeCornerFromMap_.nearestKSearch(pointSearch_, len_, pointFindIndex, pointFindDistance);

    if((pointFindDistance[len_-1]>maxDis_))
        return false;

    std::vector<Eigen::Vector3d> nearPoints;
    Eigen::Vector3d meanPoint(0, 0, 0);
    for(int ii=0; ii<len_; ii++)
    {
        Eigen::Vector3d tempPoint(cornerPointsFromMap_.points[pointFindIndex[ii]].x,
                                  cornerPointsFromMap_.points[pointFindIndex[ii]].y,
                                  cornerPointsFromMap_.points[pointFindIndex[ii]].z );
        nearPoints.push_back(tempPoint);
        meanPoint = meanPoint + tempPoint;
    }

    meanPoint = meanPoint / 5;
    Eigen::Matrix3d matCov = Eigen::Matrix3d::Zero();
    for(int ii=0; ii<len_; ii++)
    {
        Eigen::Vector3d temp = nearPoints[ii] - meanPoint;
        matCov = matCov + temp * temp.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(matCov);
    if(eig.eigenvalues()[2] > 3*eig.eigenvalues()[1])
    {
        pointFind_.push_back(nearPoints[0]);
        pointFind_.push_back(nearPoints[4]);
        return true;
    }
    return false;
}

bool findSurPointsCorrPlane(pcl::PointXYZI pointSearch_, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtreeSurFromMap_, 
                            pcl::PointCloud<pcl::PointXYZI> &surPointsFromMap_, 
                            std::vector<Eigen::Vector3d> &pointFind_, double maxDis_)
{
    std::vector<int> pointFindIndex;
    std::vector<float> pointFindDistance;
    int len_ = 5;
    kdtreeSurFromMap_.nearestKSearch(pointSearch_, len_, pointFindIndex, pointFindDistance);

    if((pointFindDistance[len_-1]>maxDis_))
        return false;

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    std::vector<Eigen::Vector3d> nearPoints;
        
    for (int j = 0; j < 5; j++)
    {
        matA0(j, 0) = surPointsFromMap_.points[pointFindIndex[j]].x;
        matA0(j, 1) = surPointsFromMap_.points[pointFindIndex[j]].y;
        matA0(j, 2) = surPointsFromMap_.points[pointFindIndex[j]].z;
        Eigen::Vector3d tempPoint(surPointsFromMap_.points[pointFindIndex[j]].x,
                                surPointsFromMap_.points[pointFindIndex[j]].y,
                                surPointsFromMap_.points[pointFindIndex[j]].z );
        nearPoints.push_back(tempPoint);
        //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
    }
    // find the norm of plane
    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
    double negative_OA_dot_norm = 1 / norm.norm();
    norm.normalize();

    // Here n(pa, pb, pc) is unit norm of plane
    bool planeValid = true;
    for (int j = 0; j < 5; j++)
    {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * surPointsFromMap_.points[pointFindIndex[j]].x +
                    norm(1) * surPointsFromMap_.points[pointFindIndex[j]].y +
                    norm(2) * surPointsFromMap_.points[pointFindIndex[j]].z + negative_OA_dot_norm) > 0.2)
        {
            planeValid = false;
            return false;
        }
    }
    pointFind_.push_back(nearPoints[0]);
    pointFind_.push_back(nearPoints[2]);
    pointFind_.push_back(nearPoints[4]);
    return true;
}

void task()
{
    TicToc tt;
    for(size_t i=0; i<newCornerPointCloudBakeThread.points.size(); i++)
    {
        Eigen::Vector3d point_curr(newCornerPointCloudBakeThread.points[i].x, newCornerPointCloudBakeThread.points[i].y, newCornerPointCloudBakeThread.points[i].z);
        Eigen::Vector3d point_w = qNowThread * point_curr + tNowThread;
        //newCornerPointCloudBakeThread存储变换后的点
        //cornerPointsFromMapThread中的点同上，再降采样
        newCornerPointCloudBakeThread.points[i].x = point_w.x();
        newCornerPointCloudBakeThread.points[i].y = point_w.y();
        newCornerPointCloudBakeThread.points[i].z = point_w.z();
        cornerPointsFromMapThread.push_back(newCornerPointCloudBakeThread.points[i]);
    }
    for(size_t i=0; i<newSurPointCloudBakeThread.points.size(); i++)
    {
        Eigen::Vector3d point_curr(newSurPointCloudBakeThread.points[i].x, newSurPointCloudBakeThread.points[i].y, newSurPointCloudBakeThread.points[i].z);
        Eigen::Vector3d point_w = qNowThread * point_curr + tNowThread;
        newSurPointCloudBakeThread.points[i].x = point_w.x();
        newSurPointCloudBakeThread.points[i].y = point_w.y();
        newSurPointCloudBakeThread.points[i].z = point_w.z();
        surPointsFromMapThread.push_back(newSurPointCloudBakeThread.points[i]);
    }

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCornerThread;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurfThread;
    downSizeFilterCornerThread.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurfThread.setLeafSize(0.8, 0.8, 0.8);

    downSizeFilterCornerThread.setInputCloud(cornerPointsFromMapThread.makeShared());
    downSizeFilterCornerThread.filter(cornerPointsFromMapThread);
    downSizeFilterSurfThread.setInputCloud(surPointsFromMapThread.makeShared());
    downSizeFilterSurfThread.filter(surPointsFromMapThread);

    // kdtreeCornerFromMapThread.setInputCloud(cornerPointsFromMapThread.makeShared());
    // kdtreeSurFromMapThread.setInputCloud(surPointsFromMapThread.makeShared());

    //这里记得留一下map的point
    cornerPointsFromMapClose.clear();
    surPointsFromMapClose.clear();
    for(size_t i=0; i<cornerPointsFromMapThread.points.size(); i++)
    {
        Eigen::Vector3d point_curr(cornerPointsFromMapThread.points[i].x, cornerPointsFromMapThread.points[i].y, cornerPointsFromMapThread.points[i].z);
        double dis = (point_curr-tNowThread).norm();
        if(dis<50){
            //cornerPointsFromMapClose存cornerPointsFromMapThread中离ladar近的点
            cornerPointsFromMapClose.push_back(cornerPointsFromMapThread.points[i]);
        }
    }
    for(size_t i=0; i<surPointsFromMapThread.points.size(); i++)
    {
        Eigen::Vector3d point_curr(surPointsFromMapThread.points[i].x, surPointsFromMapThread.points[i].y, surPointsFromMapThread.points[i].z);
        double dis = (point_curr-tNowThread).norm();
        if(dis<50)
            surPointsFromMapClose.push_back(surPointsFromMapThread.points[i]);
    }

    kdtreeCornerFromMapThread.setInputCloud(cornerPointsFromMapClose.makeShared());
    kdtreeSurFromMapThread.setInputCloud(surPointsFromMapClose.makeShared());
    
    threadOK = true;
    D(tt.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mappping");
    ros::NodeHandle nh;

    laserCloudFullSub = nh.subscribe("/velodyne_cloud_3", 100, laserCloudFullHandler);
    laserCloudCornerSub = nh.subscribe("/laser_cloud_less_sharp_formap", 100, laserCloudCornerHandler);
    laserCloudSurSub = nh.subscribe("/laser_cloud_less_flat_formap", 100, laserCloudSurHandler);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_map_path", 100);
    nav_msgs::Path laserPath;

    //这里应该是最新收到的cloud，要改一下

    // laserOdomSub = nh.subscribe("/laser_odom_to_init", 100, laserOdomHandler);

    mapCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 100);

    bool firstFrame = true;
    int frameNum = 0;
    bool firstFrameThread = true;

    Eigen::Vector3d tNowLaserOdom;
    Eigen::Quaterniond qNowLaserOdom;
    std_msgs::Header mapHeader;
    pcl::PointCloud<pcl::PointXYZI> newCornerPointCloud;
    pcl::PointCloud<pcl::PointXYZI> newSurPointCloud;
    
    pcl::PointCloud<pcl::PointXYZI> cornerPointsFromMap;
    pcl::PointCloud<pcl::PointXYZI> surPointsFromMap;
    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeSurFromMap;

    pcl::PointCloud<pcl::PointXYZI> newCornerPointCloudBake;
    pcl::PointCloud<pcl::PointXYZI> newSurPointCloudBake;
    double paraQ[4] = {0, 0, 0, 1};// x y z w 
    double paraT[3] = {0.0, 0.0, 0.0};
    Eigen::Map<Eigen::Quaterniond> qNow(paraQ);
    Eigen::Map<Eigen::Vector3d> tNow(paraT);

    std::vector<std::thread> threads;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

    while (ros::ok())
    {
        ros::spinOnce();
        if(checkOK(mapHeader))
        {
            getData(newCornerPointCloud, newSurPointCloud, tNowLaserOdom, qNowLaserOdom, newCornerPointCloudBake, newSurPointCloudBake);

            if(firstFrame)
            {
                firstFrame = false;
                cornerPointsFromMap = newCornerPointCloud;
                surPointsFromMap = newSurPointCloud;
                kdtreeCornerFromMap.setInputCloud(cornerPointsFromMap.makeShared());
                kdtreeSurFromMap.setInputCloud(surPointsFromMap.makeShared());
                continue; 
            }

            downSizeFilterCorner.setInputCloud(newCornerPointCloud.makeShared());
            downSizeFilterCorner.filter(newCornerPointCloud);
            downSizeFilterSurf.setInputCloud(newSurPointCloud.makeShared());
            downSizeFilterSurf.filter(newSurPointCloud);

            newCornerPointCloudBake = newCornerPointCloud;
            newSurPointCloudBake = newSurPointCloud;

            for(int iterCount=0; iterCount<2; iterCount++)
            {
                for(size_t i=0; i<newCornerPointCloudBake.points.size(); i++)
                {
                    Eigen::Vector3d point_curr(newCornerPointCloudBake.points[i].x, newCornerPointCloudBake.points[i].y, newCornerPointCloudBake.points[i].z);
                    Eigen::Vector3d point_w = qNow * point_curr + tNow;
                    //newCornerPointCloud中存用当前转移矩阵变换后的点云
                    newCornerPointCloud.points[i].x = point_w.x();
                    newCornerPointCloud.points[i].y = point_w.y();
                    newCornerPointCloud.points[i].z = point_w.z();
                }
                for(size_t i=0; i<newSurPointCloudBake.points.size(); i++)
                {
                    Eigen::Vector3d point_curr(newSurPointCloudBake.points[i].x, newSurPointCloudBake.points[i].y, newSurPointCloudBake.points[i].z);
                    Eigen::Vector3d point_w = qNow * point_curr + tNow;
                    newSurPointCloud.points[i].x = point_w.x();
                    newSurPointCloud.points[i].y = point_w.y();
                    newSurPointCloud.points[i].z = point_w.z();
                }
                //以上不花时间
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                problem.AddParameterBlock(paraQ, 4, q_parameterization);
                problem.AddParameterBlock(paraT, 3);
                TicToc ttt;
                
                int cornerPointsCorrNum = 0;
                for(size_t i=0; i<newCornerPointCloud.size(); i++)
                {
                    pcl::PointXYZI pointSearch = newCornerPointCloud.points[i];
                    std::vector<Eigen::Vector3d> pointFind;
                    double maxDis = MAXDISTANCE;
                    //此时pointSearch是这一帧里变换后的一点，cornerPointsFromMap是离ladar近的点集
                    if(findCornerPointsCorrLine(pointSearch, kdtreeCornerFromMap, cornerPointsFromMap, pointFind, maxDis))
                    {
                        pointSearch = newCornerPointCloudBake.points[i];
                        cornerPointsCorrNum++;
                        ceres::CostFunction* lineDisCostFun =
                            new ceres::AutoDiffCostFunction<LineCostFunctor, 1, 4, 3>(new LineCostFunctor(pointSearch, pointFind));
                        problem.AddResidualBlock(lineDisCostFun, loss_function, paraQ, paraT); 
                    }         
                }

                int surPointsCorrNum = 0;
                for(size_t i=0; i<newSurPointCloud.size(); i++)
                {
                    pcl::PointXYZI pointSearch = newSurPointCloud.points[i];
                    std::vector<Eigen::Vector3d> pointFind;
                    double maxDis = MAXDISTANCE;
                    if(findSurPointsCorrPlane(pointSearch, kdtreeSurFromMap, surPointsFromMap, pointFind, maxDis))
                    {
                        pointSearch = newSurPointCloudBake.points[i];
                        surPointsCorrNum++;
                        ceres::CostFunction* planeDisCostFun =
                            new ceres::AutoDiffCostFunction<PlaneCostFunctor, 1, 4, 3>(new PlaneCostFunctor(pointSearch, pointFind)); 
                        problem.AddResidualBlock(planeDisCostFun, loss_function, paraQ, paraT); 
                    }
                }
                D(ttt.toc());

                D(cornerPointsCorrNum+surPointsCorrNum);
                if((cornerPointsCorrNum+surPointsCorrNum)<100)
                {
                    std::cout<<"Too little!"<<std::endl;
                }
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
                options.minimizer_progress_to_stdout = false;//输出到cout
                options.max_num_iterations = 4;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;//优化信息
                Solve(options, &problem, &summary);//求解!!!
            }

//每5帧之后我们把参数传入并建立新的线程
//然后线程中如果执行完了可以改变标志位，我们就等到标志位被安排了才能在第5帧后继续按照进程
            frameNum++;
            if(frameNum%4==0)
            {
                if(!firstFrameThread)
                {
                    // while(threadOK)
                    // {
                    if(threads[0].joinable())
                    {
                        threads[0].join();
                    }
                    
                    threadOK = false;
                    //更新kd-tree
                    kdtreeCornerFromMap = kdtreeCornerFromMapThread;
                    kdtreeSurFromMap = kdtreeSurFromMapThread;
                    // cornerPointsFromMap = cornerPointsFromMapThread;
                    // surPointsFromMap = surPointsFromMapThread;
                    //更新附近的特征点
                    cornerPointsFromMap = cornerPointsFromMapClose;
                    surPointsFromMap = surPointsFromMapClose;
                    // }
                }
                else
                {
                    cornerPointsFromMapThread = cornerPointsFromMap;
                    surPointsFromMapThread = surPointsFromMap;
                }
                
                newCornerPointCloudBakeThread = newCornerPointCloudBake;
                newSurPointCloudBakeThread = newSurPointCloudBake;
                qNowThread = qNow;
                tNowThread = tNow;

                sensor_msgs::PointCloud2 mapPointCloudROS;
                pcl::toROSMsg(cornerPointsFromMapThread+surPointsFromMapThread, mapPointCloudROS);
                mapPointCloudROS.header.stamp = mapHeader.stamp;
                mapPointCloudROS.header.frame_id = "/laser_init";
                mapCloudPub.publish(mapPointCloudROS);

                threads.clear();
                threads.push_back(std::thread(task));
                firstFrameThread = false;
            }

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.stamp = mapHeader.stamp;
            poseStamped.header.frame_id = "/laser_init";
            poseStamped.pose.position.x = tNow.x();
            poseStamped.pose.position.y = tNow.y();
            poseStamped.pose.position.z = tNow.z();
            poseStamped.pose.orientation.x = qNow.x();
            poseStamped.pose.orientation.y = qNow.y();
            poseStamped.pose.orientation.z = qNow.z();
            poseStamped.pose.orientation.w = qNow.w();
            laserPath.header.frame_id = "/laser_init";
            laserPath.header.stamp = mapHeader.stamp;
            laserPath.poses.push_back(poseStamped);
            pubLaserPath.publish(laserPath);
        }
    }
    return 0;
}