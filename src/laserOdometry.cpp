#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>
#include <iostream>

#include "lidarFactor.hpp"

using namespace std;
using std::sin;
using std::cos;
using std::atan2;




typedef pcl::PointXYZI PointType;

const int N_SCANS = 16;
const float scanPeriod = 0.1;



std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

pcl::PointCloud<pcl::PointXYZI>::Ptr lastCornerPointsLessSharpPtr(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr lastSurfPointsLessFlatPtr(new pcl::PointCloud<pcl::PointXYZI>);
double MAXDISTANCE = 10;

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

bool checkOK(std_msgs::Header &header_)
{
    if(!cornerSharpBuf.empty()&&!cornerLessSharpBuf.empty()&&
       !surfFlatBuf.empty()&&!surfLessFlatBuf.empty()&&
       !fullPointsBuf.empty())
    {
        double timeCornerPointsSharp_ = cornerSharpBuf.front()->header.stamp.toSec();
        double timeCornerPointsLessSharp_ = cornerLessSharpBuf.front()->header.stamp.toSec();
        double timeSurfPointsFlat_ = surfFlatBuf.front()->header.stamp.toSec();
        double timeSurfPointsLessFlat_ = surfLessFlatBuf.front()->header.stamp.toSec();
        double timeLaserCloudFullRes_ = fullPointsBuf.front()->header.stamp.toSec();

        if((timeCornerPointsSharp_ == timeLaserCloudFullRes_)&&
            (timeCornerPointsLessSharp_ == timeLaserCloudFullRes_)&&
            (timeSurfPointsFlat_ == timeLaserCloudFullRes_)&&
            (timeSurfPointsLessFlat_ == timeLaserCloudFullRes_))
        {
            header_.stamp = fullPointsBuf.front()->header.stamp;
            return true;
        }
        else
        {
            cout<<"************* Error in the timestamps! *************"<<endl;
            return false;
        }
    }
    else
    {
        cout<<"************* Buffer Empty! *************"<<endl;
        return false;
    }
}

void getPointCloud(pcl::PointCloud<pcl::PointXYZI> &cornerPointsSharp_, pcl::PointCloud<pcl::PointXYZI> &cornerPointsLessSharp_,
                   pcl::PointCloud<pcl::PointXYZI> &surfPointsFlat_,    pcl::PointCloud<pcl::PointXYZI> &surfPointsLessFlat_,
                   pcl::PointCloud<pcl::PointXYZI> &velodyneCloud_)
{
    mBuf.lock();
    cornerPointsSharp_.clear();
    pcl::fromROSMsg(*cornerSharpBuf.front(), cornerPointsSharp_);
    cornerSharpBuf.pop();

    cornerPointsLessSharp_.clear();
    pcl::fromROSMsg(*cornerLessSharpBuf.front(), cornerPointsLessSharp_);
    cornerLessSharpBuf.pop();

    surfPointsFlat_.clear();
    pcl::fromROSMsg(*surfFlatBuf.front(), surfPointsFlat_);
    surfFlatBuf.pop();

    surfPointsLessFlat_.clear();
    pcl::fromROSMsg(*surfLessFlatBuf.front(), surfPointsLessFlat_);
    surfLessFlatBuf.pop();

    velodyneCloud_.clear();
    pcl::fromROSMsg(*fullPointsBuf.front(), velodyneCloud_);
    fullPointsBuf.pop();
    mBuf.unlock();
}

bool findCornerPointsCorr(pcl::PointXYZI pointSearch_, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr &kdtreeLast_,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &lastPointsLessSharpPtr_, std::vector<pcl::PointXYZI> &pointFind_, 
                          double maxDis_)
{
    std::vector<int> pointFindIndex;
    std::vector<float> pointFindDistance;
    int len_ = lastPointsLessSharpPtr_->size()/500;//len为什么取这么多，记得cout看看???
    kdtreeLast_->nearestKSearch(pointSearch_, len_, pointFindIndex, pointFindDistance);

    //如果最近点太远 或 找到的最近邻点个数<3??? false
    if((pointFindDistance[0]>maxDis_)||pointFindDistance.size()<3)
        return false;

    int ind_ = pointFindIndex[0];
    int scanID_ = int(lastPointsLessSharpPtr_->points[ind_].intensity);
    //第一个stack in pointFind_ 中的点是最近点
    pointFind_.push_back(lastPointsLessSharpPtr_->points[ind_]);

    for(int ii=1; ii<pointFindDistance.size(); ii++)
    {
        if(pointFindDistance[ii]>maxDis_)
            return false;
        ind_ = pointFindIndex[ii];
        int scanIDFind_ = int(lastPointsLessSharpPtr_->points[ind_].intensity);
        //确定找到的点在相邻scan
        if(abs(scanID_-scanIDFind_)<=2 && (scanID_!=scanIDFind_))
        {
            //之后stack in pointFind_ 中的点是在最近点相邻scan中的次近点
            pointFind_.push_back(lastPointsLessSharpPtr_->points[ind_]);
            return true;
        }
    }
    //未找到最近点相邻scan中的次近点 false
    return false;
}

bool findSurPointsCorr(pcl::PointXYZI pointSearch_, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr &kdtreeLast_,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr &lastPointsLessSharpPtr_, std::vector<pcl::PointXYZI> &pointFind_, 
                       double maxDis_)
{
    std::vector<int> pointFindIndex;
    std::vector<float> pointFindDistance;
    int len_ = lastPointsLessSharpPtr_->size()/500;
    kdtreeLast_->nearestKSearch(pointSearch_, len_, pointFindIndex, pointFindDistance);

    if((pointFindDistance[0]>maxDis_)||pointFindDistance.size()<3)
        return false;

    int ind_ = pointFindIndex[0];
    int scanID_ = int(lastPointsLessSharpPtr_->points[ind_].intensity);
    pointFind_.push_back(lastPointsLessSharpPtr_->points[ind_]);

    bool findInSameScan = false;
    bool findInOtherScan = false;

    for(int ii=1; ii<pointFindDistance.size(); ii++)
    {
        if(pointFindDistance[ii]>maxDis_)
            return false;

        ind_ = pointFindIndex[ii];
        int scanIDFind_ = int(lastPointsLessSharpPtr_->points[ind_].intensity);
        if(abs(scanID_-scanIDFind_)==0)
        {
            if(!findInSameScan)
            {
                pointFind_.push_back(lastPointsLessSharpPtr_->points[ind_]);
                findInSameScan = true;
                if (findInOtherScan)
                    return true;
            }
        }
        else if(abs(scanID_-scanIDFind_)<=2)
        {
            if(!findInOtherScan)
            {
                pointFind_.push_back(lastPointsLessSharpPtr_->points[ind_]);
                findInOtherScan = true;
                if (findInSameScan)
                    return true;
            }
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    // ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    // ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubVelodyneCloudMove = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_move", 100);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<geometry_msgs::PoseStamped>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
    nav_msgs::Path laserPath;
    std_msgs::Header odomHeader;

    ros::Rate rate(100);

    double paraQ[4] = {0, 0, 0, 1};// x y z w 
    double paraT[3] = {0.01, 0.01, 0.01};

    Eigen::Map<Eigen::Quaterniond> qLast2Now(paraQ);
    Eigen::Map<Eigen::Vector3d> tLast2Now(paraT);

    Eigen::Quaterniond qNow{1, 0, 0, 0};
    Eigen::Vector3d tNow{0, 0, 0};

    while (ros::ok())
    {
        ros::spinOnce();
        if(checkOK(odomHeader))
        {


            pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
            pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
            pcl::PointCloud<pcl::PointXYZI> surfPointsFlat;
            pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlat;
            pcl::PointCloud<pcl::PointXYZI> velodyneCloud;

            getPointCloud(cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat, velodyneCloud);

            // std::cout<<"========================"<<std::endl;            
            // std::cout<<lastCornerPointsLessSharpPtr->points.size()<<std::endl;
            // std::cout<<lastSurfPointsLessFlatPtr->points.size()<<std::endl;
            // std::cout<<"========================"<<std::endl;

            if(lastCornerPointsLessSharpPtr->points.size()<5||lastSurfPointsLessFlatPtr->points.size()<5)
            {
                *lastCornerPointsLessSharpPtr = cornerPointsLessSharp;
                *lastSurfPointsLessFlatPtr = surfPointsLessFlat;
                continue;
            }

            pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
            pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
            kdtreeCornerLast->setInputCloud(lastCornerPointsLessSharpPtr);
            kdtreeSurfLast->setInputCloud(lastSurfPointsLessFlatPtr);

            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization =
                new ceres::EigenQuaternionParameterization();
            problem.AddParameterBlock(paraQ, 4, q_parameterization);
            problem.AddParameterBlock(paraT, 3);

            int cornerPointsCorrNum = 0;

            for(size_t i=0; i<cornerPointsSharp.size(); i++)
            {
                pcl::PointXYZI pointSearch = cornerPointsSharp.points[i];
                std::vector<pcl::PointXYZI> pointFind;
                double maxDis = MAXDISTANCE;
                //此处省略了用IMU数据remap到sweep_begin
                if(findCornerPointsCorr(pointSearch, kdtreeCornerLast, lastCornerPointsLessSharpPtr, pointFind, maxDis))
                {
                    cornerPointsCorrNum++;
                    //把点都给拿出来，然后拿来做计算
                    ceres::CostFunction* lineDisCostFun =
                        new ceres::AutoDiffCostFunction<LineCostFunctor, 1, 4, 3>(new LineCostFunctor(pointSearch, pointFind));
                    problem.AddResidualBlock(lineDisCostFun, loss_function, paraQ, paraT); 
                }         
            }
            int surPointsCorrNum = 0;
            for(size_t i=0; i<surfPointsFlat.size(); i++)
            {
                pcl::PointXYZI pointSearch = surfPointsFlat.points[i];
                std::vector<pcl::PointXYZI> pointFind;
                double maxDis = MAXDISTANCE;
                if(findSurPointsCorr(pointSearch, kdtreeSurfLast, lastSurfPointsLessFlatPtr, pointFind, maxDis))
                {
                    surPointsCorrNum++;
                    ceres::CostFunction* planeDisCostFun =
                        new ceres::AutoDiffCostFunction<PlaneCostFunctor, 1, 4, 3>(new PlaneCostFunctor(pointSearch, pointFind)); 
                    problem.AddResidualBlock(planeDisCostFun, loss_function, paraQ, paraT); 
                }
            }

            // std::cout<<(cornerPointsCorrNum+surPointsCorrNum)<<std::endl;

            if((cornerPointsCorrNum+surPointsCorrNum)<10)
            {
                std::cout<<"Too little!"<<std::endl;
            }
          
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
            options.minimizer_progress_to_stdout = false;//输出到cout
            options.max_num_iterations = 4;
            ceres::Solver::Summary summary;//优化信息
            Solve(options, &problem, &summary);//求解!!!
            
            *lastCornerPointsLessSharpPtr = cornerPointsLessSharp;
            *lastSurfPointsLessFlatPtr = surfPointsLessFlat;

            //tqNow是累计的全局转移矩阵，tqLast2Now是两帧之间的转移矩阵
            tNow = tNow + qNow*tLast2Now;
            qNow = qNow*qLast2Now;

            //将所有点velodyneCloud原封不动pub出去，调试用
            sensor_msgs::PointCloud2 cloud_out;
            pcl::toROSMsg(velodyneCloud, cloud_out);
            cloud_out.header.stamp = odomHeader.stamp;
            cloud_out.header.frame_id = "/laser_init";
            pubLaserCloudFull.publish(cloud_out);

            //将所有点velodyneCloud按计算出的全局转移矩阵进行坐标变化，重写原变量，pub
            for(size_t i=0; i<velodyneCloud.points.size(); i++)
            {
                Eigen::Vector3d point_curr(velodyneCloud.points[i].x, velodyneCloud.points[i].y, velodyneCloud.points[i].z);
                Eigen::Vector3d point_w = qNow * point_curr + tNow;
                velodyneCloud.points[i].x = point_w.x();
                velodyneCloud.points[i].y = point_w.y();
                velodyneCloud.points[i].z = point_w.z();
            }

            pcl::toROSMsg(velodyneCloud, cloud_out);
            cloud_out.header.stamp = odomHeader.stamp;
            cloud_out.header.frame_id = "/laser_init";
            pubVelodyneCloudMove.publish(cloud_out);

            //把整个路径加上计算path
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.stamp = odomHeader.stamp;
            poseStamped.header.frame_id = "/laser_init";
            poseStamped.pose.position.x = tNow.x();
            poseStamped.pose.position.y = tNow.y();
            poseStamped.pose.position.z = tNow.z();
            poseStamped.pose.orientation.x = qNow.x();
            poseStamped.pose.orientation.y = qNow.y();
            poseStamped.pose.orientation.z = qNow.z();
            poseStamped.pose.orientation.w = qNow.w();
            pubLaserOdometry.publish(poseStamped);
            
            laserPath.header.frame_id = "/laser_init";
            laserPath.header.stamp = odomHeader.stamp;
            laserPath.poses.push_back(poseStamped);//stack in
            pubLaserPath.publish(laserPath);

        }
    }
}