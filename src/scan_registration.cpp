#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;
using std::sin;
using std::cos;
using std::atan2;

ros::Publisher pubLaserCloud;

ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;


typedef pcl::PointXYZI PointType;

const int N_SCANS = 16;
const float scanPeriod = 0.1;

int framecnt = 0;



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

    bool halfPassed = false;
    //丢弃前20帧
    //...

    /* 数据格式转换 -> laserCloudIn */
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    int cloudsize = laserCloudIn.points.size();
    int count = cloudsize;

    /* 计算开始和结束点角平面角度 */
    int endind;
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    endind = cloudsize - 1;
    while(abs(laserCloudIn.points[endind].x)+abs(laserCloudIn.points[endind].y)+abs(laserCloudIn.points[endind].z)<0.001){
        endind--;
    }
    float endOri = -atan2(laserCloudIn.points[endind].y,
                        laserCloudIn.points[endind].x) + 2 * M_PI;
    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }


    /* 准备好用于存储的变量 */
    //将数据按Scan划分后的存储变量
    std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
    //将pub的数据
    pcl::PointCloud<PointType> laserCloudOut_raw;


    if(cloudsize>0){
        framecnt++;
        if(framecnt!=25){
            return;
        }
    }
    cout<<endl;
    cout<<"-------------------- ";
    cout<<"Frame No."<<framecnt<<"-------------------- ";
    cout<<endl;
    cout<<"startOri: "<<startOri*180/M_PI<<endl;
    cout<<"endOri: "<<endOri*180/M_PI<<endl;


    /* 对每个点进行处理 */
    PointType point;
    for(int i = 0; i < cloudsize; i++){
        /* 坐标系转换 */
        //目标坐标系（Lidar位姿坐标系）：y轴向上，z轴向前，x轴向左
        point.x = laserCloudIn.points[i].y;
        point.y = laserCloudIn.points[i].z;
        point.z = laserCloudIn.points[i].x;

        /* 1.计算仰角&scanID */
        int scanID;
        float angle,raw_angle;
        raw_angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
        angle = int(raw_angle + (raw_angle<0.0?-0.5:+0.5));

        if (angle > 0){
            scanID = angle;
        }
        else {
            scanID = angle + (N_SCANS - 1);
        }
        if (scanID > (N_SCANS - 1) || scanID < 0 ){
            count--;
            laserCloudOut_raw.push_back(point);
            continue;
        }
        // cout<<"-----------------"<<endl;
        // cout<<"point"<<i<<endl;
        // cout<<"axis: ("<<point.x<<", "<<point.y<<", "<<point.z<<")"<<endl;
        // cout<<"angle:"<<angle<<"; scanID:"<<scanID<<endl;




        

        /* 2.计算intensity */
        //计算当前点平面角度
        float ori = -atan2(point.x, point.z);

        if (!halfPassed) {//根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
            //确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;

            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            } 
        }




        float relTime = (ori - startOri) / (endOri - startOri);
        ///点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point.intensity = scanID + scanPeriod * relTime;


        /* stack in 将pub的变量 */
        laserCloudOut_raw.push_back(point);       
        laserCloudScans[scanID].push_back(point);

    }
    cloudsize = count;


    /* 准备好存储特征点的变量 */
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;


    /* 以下对每个scan进行处理 */
    for(int scanID = 0; scanID < N_SCANS; scanID++){
        pcl::PointCloud<PointType>::Ptr curscan(new pcl::PointCloud<PointType>());
        *curscan += laserCloudScans[scanID];
        int curscanSize = curscan->points.size();

        //点云曲率
        float cloudCurvature[3000]; //按每帧最多4万点来算，每个scan平均2500点
        //曲率点对应的序号
        int cloudSortInd[3000];
        //点是否筛选过标志：0-未筛选过，1-筛选过
        int cloudNeighborPicked[3000];
        //点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
        int cloudLabel[3000];   
        
        /* 3.计算所有点曲率 */
        for (int i = 5; i < curscanSize - 5; i++) {//使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
            float diffX = curscan->points[i - 5].x + curscan->points[i - 4].x 
                        + curscan->points[i - 3].x + curscan->points[i - 2].x 
                        + curscan->points[i - 1].x - 10 * curscan->points[i].x 
                        + curscan->points[i + 1].x + curscan->points[i + 2].x
                        + curscan->points[i + 3].x + curscan->points[i + 4].x
                        + curscan->points[i + 5].x;
            float diffY = curscan->points[i - 5].y + curscan->points[i - 4].y 
                        + curscan->points[i - 3].y + curscan->points[i - 2].y 
                        + curscan->points[i - 1].y - 10 * curscan->points[i].y 
                        + curscan->points[i + 1].y + curscan->points[i + 2].y
                        + curscan->points[i + 3].y + curscan->points[i + 4].y
                        + curscan->points[i + 5].y;
            float diffZ = curscan->points[i - 5].z + curscan->points[i - 4].z 
                        + curscan->points[i - 3].z + curscan->points[i - 2].z 
                        + curscan->points[i - 1].z - 10 * curscan->points[i].z 
                        + curscan->points[i + 1].z + curscan->points[i + 2].z
                        + curscan->points[i + 3].z + curscan->points[i + 4].z
                        + curscan->points[i + 5].z;
            //曲率计算//此处曲率相当于该点与平均中心点的距离
            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            //记录曲率点的索引
            cloudSortInd[i] = i;
            //初始时，点全未筛选过
            cloudNeighborPicked[i] = 0;
            //初始化为less flat点
            cloudLabel[i] = 0;
        }

        /* 4.筛选出坏点 */
        for (int i = 5; i < curscanSize - 6; i++) {  //与后一个点差值，所以减6
            float diffX = curscan->points[i + 1].x - curscan->points[i].x;
            float diffY = curscan->points[i + 1].y - curscan->points[i].y;
            float diffZ = curscan->points[i + 1].z - curscan->points[i].z;
            //计算有效曲率点与后一个点之间的距离平方
            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            /* 4.1.筛选面断点 */
            if (diff > 0.1) {//前提:两个点之间距离平方要大于0.1，可能该点有问题

                //点的深度
                float depth1 = sqrt(curscan->points[i].x * curscan->points[i].x + 
                                curscan->points[i].y * curscan->points[i].y +
                                curscan->points[i].z * curscan->points[i].z);

                //后一个点的深度
                float depth2 = sqrt(curscan->points[i + 1].x * curscan->points[i + 1].x + 
                                curscan->points[i + 1].y * curscan->points[i + 1].y +
                                curscan->points[i + 1].z * curscan->points[i + 1].z);

                //按照两点的深度的比例，将深度较大的点拉回后计算距离
                if (depth1 > depth2) {/* 前方的面被遮住 */
                    diffX = curscan->points[i + 1].x - curscan->points[i].x * depth2 / depth1;
                    diffY = curscan->points[i + 1].y - curscan->points[i].y * depth2 / depth1;
                    diffZ = curscan->points[i + 1].z - curscan->points[i].z * depth2 / depth1;

                    //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {//排除容易被斜面挡住的点
                        //该点及前面五个点（大致都在斜面上）全部置为筛选过
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    }
                }else{/* 后方的面被遮住 */
                    diffX = curscan->points[i + 1].x * depth1 / depth2 - curscan->points[i].x;
                    diffY = curscan->points[i + 1].y * depth1 / depth2 - curscan->points[i].y;
                    diffZ = curscan->points[i + 1].z * depth1 / depth2 - curscan->points[i].z;

                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }
            }  
            /* 4.2.筛选离群点 */
            float diffX2 = curscan->points[i].x - curscan->points[i - 1].x;
            float diffY2 = curscan->points[i].y - curscan->points[i - 1].y;
            float diffZ2 = curscan->points[i].z - curscan->points[i - 1].z;
            //与前一个点的距离平方和
            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

            //点深度的平方和
            float dis = curscan->points[i].x * curscan->points[i].x
                    + curscan->points[i].y * curscan->points[i].y
                    + curscan->points[i].z * curscan->points[i].z;

            //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
            if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
                cloudNeighborPicked[i] = 1;
            }
        }


        /* 5.点分类 */
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        /* 5.1.每个scan六等分预处理 */
        for(int segID = 0; segID < 6; segID++){
            int sp = 5 + segID*curscanSize/6; //起点
            int ep = 5 + (segID + 1)*curscanSize/6 - 1; //终点
            /* 根据曲率从大到小冒泡排序 */
            for (int k = sp + 1; k <= ep; k++) {
                for (int l = k; l >= sp + 1; l--) {
                    //如果后面曲率点大于前面，则交换
                    if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
                        int temp = cloudSortInd[l - 1];
                        cloudSortInd[l - 1] = cloudSortInd[l];
                        cloudSortInd[l] = temp;
                    }
                }
            }

            /* 5.2.大曲率点处理 */
            int largestPickedNum = 0;
            for(int i = ep; i >= sp; i--){
                int ind = cloudSortInd[i];

                //如果曲率大的点，曲率的确比较大，并且未被筛选过滤掉
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1) {
                
                    largestPickedNum++;
                    if (largestPickedNum <= 2) {//挑选曲率最大的前2个点放入sharp点集合
                        cloudLabel[ind] = 2;//2代表点曲率很大
                        cornerPointsSharp.push_back(curscan->points[ind]);
                        cornerPointsLessSharp.push_back(curscan->points[ind]);
                    } else if (largestPickedNum <= 20) {//挑选曲率最大的前20个点放入less sharp点集合
                        cloudLabel[ind] = 1;//1代表点曲率比较尖锐
                        cornerPointsLessSharp.push_back(curscan->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;//筛选标志置位

                    //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                    for (int l = 1; l <= 5; l++) {
                        float diffX = curscan->points[ind + l].x 
                                    - curscan->points[ind + l - 1].x;
                        float diffY = curscan->points[ind + l].y 
                                    - curscan->points[ind + l - 1].y;
                        float diffZ = curscan->points[ind + l].z 
                                    - curscan->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = curscan->points[ind + l].x 
                                    - curscan->points[ind + l + 1].x;
                        float diffY = curscan->points[ind + l].y 
                                    - curscan->points[ind + l + 1].y;
                        float diffZ = curscan->points[ind + l].z 
                                    - curscan->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }      


            /* 5.3.小曲率点处理 */
            int smallestPickedNum = 0;
            for(int i = sp; i <= ep; i++){
                int ind = cloudSortInd[i];
                //如果曲率的确比较小，并且未被筛选出
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1) {

                    cloudLabel[ind] = -1;//-1代表曲率很小的点
                    surfPointsFlat.push_back(curscan->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {//只选最小的四个，剩下的Label==0,就都是曲率比较小的
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {//同样防止特征点聚集
                        float diffX = curscan->points[ind + l].x 
                                    - curscan->points[ind + l - 1].x;
                        float diffY = curscan->points[ind + l].y 
                                    - curscan->points[ind + l - 1].y;
                        float diffZ = curscan->points[ind + l].z 
                                    - curscan->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = curscan->points[ind + l].x 
                                    - curscan->points[ind + l + 1].x;
                        float diffY = curscan->points[ind + l].y 
                                    - curscan->points[ind + l + 1].y;
                        float diffZ = curscan->points[ind + l].z 
                                    - curscan->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            //将剩余的点（包括之前被排除的点cloudNeighborPicked[ind] = 1）和surfPointsFlat全部归入平面点中less flat类别中
            
            for (int i = sp; i <= ep; i++) {
                if (cloudLabel[i] <= 0) {
                    surfPointsLessFlatScan->push_back(curscan->points[i]);
                }
            }
        }//六等分的一部分完毕
        //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        //less flat点汇总
        surfPointsLessFlat += surfPointsLessFlatScanDS;       
    }//一个scan完毕




    //汇总生成一个frame整体点云数据，按scan stack in
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for(int i = 0; i < N_SCANS; i++){
        *laserCloud += laserCloudScans[i];
    }
   


    /* 6.Publish */
    //sensor_msgs::PointCloud2 laserCloudOutMsg;
    //Publish laserCloudOut_raw //调试用，pub未处理的所有点
    //pcl::toROSMsg(laserCloudOut_raw, laserCloudOutMsg);
    //laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    //laserCloudOutMsg.header.frame_id = "/camera";
    //pubLaserCloud.publish(laserCloudOutMsg);

    //Publish laserCloud
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scan_regidtration");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, laserCloudHandler);


    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/raw_cloud_2", 2);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);

    ros::spin();
    
    
    return 0;
}