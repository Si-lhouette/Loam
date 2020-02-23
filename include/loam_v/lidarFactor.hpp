#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "wxx_loam/tic_toc.h"

class LineCostFunctor {
public:
  LineCostFunctor(pcl::PointXYZI pointSearch_, std::vector<pcl::PointXYZI> pointFind_): 
    point_ii (Eigen::Vector3d(pointSearch_.x, pointSearch_.y, pointSearch_.z)),
    point_jj (Eigen::Vector3d(pointFind_[0].x, pointFind_[0].y, pointFind_[0].z)),
    point_ll (Eigen::Vector3d(pointFind_[1].x, pointFind_[1].y, pointFind_[1].z)) 
    {
      //计算分母
      downNorm = (point_jj-point_ll).norm();
      downNorm = 1/downNorm;
    }

  template <typename T>
   bool operator()(const T* q, const T* t, T* residual) const {

    // TicToc ticc;
    Eigen::Matrix<T, 3, 1> tInSweep{t[0], t[1], t[2]};
    Eigen::Quaternion<T> qInSweep{q[3], q[0], q[1], q[2]};

    Eigen::Matrix<T, 3, 1> point_i{T(point_ii.x()), T(point_ii.y()), T(point_ii.z())};  
    Eigen::Matrix<T, 3, 1> point_j{T(point_jj.x()), T(point_jj.y()), T(point_jj.z())};  
    Eigen::Matrix<T, 3, 1> point_l{T(point_ll.x()), T(point_ll.y()), T(point_ll.z())};  

    //坐标变换！
    point_i = qInSweep*point_i + tInSweep;

    //计算分子
    Eigen::Matrix<T, 3, 1> up_ = (point_i-point_j).cross(point_i-point_l);

    //计算残差
    residual[0] = up_.norm()*T(downNorm);
    // std::cout<<"tiem:"<<ticc.toc()<<std::endl;
    return true;
    }

    Eigen::Vector3d point_ii, point_jj, point_ll;
    double downNorm;
};

class PlaneCostFunctor {
public:
  PlaneCostFunctor(pcl::PointXYZI pointSearch_, std::vector<pcl::PointXYZI> pointFind_): 
    point_ii (Eigen::Vector3d(pointSearch_.x, pointSearch_.y, pointSearch_.z)),
    point_jj (Eigen::Vector3d(pointFind_[0].x, pointFind_[0].y, pointFind_[0].z)),
    point_ll (Eigen::Vector3d(pointFind_[1].x, pointFind_[1].y, pointFind_[1].z)),
    point_mm (Eigen::Vector3d(pointFind_[2].x, pointFind_[2].y, pointFind_[2].z)) 
    {
      down = (point_jj-point_ll).cross(point_jj-point_mm);
      down.normalize();
    }

  template <typename T>
   bool operator()(const T* q, const T* t, T* residual) const {
    TicToc t_opt;

    Eigen::Matrix<T, 3, 1> tInSweep{t[0], t[1], t[2]};
    Eigen::Quaternion<T> qInSweep{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> point_i{T(point_ii.x()), T(point_ii.y()), T(point_ii.z())};  
    Eigen::Matrix<T, 3, 1> point_j{T(point_jj.x()), T(point_jj.y()), T(point_jj.z())};  
    Eigen::Matrix<T, 3, 1> down_{T(down.x()), T(down.y()), T(down.z())};
    // std::cout<<"Time:"<<t_opt.toc()<<std::endl;

    point_i = qInSweep*point_i + tInSweep;
            
    residual[0] = (point_i-point_j).dot(down_);

    // std::cout<<residual[0]<<std::endl;
    return true;
    }

    Eigen::Vector3d point_ii, point_jj, point_ll, point_mm;
    Eigen::Vector3d down;
};


//这里全部都用Eigen包来操作，然后就是把收到的点然后计算转换后两者的距离即可